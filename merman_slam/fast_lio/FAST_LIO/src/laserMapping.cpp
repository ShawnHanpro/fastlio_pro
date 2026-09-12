// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <Python.h>
#include <math.h>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <so3_math.h>
#include <tf2_ros/transform_broadcaster.h>
#include <unistd.h>

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <visualization_msgs/msg/marker.hpp>

#include "IMU_Processing.hpp"
// #include <livox_interfaces/msg/custom_msg.hpp>
#include <ikd-Tree/ikd_Tree.h>

#include "preprocess.h"

// add base_link
#include <tf2_ros/static_transform_broadcaster.h>

#include <Eigen/Geometry>

// add 2D map
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <unordered_map>
#include <vector>

// add thread
#include <rclcpp/executors/multi_threaded_executor.hpp>

// add mode change
#include <std_srvs/srv/set_bool.hpp>

// pub livox scan
#include <sensor_msgs/msg/laser_scan.hpp>

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)
#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

#define SAVE_KEYFRAME 1
#define SAVE_LOG 0
#define DEBUG_LOG 0

// add cost time
class ThreadWallTimer {
public:
    ThreadWallTimer() {
        wall_begin_ = omp_get_wtime();
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_begin_);
    }

    ~ThreadWallTimer() {
        timespec cpu_end{};
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &cpu_end);

        double wall_ms = (omp_get_wtime() - wall_begin_) * 1000.0;

        double cpu_ms = (cpu_end.tv_sec - cpu_begin_.tv_sec) * 1000.0 +
                        (cpu_end.tv_nsec - cpu_begin_.tv_nsec) / 1e6;

        if (wall_ms > 50.0) {
            std::cerr << "[CALLBACK CPU/WALL]"
                      << " wall=" << wall_ms << "ms"
                      << " cpu=" << cpu_ms << "ms"
                      << " wait=" << wall_ms - cpu_ms << "ms\n";
        }
    }

private:
    double   wall_begin_;
    timespec cpu_begin_{};
};

// add mode change
enum class SystemMode { MAPPING = 0, LOCALIZATION = 1 };
std::atomic<SystemMode> system_mode(SystemMode::LOCALIZATION);

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0,
       kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN],
    s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN],
    s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0,
    kdtree_delete_counter = 0;
bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false,
     extrinsic_est_en = true, path_en = true;
/**************************/

float       res_last[100000] = {0.0};
float       DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double      time_diff_lidar_to_imu = 0.0;

mutex              mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0,
       filter_size_map_min = 0, fov_deg = 0;
// add points position filter
double mapping_min_height = 0.15;
double mapping_max_height = 1.80;

// add points range filter
double mapping_min_range = 0.10;
double mapping_max_range = 30.0;

#if SAVE_KEYFRAME
// add keyframe save
bool        sc_keyframe_save_en = true;
std::string sc_keyframe_save_dir =
    "/home/niic/slam_nav/merman_common/maps/scan_context";
double sc_keyframe_min_translation = 2.0;    // m
double sc_keyframe_min_rotation_deg = 20.0;  // degree
double sc_keyframe_voxel_size = 0.20;        // m
double sc_keyframe_min_range = 0.50;         // m
double sc_keyframe_max_range = 30.0;         // m
int    sc_keyframe_min_points = 200;
int    sc_keyframe_index = 0;
bool   sc_have_last_keyframe = false;
V3D    sc_last_keyframe_position(Zero3d);
double sc_last_keyframe_yaw = 0.0;
bool   sc_keyframe_storage_initialized = false;
#endif

double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0,
       lidar_end_time = 0, first_lidar_time = 0.0;
int effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0,
    laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool point_selected_surf[100000] = {0};
bool lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false,
     livox_scan_pub_en = false;
bool is_first_lidar = true;

vector<vector<int>>  pointSearchInd_surf;
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points;
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);

// add base_link
// base_link -> body(IMU) 静态外参
// T：IMU 原点在 base_link 坐标系中的位置
// R：IMU 坐标系相对于 base_link 坐标系的旋转
vector<double> baseToImuT{0.0, 0.0, 0.0};
vector<double> baseToImuR{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

deque<double>                                time_buffer;
deque<PointCloudXYZI::Ptr>                   lidar_buffer;
deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

// add base_link
// p_base = IMU_R_wrt_BASE * p_imu + IMU_T_wrt_BASE
V3D IMU_T_wrt_BASE(Zero3d);
M3D IMU_R_wrt_BASE(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup                                 Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom                                  state_point;
vect3                                        pos_lid;

nav_msgs::msg::Path             path;
nav_msgs::msg::Odometry         odomAftMapped;
geometry_msgs::msg::Quaternion  geoQuat;
geometry_msgs::msg::PoseStamped msg_body_pose;

// add base_link
geometry_msgs::msg::PoseStamped msg_base_pose;

// pub livox scan
// 保存最新Livox原始数据，用于scan生成
livox_ros_driver2::msg::CustomMsg::SharedPtr latest_livox_msg_ = nullptr;
std::mutex                                   livox_scan_mutex;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig) {
    flg_exit = true;
    std::cout << "catch sig %d" << sig << std::endl;
    sig_buffer.notify_all();
    rclcpp::shutdown();
}

inline void dump_lio_state_to_log(FILE *fp) {
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1),
            state_point.pos(2));                 // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // omega
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1),
            state_point.vel(2));                 // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // Acc
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1),
            state_point.bg(2));  // Bias_g
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1),
            state_point.ba(2));  // Bias_a
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1],
            state_point.grav[2]);  // Bias_a
    fprintf(fp, "\r\n");
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const *const pi, PointType *const po,
                            state_ikfom &s) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointBodyToWorld(PointType const *const pi, PointType *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                                    state_point.offset_T_L_I) +
                 state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                                    state_point.offset_T_L_I) +
                 state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po) {
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_body +
                                    state_point.offset_T_L_I) +
                 state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po) {
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I * p_body_lidar +
                   state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect() {
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++)
    // _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool         Localmap_Initialized = false;
void         lasermap_fov_segment() {
            cub_needrm.clear();
            kdtree_delete_counter = 0;
            kdtree_delete_time = 0.0;
            pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
            V3D pos_LiD = pos_lid;
            if (!Localmap_Initialized) {
                for (int i = 0; i < 3; i++) {
                    LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
                    LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
                Localmap_Initialized = true;
                return;
    }
            float dist_to_map_edge[3][2];
            bool  need_move = false;
            for (int i = 0; i < 3; i++) {
                dist_to_map_edge[i][0] =
                    fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
                dist_to_map_edge[i][1] =
                    fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
                if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
            dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE)
            need_move = true;
    }
            if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
            New_LocalMap_Points = LocalMap_Points;
            float mov_dist =
                max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                    double(DET_RANGE * (MOV_THRESHOLD - 1)));
            for (int i = 0; i < 3; i++) {
                tmp_boxpoints = LocalMap_Points;
                if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
                    New_LocalMap_Points.vertex_max[i] -= mov_dist;
                    New_LocalMap_Points.vertex_min[i] -= mov_dist;
                    tmp_boxpoints.vertex_min[i] =
                        LocalMap_Points.vertex_max[i] - mov_dist;
                    cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
                    New_LocalMap_Points.vertex_max[i] += mov_dist;
                    New_LocalMap_Points.vertex_min[i] += mov_dist;
                    tmp_boxpoints.vertex_max[i] =
                        LocalMap_Points.vertex_min[i] + mov_dist;
                    cub_needrm.push_back(tmp_boxpoints);
        }
    }
            LocalMap_Points = New_LocalMap_Points;

            points_cache_collect();
            double delete_begin = omp_get_wtime();
            if (cub_needrm.size() > 0)
        kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk_old(const sensor_msgs::msg::PointCloud2::UniquePtr msg) {
    mtx_buffer.lock();
    scan_count++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar) {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar) {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) {
    const double cur_time = get_time_sec(msg->header.stamp);
    const double preprocess_start_time = omp_get_wtime();

    const auto      callback_enter = std::chrono::steady_clock::now();
    const double    cur_stamp = get_time_sec(msg->header.stamp);
    static double   last_stamp = -1.0;
    static auto     last_callback_enter = callback_enter;
    static uint64_t callback_count = 0;
    ++callback_count;
    const double callback_gap =
        std::chrono::duration<double>(callback_enter - last_callback_enter)
            .count();
    const double stamp_gap = last_stamp > 0.0 ? cur_stamp - last_stamp : 0.0;
    if (last_stamp > 0.0 && (stamp_gap > 0.15 || callback_gap > 0.15)) {
        std::cerr << "[LIDAR RECEIVE GAP]"
                  << " count=" << callback_count << " stamp_gap=" << stamp_gap
                  << " callback_gap=" << callback_gap * 1000 << "ms "
                  << " width=" << msg->width << " height=" << msg->height
                  << " point_step=" << msg->point_step
                  << " data_bytes=" << msg->data.size()
                  << " stamp=" << std::fixed << cur_stamp * 1000 << "ms "
                  << std::endl;
    }

    last_stamp = cur_stamp;
    last_callback_enter = callback_enter;

    const double preprocess_start = omp_get_wtime();
    // 点云预处理比较耗时，不要占用数据缓冲区锁
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    p_pre->process(msg, cloud);
    const double preprocess_end = omp_get_wtime();

    const double preprocess_cost = omp_get_wtime() - preprocess_start_time;

    const double preprocess_ms = (preprocess_end - preprocess_start) * 1000.0;

    static double max_preprocess_ms = 0.0;
    max_preprocess_ms = std::max(max_preprocess_ms, preprocess_ms);
    if (preprocess_ms > 50.0) {
        std::cerr << "[LIDAR PREPROCESS SLOW]"
                  << " process cost_ms=" << preprocess_ms
                  << " max_ms=" << max_preprocess_ms
                  << " input_bytes=" << msg->data.size()
                  << " output_points=" << cloud->size() << std::endl;
    }

    {
        // 使用 RAII，离开作用域后自动解锁
        std::lock_guard<std::mutex> lock(mtx_buffer);

        ++scan_count;

        // 检测 rosbag 回放、设备重启等造成的时间戳回退
        if (!is_first_lidar && cur_time < last_timestamp_lidar) {
            std::cerr << "lidar loop back, clear buffer" << std::endl;
            // 点云和时间队列必须同步清空
            lidar_buffer.clear();
            time_buffer.clear();
        }

        if (is_first_lidar) {
            is_first_lidar = false;
        }

        lidar_buffer.push_back(std::move(cloud));
        time_buffer.push_back(cur_time);
        last_timestamp_lidar = cur_time;

        // todo: 数组越界
        s_plot11[scan_count] = preprocess_cost;
    }

    // 解锁后再唤醒处理线程，避免线程醒来后立即阻塞在锁上
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void   livox_pcl_cbk_old(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg)
// void livox_pcl_cbk(const livox_interfaces::msg::CustomMsg::UniquePtr msg)
{
    mtx_buffer.lock();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar) {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar) {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;

    if (!time_sync_en &&
        abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 &&
        !imu_buffer.empty() && !lidar_buffer.empty()) {
        printf(
            "IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf "
            "\n",
            last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg &&
        abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
        !imu_buffer.empty()) {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu =
            last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n",
               timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {
    double cur_time = get_time_sec(msg->header.stamp);

    // pub livox scan
    {
        std::lock_guard<std::mutex> lock(livox_scan_mutex);
        latest_livox_msg_ =
            std::make_shared<livox_ros_driver2::msg::CustomMsg>(*msg);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());

    p_pre->process(msg, ptr);

    mtx_buffer.lock();

    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);

    mtx_buffer.unlock();

    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in) {
    publish_count++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) -
                                     time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
        msg->header.stamp = rclcpp::Time(timediff_lidar_wrt_imu +
                                         get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu) {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool   sync_packages(MeasureGroup &meas) {
      std::lock_guard<std::mutex> lock(mtx_buffer);

      if (lidar_buffer.empty() || imu_buffer.empty()) {
          return false;
    }

      /*** push a lidar scan ***/
      if (!lidar_pushed) {
          meas.lidar = lidar_buffer.front();
          meas.lidar_beg_time = time_buffer.front();
          if (meas.lidar->points.size() <= 1)  // time too little
          {
              lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
              std::cerr << "Too few input point cloud!\n";
        } else if (meas.lidar->points.back().curvature / double(1000) <
                   0.5 * lidar_mean_scantime) {
              lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        } else {
              scan_num++;
              lidar_end_time = meas.lidar_beg_time +
                               meas.lidar->points.back().curvature / double(1000);
              lidar_mean_scantime +=
                  (meas.lidar->points.back().curvature / double(1000) -
                 lidar_mean_scantime) /
                  scan_num;
        }

          meas.lidar_end_time = lidar_end_time;

          lidar_pushed = true;
    }

      if (last_timestamp_imu < lidar_end_time) {
          return false;
    }

      /*** push imu data, and pop from imu buffer ***/
      double imu_time = get_time_sec(imu_buffer.front()->header.stamp);
      meas.imu.clear();
      while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
          imu_time = get_time_sec(imu_buffer.front()->header.stamp);
          if (imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
          imu_buffer.pop_front();
    }

      lidar_buffer.pop_front();
      time_buffer.pop_front();
      lidar_pushed = false;
      return true;
}

int  process_increments = 0;
void map_incremental() {
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++) {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]),
                         &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited) {
            const PointVector &points_near = Nearest_Points[i];
            bool               need_add = true;
            BoxPointType       Box_of_Point;
            PointType          downsample_result, mid_point;
            mid_point.x =
                floor(feats_down_world->points[i].x / filter_size_map_min) *
                    filter_size_map_min +
                0.5 * filter_size_map_min;
            mid_point.y =
                floor(feats_down_world->points[i].y / filter_size_map_min) *
                    filter_size_map_min +
                0.5 * filter_size_map_min;
            mid_point.z =
                floor(feats_down_world->points[i].z / filter_size_map_min) *
                    filter_size_map_min +
                0.5 * filter_size_map_min;
            float dist = calc_dist(feats_down_world->points[i], mid_point);
            if (fabs(points_near[0].x - mid_point.x) >
                    0.5 * filter_size_map_min &&
                fabs(points_near[0].y - mid_point.y) >
                    0.5 * filter_size_map_min &&
                fabs(points_near[0].z - mid_point.z) >
                    0.5 * filter_size_map_min) {
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++) {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist) {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        } else {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false);
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void                publish_frame_world(
                   rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
                       pubLaserCloudFull) {
    if (scan_pub_en) {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort
                                                           : feats_down_body);
        int                 size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++) {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::msg::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
        laserCloudmsg.header.frame_id = "odom";
        pubLaserCloudFull->publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    /*
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num
    >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") +
    to_string(pcd_index) + string(".pcd")); pcl::PCDWriter pcd_writer; cout <<
    "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
    */
}

void publish_frame_body(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubLaserCloudFull_body) {
    int                 size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i],
                               &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "imu_link";
    pubLaserCloudFull_body->publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubLaserCloudEffect) {
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++) {
        RGBpointBodyToWorld(&laserCloudOri->points[i],
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = get_ros_time(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "odom";
    pubLaserCloudEffect->publish(laserCloudFullRes3);
}

void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
                     pubLaserCloudMap) {
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort
                                                       : feats_down_body);
    int                 size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
        RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                            &laserCloudWorld->points[i]);
    }
    // *pcl_wait_pub += *laserCloudWorld;

    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    // pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "odom";
    pubLaserCloudMap->publish(laserCloudmsg);

    // sensor_msgs::msg::PointCloud2 laserCloudMap;
    // pcl::toROSMsg(*featsFromMap, laserCloudMap);
    // laserCloudMap.header.stamp = get_ros_time(lidar_end_time);
    // laserCloudMap.header.frame_id = "camera_init";
    // pubLaserCloudMap->publish(laserCloudMap);
}

void save_to_pcd_old() {
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(map_file_path, *pcl_wait_pub);
}

bool save_to_pcd() {
    if (ikdtree.Root_Node == nullptr) {
        std::cout << "Cannot save map: ikd-tree is empty." << std::endl;
        return false;
    }

    PointVector map_points;

    // 从当前 ikd-tree 中提取完整地图
    ikdtree.flatten(ikdtree.Root_Node, map_points, NOT_RECORD);

    if (map_points.empty()) {
        std::cout << "Cannot save map: no points in ikd-tree." << std::endl;
        return false;
    }

    PointCloudXYZI map_cloud;
    map_cloud.points = std::move(map_points);
    map_cloud.width = map_cloud.points.size();
    map_cloud.height = 1;
    map_cloud.is_dense = false;

    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setLeafSize(filter_size_map_min, filter_size_map_min,
                             filter_size_map_min);
    voxel_filter.setInputCloud(map_cloud.makeShared());

    PointCloudXYZI filtered_map;
    voxel_filter.filter(filtered_map);

    pcl::PCDWriter writer;

    const int result = writer.writeBinary(map_file_path, filtered_map);

    if (result != 0) {
        std::cout << "Failed to save PCD" << std::endl;
        return false;
    }
    std::cout << "Saved PCD: " << map_file_path.c_str() << std::endl;

    return true;
}

template <typename T>
void set_posestamp(T &out) {
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
}

// add base_link
void get_base_link_pose(V3D                &base_position_world,
                        Eigen::Quaterniond &base_orientation_world) {
    // body(IMU) 在 camera_init 中的旋转
    const M3D R_WI = state_point.rot.toRotationMatrix();
    const M3D R_WB = R_WI * IMU_R_wrt_BASE.transpose();
    base_position_world = state_point.pos - R_WB * IMU_T_wrt_BASE;
    base_orientation_world = Eigen::Quaterniond(R_WB);
    base_orientation_world.normalize();
}

// add base_link
template <typename T>
void set_base_link_posestamp(T &out) {
    V3D                base_position_world;
    Eigen::Quaterniond base_orientation_world;
    get_base_link_pose(base_position_world, base_orientation_world);
    out.pose.position.x = base_position_world.x();
    out.pose.position.y = base_position_world.y();
    out.pose.position.z = base_position_world.z();
    out.pose.orientation.x = base_orientation_world.x();
    out.pose.orientation.y = base_orientation_world.y();
    out.pose.orientation.z = base_orientation_world.z();
    out.pose.orientation.w = base_orientation_world.w();
}

#if SAVE_KEYFRAME
/* add keyframe save */ 
// ============================================================
// Scan Context keyframe
// ============================================================

double normalize_angle_sc(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double quaternion_to_yaw_sc(const Eigen::Quaterniond &q) {
    const double siny_cosp = 2.0 * (q.w() * q.z() + q.x() * q.y());

    const double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());

    return std::atan2(siny_cosp, cosy_cosp);
}

bool init_scan_context_keyframe_storage() {
    namespace fs = std::filesystem;

    const fs::path root(sc_keyframe_save_dir);
    const fs::path keyframe_dir = root / "keyframes";

    std::error_code ec;

    fs::create_directories(keyframe_dir, ec);

    if (ec) {
        std::cerr << "[SC KEYFRAME] Failed to create directory: "
                  << keyframe_dir << " error=" << ec.message() << std::endl;

        return false;
    }

    // --------------------------------------------------------
    // 查找已有最大编号，防止程序重启后覆盖原关键帧
    // --------------------------------------------------------
    int max_index = -1;

    for (const auto &entry : fs::directory_iterator(keyframe_dir, ec)) {
        if (ec) {
            break;
        }

        if (!entry.is_regular_file()) {
            continue;
        }

        if (entry.path().extension() != ".pcd") {
            continue;
        }

        try {
            const int index = std::stoi(entry.path().stem().string());

            max_index = std::max(max_index, index);

        } catch (...) {
            // 忽略非数字文件名
        }
    }

    sc_keyframe_index = max_index + 1;

    // --------------------------------------------------------
    // 第一次创建 poses.txt 时写表头
    // --------------------------------------------------------
    const fs::path poses_path = root / "poses.txt";

    if (!fs::exists(poses_path)) {
        std::ofstream pose_file(poses_path);

        if (!pose_file.is_open()) {
            std::cerr << "[SC KEYFRAME] Cannot create " << poses_path
                      << std::endl;

            return false;
        }

        pose_file << "# id stamp x y z qx qy qz qw pcd_file\n";
    }

    sc_keyframe_storage_initialized = true;

    std::cout << "[SC KEYFRAME] storage ready: " << root
              << " next_index=" << sc_keyframe_index << std::endl;

    return true;
}

void try_save_scan_context_keyframe() {
    if (!sc_keyframe_save_en) {
        return;
    }

    if (!feats_undistort || feats_undistort->empty()) {
        return;
    }

    if (!sc_keyframe_storage_initialized) {
        if (!init_scan_context_keyframe_storage()) {
            return;
        }
    }

    // ========================================================
    // 1. 获取当前 base_link 位姿
    // ========================================================

    V3D base_position_world;

    Eigen::Quaterniond base_orientation_world;

    get_base_link_pose(base_position_world, base_orientation_world);

    const double current_yaw = quaternion_to_yaw_sc(base_orientation_world);

    // ========================================================
    // 2. 判断是否达到关键帧条件
    // ========================================================

    double translation = 0.0;
    double rotation = 0.0;

    if (sc_have_last_keyframe) {
        const double dx =
            base_position_world.x() - sc_last_keyframe_position.x();

        const double dy =
            base_position_world.y() - sc_last_keyframe_position.y();

        // 地面机器人只判断 XY 距离
        translation = std::sqrt(dx * dx + dy * dy);

        rotation =
            std::abs(normalize_angle_sc(current_yaw - sc_last_keyframe_yaw));

        const double rotation_threshold =
            sc_keyframe_min_rotation_deg * M_PI / 180.0;

        // 移动和旋转都没达到阈值
        if (translation < sc_keyframe_min_translation &&
            rotation < rotation_threshold) {
            return;
        }
    }

    // ========================================================
    // 3. 当前去畸变点云:
    //
    //    LiDAR -> IMU -> base_link
    //
    // 保存的关键帧统一使用 base_link 坐标系
    // ========================================================

    PointCloudXYZI keyframe_cloud;

    keyframe_cloud.reserve(feats_undistort->size());

    for (const auto &point : feats_undistort->points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
            continue;
        }

        // LiDAR -> IMU
        const V3D p_lidar(point.x, point.y, point.z);

        const V3D p_imu =
            state_point.offset_R_L_I * p_lidar + state_point.offset_T_L_I;

        // IMU -> base_link
        const V3D p_base = IMU_R_wrt_BASE * p_imu + IMU_T_wrt_BASE;

        const double x = p_base.x();
        const double y = p_base.y();
        const double z = p_base.z();

        const double range_xy = std::sqrt(x * x + y * y);

        if (range_xy < sc_keyframe_min_range ||
            range_xy > sc_keyframe_max_range) {
            continue;
        }

        // 和现有 mapping filter 一致：
        // 删除机器人自身底盘、腰部、手臂附近点
        const bool chassis_inside_points = x > -0.60 && x < 0.60 && y > -0.50 &&
                                           y < 0.50 && z > 0.30 && z < 1.80;

        if (chassis_inside_points) {
            continue;
        }

        PointType output_point = point;

        output_point.x = x;
        output_point.y = y;
        output_point.z = z;

        keyframe_cloud.push_back(output_point);
    }

    if (static_cast<int>(keyframe_cloud.size()) < sc_keyframe_min_points) {
        std::cerr << "[SC KEYFRAME] Too few points: " << keyframe_cloud.size()
                  << std::endl;

        return;
    }

    // ========================================================
    // 4. 单独降采样，避免关键帧文件过大
    // ========================================================

    PointCloudXYZI filtered_cloud;

    if (sc_keyframe_voxel_size > 0.0) {
        pcl::VoxelGrid<PointType> voxel;

        voxel.setLeafSize(sc_keyframe_voxel_size, sc_keyframe_voxel_size,
                          sc_keyframe_voxel_size);

        voxel.setInputCloud(keyframe_cloud.makeShared());

        voxel.filter(filtered_cloud);

    } else {
        filtered_cloud = std::move(keyframe_cloud);
    }

    if (static_cast<int>(filtered_cloud.size()) < sc_keyframe_min_points) {
        std::cerr << "[SC KEYFRAME] Too few filtered points: "
                  << filtered_cloud.size() << std::endl;

        return;
    }

    // ========================================================
    // 5. 保存 PCD
    // ========================================================

    namespace fs = std::filesystem;

    char filename[64];

    std::snprintf(filename, sizeof(filename), "%06d.pcd", sc_keyframe_index);

    const fs::path relative_pcd = fs::path("keyframes") / filename;

    const fs::path pcd_path = fs::path(sc_keyframe_save_dir) / relative_pcd;

    pcl::PCDWriter writer;

    if (writer.writeBinary(pcd_path.string(), filtered_cloud) != 0) {
        std::cerr << "[SC KEYFRAME] Failed to save: " << pcd_path << std::endl;

        return;
    }

    // ========================================================
    // 6. 保存对应 map/odom -> base_link 位姿
    // ========================================================

    const fs::path poses_path = fs::path(sc_keyframe_save_dir) / "poses.txt";

    std::ofstream pose_file(poses_path, std::ios::app);

    if (!pose_file.is_open()) {
        std::cerr << "[SC KEYFRAME] Cannot open poses.txt" << std::endl;

        return;
    }

    pose_file << sc_keyframe_index << " " << std::fixed << std::setprecision(9)
              << lidar_end_time << " " << base_position_world.x() << " "
              << base_position_world.y() << " " << base_position_world.z()
              << " " << base_orientation_world.x() << " "
              << base_orientation_world.y() << " " << base_orientation_world.z()
              << " " << base_orientation_world.w() << " "
              << relative_pcd.string() << "\n";

    pose_file.close();

    // ========================================================
    // 7. 更新上一关键帧状态
    // ========================================================

    sc_last_keyframe_position = base_position_world;

    sc_last_keyframe_yaw = current_yaw;

    sc_have_last_keyframe = true;

    std::cout << "[SC KEYFRAME] saved " << filename
              << " points=" << filtered_cloud.size() << " pose=("
              << base_position_world.x() << ", " << base_position_world.y()
              << ") yaw=" << current_yaw * 180.0 / M_PI << " deg";

    if (sc_keyframe_index > 0) {
        std::cout << " translation=" << translation
                  << " rotation=" << rotation * 180.0 / M_PI << " deg";
    }

    std::cout << std::endl;

    ++sc_keyframe_index;
}

bool reset_scan_context_keyframe_storage() {
    namespace fs = std::filesystem;

    const fs::path root(sc_keyframe_save_dir);
    const fs::path keyframe_dir = root / "keyframes";
    const fs::path poses_path = root / "poses.txt";

    std::error_code ec;

    // 1. 删除整个旧 Scan Context 数据库
    if (fs::exists(root)) {
        fs::remove_all(root, ec);

        if (ec) {
            std::cerr << "[SC KEYFRAME] Failed to remove old database: " << root
                      << " error=" << ec.message() << std::endl;

            return false;
        }
    }

    // 2. 重新创建目录
    ec.clear();

    fs::create_directories(keyframe_dir, ec);

    if (ec) {
        std::cerr << "[SC KEYFRAME] Failed to create directory: "
                  << keyframe_dir << " error=" << ec.message() << std::endl;

        return false;
    }

    // 3. 重新创建 poses.txt
    std::ofstream pose_file(poses_path, std::ios::out | std::ios::trunc);

    if (!pose_file.is_open()) {
        std::cerr << "[SC KEYFRAME] Cannot create: " << poses_path << std::endl;

        return false;
    }

    pose_file << "# id stamp x y z qx qy qz qw pcd_file\n";

    pose_file.close();

    // 4. 重置内存状态
    sc_keyframe_index = 0;

    sc_have_last_keyframe = false;

    sc_last_keyframe_position = Zero3d;

    sc_last_keyframe_yaw = 0.0;

    // 目录已经初始化好了
    sc_keyframe_storage_initialized = true;

    std::cout << "[SC KEYFRAME] database reset: " << root << std::endl;

    return true;
}
/* add keyframe save */ 
#endif 

void publish_odometry_old(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
                                                    pubOdomAftMapped,
    std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br) {
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped->publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++) {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    geometry_msgs::msg::TransformStamped trans;
    trans.header.frame_id = "camera_init";
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.child_frame_id = "body";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    trans.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    trans.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    trans.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    tf_br->sendTransform(trans);
}

// add base_link
void publish_odometry_base(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
                                                    pubOdomAftMapped,
    std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br) {
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    // 世界坐标系
    odomAftMapped.header.frame_id = "camera_init";
    // 输出机器人中心位姿
    odomAftMapped.child_frame_id = "base_link";
    // 根据 IMU 位姿计算 base_link 位姿
    set_base_link_posestamp(odomAftMapped.pose);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i++) {
        const int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    pubOdomAftMapped->publish(odomAftMapped);

    // 发布动态 TF：camera_init -> base_link
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.header.frame_id = "camera_init";
    trans.child_frame_id = "base_link";
    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation = odomAftMapped.pose.pose.orientation;
    tf_br->sendTransform(trans);
}

// add odom
void publish_odometry(const rclcpp::Publisher<
                          nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
                      std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br) {
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
    // FAST-LIO 的内部世界坐标对外统一叫 odom
    odomAftMapped.header.frame_id = "odom";
    // FAST-LIO 输出机器人中心位姿
    odomAftMapped.child_frame_id = "base_link";

    // 根据 IMU 位姿和 base_link->imu_link 外参，
    // 计算 odom 坐标系下 base_link 的位姿
    set_base_link_posestamp(odomAftMapped.pose);

    auto P = kf.get_P();

    for (int i = 0; i < 6; i++) {
        const int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    pubOdomAftMapped->publish(odomAftMapped);

    // 发布动态 TF：odom -> base_link
    geometry_msgs::msg::TransformStamped trans;
    trans.header.stamp = odomAftMapped.header.stamp;
    trans.header.frame_id = "odom";
    trans.child_frame_id = "base_link";

    trans.transform.translation.x = odomAftMapped.pose.pose.position.x;
    trans.transform.translation.y = odomAftMapped.pose.pose.position.y;
    trans.transform.translation.z = odomAftMapped.pose.pose.position.z;
    trans.transform.rotation = odomAftMapped.pose.pose.orientation;

    tf_br->sendTransform(trans);
}

// add map-odom static tf
void publish_map_to_odom(
    std::unique_ptr<tf2_ros::TransformBroadcaster> &tf_br) {
    geometry_msgs::msg::TransformStamped trans;

    trans.header.stamp = get_ros_time(lidar_end_time);
    trans.header.frame_id = "map";
    trans.child_frame_id = "odom";

    trans.transform.translation.x = 0.0;
    trans.transform.translation.y = 0.0;
    trans.transform.translation.z = 0.0;

    trans.transform.rotation.x = 0.0;
    trans.transform.rotation.y = 0.0;
    trans.transform.rotation.z = 0.0;
    trans.transform.rotation.w = 1.0;

    tf_br->sendTransform(trans);
}

void publish_path_old(
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath) {
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp =
        get_ros_time(lidar_end_time);  // ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) {
        path.poses.push_back(msg_body_pose);
        pubPath->publish(path);
    }
}

// add base_link
void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath) {
    // 路径记录 base_link 位姿，而不是 IMU 位姿
    set_base_link_posestamp(msg_base_pose);
    msg_base_pose.header.stamp = get_ros_time(lidar_end_time);
    msg_base_pose.header.frame_id = "odom";

    static int path_count = 0;
    ++path_count;
    // 防止路径消息增长过快
    if (path_count % 10 == 0) {
        path.poses.push_back(msg_base_pose);
        path.header.stamp = msg_base_pose.header.stamp;
        path.header.frame_id = "odom";
        pubPath->publish(path);
    }
}

void h_share_model(state_ikfom                           &s,
                   esekfom::dyn_share_datastruct<double> &ekfom_data) {
    double match_start = omp_get_wtime();
    laserCloudOri->clear();
    corr_normvect->clear();
    total_residual = 0.0;

/** closest surface search and residual computation **/
#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < feats_down_size; i++) {
        PointType &point_body = feats_down_body->points[i];
        PointType &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) +
                     s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge) {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near,
                                   pointSearchSqDis);
            point_selected_surf[i] =
                points_near.size() < NUM_MATCH_POINTS        ? false
                : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                             : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4)
        pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f)) {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y +
                        pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9) {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++) {
        if (point_selected_surf[i]) {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num++;
        }
    }

    if (effct_feat_num < 1) {
        ekfom_data.valid = false;
        std::cerr << "No Effective Points!" << std::endl;
        // ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time += omp_get_wtime() - match_start;
    double solve_start_ = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);  // 23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++) {
        const PointType &laser_p = laserCloudOri->points[i];
        V3D              point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D              point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D              norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() * norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en) {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() *
                  C);  // s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        } else {
            ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z,
                VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

class LaserMappingNode : public rclcpp::Node {
public:
    LaserMappingNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("laser_mapping", options) {
        this->declare_parameter<bool>("publish.path_en", true);
        this->declare_parameter<bool>("publish.effect_map_en", false);
        this->declare_parameter<bool>("publish.map_en", false);
        this->declare_parameter<bool>("publish.scan_publish_en", true);
        this->declare_parameter<bool>("publish.dense_publish_en", true);
        this->declare_parameter<bool>("publish.scan_bodyframe_pub_en", true);
        this->declare_parameter<int>("max_iteration", 4);
        this->declare_parameter<string>("map_file_path", "");
        this->declare_parameter<string>("common.lid_topic", "/livox/lidar");
        this->declare_parameter<string>("common.imu_topic", "/livox/imu");
        this->declare_parameter<bool>("common.time_sync_en", false);
        this->declare_parameter<double>("common.time_offset_lidar_to_imu", 0.0);
        this->declare_parameter<double>("filter_size_corner", 0.5);
        this->declare_parameter<double>("filter_size_surf", 0.5);
        this->declare_parameter<double>("filter_size_map", 0.5);
        this->declare_parameter<double>("cube_side_length", 200.);
        this->declare_parameter<float>("mapping.det_range", 300.);
        this->declare_parameter<double>("mapping.fov_degree", 180.);
        this->declare_parameter<double>("mapping.gyr_cov", 0.1);
        this->declare_parameter<double>("mapping.acc_cov", 0.1);
        this->declare_parameter<double>("mapping.b_gyr_cov", 0.0001);
        this->declare_parameter<double>("mapping.b_acc_cov", 0.0001);
        this->declare_parameter<double>("preprocess.blind", 0.01);
        this->declare_parameter<int>("preprocess.lidar_type", AVIA);
        this->declare_parameter<int>("preprocess.scan_line", 16);
        this->declare_parameter<int>("preprocess.timestamp_unit", US);
        this->declare_parameter<int>("preprocess.scan_rate", 10);
        this->declare_parameter<int>("point_filter_num", 2);
        this->declare_parameter<bool>("feature_extract_enable", false);
        this->declare_parameter<bool>("runtime_pos_log_enable", false);
        this->declare_parameter<bool>("mapping.extrinsic_est_en", true);
        this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
        this->declare_parameter<int>("pcd_save.interval", -1);
        this->declare_parameter<vector<double>>("mapping.extrinsic_T",
                                                vector<double>());
        this->declare_parameter<vector<double>>("mapping.extrinsic_R",
                                                vector<double>());

        /** add 2D map */
        this->declare_parameter<bool>("occupancy_2d.enable", true);
        this->declare_parameter<double>("occupancy_2d.resolution", 0.05);
        this->declare_parameter<double>("occupancy_2d.min_range", 0.30);
        this->declare_parameter<double>("occupancy_2d.max_range", 15.0);
        this->declare_parameter<double>("occupancy_2d.ground_min_height",
                                        -0.15);
        this->declare_parameter<double>("occupancy_2d.obstacle_min_height",
                                        0.10);
        this->declare_parameter<double>("occupancy_2d.obstacle_max_height",
                                        1.80);
        this->declare_parameter<double>("occupancy_2d.robot_clear_radius",
                                        0.35);
        this->declare_parameter<int>("occupancy_2d.input_frame_stride", 2);
        this->declare_parameter<int>("occupancy_2d.point_step", 2);
        this->declare_parameter<int>("occupancy_2d.angular_bins", 720);
        this->declare_parameter<int>("occupancy_2d.queue_size", 2);
        this->declare_parameter<double>("occupancy_2d.update_publish_period",
                                        0.5);
        this->declare_parameter<double>("occupancy_2d.full_publish_period",
                                        10.0);
        this->declare_parameter<std::string>("occupancy_2d.frame_id", "map");

        this->declare_parameter<double>("occupancy_2d.publish_period", 1.0);
        this->declare_parameter<double>("occupancy_2d.auto_save_period", 30.0);
        this->declare_parameter<std::string>("occupancy_2d.save_prefix",
                                             "/tmp/fastlio_2d_map");
        this->declare_parameter<bool>("occupancy_2d.save_on_shutdown", true);
        /** add 2D map */

        // add base_link
        this->declare_parameter<vector<double>>("frames.base_to_imu_T",
                                                vector<double>{0.0, 0.0, 0.0});
        this->declare_parameter<vector<double>>(
            "frames.base_to_imu_R",
            vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

        // add point range filter
        this->declare_parameter<double>("mapping.mapping_min_range", 0.10);
        this->declare_parameter<double>("mapping.mapping_max_range", 30.0);

        // pub livox scan
        this->declare_parameter<bool>("publish.livox_scan_pub_en", true);

#if SAVE_KEYFRAME
        // add keyframe save
        this->declare_parameter<bool>("scan_context_keyframe.enable", true);
        this->declare_parameter<std::string>(
            "scan_context_keyframe.save_dir",
            "/home/niic/slam_nav/merman_common/maps/scan_context");
        this->declare_parameter<double>("scan_context_keyframe.min_translation",
                                        2.0);
        this->declare_parameter<double>(
            "scan_context_keyframe.min_rotation_deg", 20.0);
        this->declare_parameter<double>("scan_context_keyframe.voxel_size",
                                        0.20);
        this->declare_parameter<double>("scan_context_keyframe.min_range",
                                        0.50);
        this->declare_parameter<double>("scan_context_keyframe.max_range",
                                        30.0);
        this->declare_parameter<int>("scan_context_keyframe.min_points", 200);
#endif

        this->get_parameter_or<bool>("publish.path_en", path_en, true);
        this->get_parameter_or<bool>("publish.effect_map_en", effect_pub_en,
                                     false);
        this->get_parameter_or<bool>("publish.map_en", map_pub_en, false);
        this->get_parameter_or<bool>("publish.scan_publish_en", scan_pub_en,
                                     true);
        this->get_parameter_or<bool>("publish.dense_publish_en", dense_pub_en,
                                     true);
        this->get_parameter_or<bool>("publish.scan_bodyframe_pub_en",
                                     scan_body_pub_en, true);
        this->get_parameter_or<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
        this->get_parameter_or<string>("map_file_path", map_file_path, "");
        this->get_parameter_or<string>("common.lid_topic", lid_topic,
                                       "/livox/lidar");
        this->get_parameter_or<string>("common.imu_topic", imu_topic,
                                       "/livox/imu");
        this->get_parameter_or<bool>("common.time_sync_en", time_sync_en,
                                     false);
        this->get_parameter_or<double>("common.time_offset_lidar_to_imu",
                                       time_diff_lidar_to_imu, 0.0);
        this->get_parameter_or<double>("filter_size_corner",
                                       filter_size_corner_min, 0.5);
        this->get_parameter_or<double>("filter_size_surf", filter_size_surf_min,
                                       0.5);
        this->get_parameter_or<double>("filter_size_map", filter_size_map_min,
                                       0.5);
        this->get_parameter_or<double>("cube_side_length", cube_len, 200.f);
        this->get_parameter_or<float>("mapping.det_range", DET_RANGE, 300.f);
        this->get_parameter_or<double>("mapping.fov_degree", fov_deg, 180.f);
        this->get_parameter_or<double>("mapping.gyr_cov", gyr_cov, 0.1);
        this->get_parameter_or<double>("mapping.acc_cov", acc_cov, 0.1);
        this->get_parameter_or<double>("mapping.b_gyr_cov", b_gyr_cov, 0.0001);
        this->get_parameter_or<double>("mapping.b_acc_cov", b_acc_cov, 0.0001);
        this->get_parameter_or<double>("preprocess.blind", p_pre->blind, 0.01);
        this->get_parameter_or<int>("preprocess.lidar_type", p_pre->lidar_type,
                                    AVIA);
        this->get_parameter_or<int>("preprocess.scan_line", p_pre->N_SCANS, 16);
        this->get_parameter_or<int>("preprocess.timestamp_unit",
                                    p_pre->time_unit, US);
        this->get_parameter_or<int>("preprocess.scan_rate", p_pre->SCAN_RATE,
                                    10);
        this->get_parameter_or<int>("point_filter_num", p_pre->point_filter_num,
                                    2);
        this->get_parameter_or<bool>("feature_extract_enable",
                                     p_pre->feature_enabled, false);
        this->get_parameter_or<bool>("runtime_pos_log_enable", runtime_pos_log,
                                     0);
        this->get_parameter_or<bool>("mapping.extrinsic_est_en",
                                     extrinsic_est_en, true);
        this->get_parameter_or<bool>("pcd_save.pcd_save_en", pcd_save_en,
                                     false);
        this->get_parameter_or<int>("pcd_save.interval", pcd_save_interval, -1);
        this->get_parameter_or<vector<double>>("mapping.extrinsic_T", extrinT,
                                               vector<double>());
        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR,
                                               vector<double>());

        /**add 2D map */
        this->get_parameter("occupancy_2d.enable", occupancy_enable_);
        this->get_parameter("occupancy_2d.resolution", occupancy_resolution_);
        this->get_parameter("occupancy_2d.min_range", occupancy_min_range_);
        this->get_parameter("occupancy_2d.max_range", occupancy_max_range_);
        this->get_parameter("occupancy_2d.ground_min_height",
                            occupancy_ground_min_height_);
        this->get_parameter("occupancy_2d.obstacle_min_height",
                            occupancy_obstacle_min_height_);
        this->get_parameter("occupancy_2d.obstacle_max_height",
                            occupancy_obstacle_max_height_);
        this->get_parameter("occupancy_2d.robot_clear_radius",
                            occupancy_robot_clear_radius_);
        this->get_parameter("occupancy_2d.input_frame_stride",
                            occupancy_input_frame_stride_);
        this->get_parameter("occupancy_2d.point_step", occupancy_point_step_);
        this->get_parameter("occupancy_2d.angular_bins",
                            occupancy_angular_bins_);
        int occupancy_queue_size = 2;
        this->get_parameter("occupancy_2d.queue_size", occupancy_queue_size);
        occupancy_queue_max_size_ =
            static_cast<std::size_t>(std::max(1, occupancy_queue_size));
        this->get_parameter("occupancy_2d.update_publish_period",
                            occupancy_update_publish_period_);
        this->get_parameter("occupancy_2d.full_publish_period",
                            occupancy_full_publish_period_);
        this->get_parameter("occupancy_2d.frame_id", occupancy_frame_id_);

        this->get_parameter("occupancy_2d.publish_period",
                            occupancy_publish_period_);
        this->get_parameter("occupancy_2d.auto_save_period",
                            occupancy_auto_save_period_);
        this->get_parameter("occupancy_2d.save_prefix", occupancy_save_prefix_);
        this->get_parameter("occupancy_2d.save_on_shutdown",
                            occupancy_save_on_shutdown_);
        /**add 2D map */

        // add base_link
        this->get_parameter_or<vector<double>>(
            "frames.base_to_imu_T", baseToImuT, vector<double>{0.0, 0.0, 0.0});
        this->get_parameter_or<vector<double>>(
            "frames.base_to_imu_R", baseToImuR,
            vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0});

        // add points position filter
        this->get_parameter_or<double>("mapping.lidar_points_min_height",
                                       mapping_min_height, 0.15);
        this->get_parameter_or<double>("mapping.lidar_points_max_height",
                                       mapping_max_height, 1.80);

        // add points range filter
        this->get_parameter_or<double>("mapping.mapping_min_range",
                                       mapping_min_range, 0.10);
        this->get_parameter_or<double>("mapping.mapping_max_range",
                                       mapping_max_range, 30.0);

        // pub livox scan
        this->get_parameter_or<bool>("publish.livox_scan_pub_en",
                                     livox_scan_pub_en, true);

#if SAVE_KEYFRAME
        // add keyframe save
        this->get_parameter_or<bool>("scan_context_keyframe.enable",
                                     sc_keyframe_save_en, true);
        this->get_parameter_or<std::string>(
            "scan_context_keyframe.save_dir", sc_keyframe_save_dir,
            "/home/niic/slam_nav/merman_common/maps/scan_context");
        this->get_parameter_or<double>("scan_context_keyframe.min_translation",
                                       sc_keyframe_min_translation, 2.0);
        this->get_parameter_or<double>("scan_context_keyframe.min_rotation_deg",
                                       sc_keyframe_min_rotation_deg, 20.0);
        this->get_parameter_or<double>("scan_context_keyframe.voxel_size",
                                       sc_keyframe_voxel_size, 0.20);
        this->get_parameter_or<double>("scan_context_keyframe.min_range",
                                       sc_keyframe_min_range, 0.50);
        this->get_parameter_or<double>("scan_context_keyframe.max_range",
                                       sc_keyframe_max_range, 30.0);
        this->get_parameter_or<int>("scan_context_keyframe.min_points",
                                    sc_keyframe_min_points, 200);
#endif

        RCLCPP_INFO(this->get_logger(), "p_pre->lidar_type %d",
                    p_pre->lidar_type);

        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = "odom";

        // /*** variables definition ***/
        // int effect_feat_num = 0, frame_num = 0;
        // double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0,
        // aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0,
        // aver_time_const_H_time = 0; bool flg_EKF_converged, EKF_stop_flg = 0;

        FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
        HALF_FOV_COS = cos((FOV_DEG)*0.5 * PI_M / 180.0);

        _featsArray.reset(new PointCloudXYZI());

        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));
        downSizeFilterSurf.setLeafSize(
            filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
        downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min,
                                      filter_size_map_min);
        memset(point_selected_surf, true, sizeof(point_selected_surf));
        memset(res_last, -1000.0f, sizeof(res_last));

        Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
        Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

        // add base_link
        IMU_T_wrt_BASE << VEC_FROM_ARRAY(baseToImuT);
        IMU_R_wrt_BASE << MAT_FROM_ARRAY(baseToImuR);

        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
        p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
        p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
        p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
        p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

        fill(epsi, epsi + 23, 0.001);
        kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model,
                          NUM_MAX_ITERATIONS, epsi);

#if SAVE_LOG
        /*** debug record ***/
        // FILE *fp;
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(), "w");

        // ofstream fout_pre, fout_out, fout_dbg;
        fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), ios::out);
        fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), ios::out);
        fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"), ios::out);
        if (fout_pre && fout_out)
            cout << "~~~~" << ROOT_DIR << " file opened" << endl;
        else
            cout << "~~~~" << ROOT_DIR << " doesn't exist" << endl;
#endif

        /*** ROS subscribe initialization ***/

        /** add thread */
        // timer callback 使用独立回调组
        timer_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        // 独立激光回调组。
        // 使用 MutuallyExclusive，防止两个点云回调同时执行 p_pre->process()。
        lidar_callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions lidar_sub_options;
        lidar_sub_options.callback_group = lidar_callback_group_;

        if (p_pre->lidar_type == AVIA) {
            auto lidar_qos = rclcpp::QoS(rclcpp::KeepLast(20));

            sub_pcl_livox_ =
                this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                    lid_topic, lidar_qos, livox_pcl_cbk, lidar_sub_options);
        } else {
            // auto lidar_qos = rclcpp::SensorDataQoS();

            // // 短时间阻塞时保留更多点云。
            // // 这里只是增加缓冲能力，不能代替多线程。
            // lidar_qos.keep_last(20);

            rclcpp::QoS lidar_qos(rclcpp::KeepLast(50));

            lidar_qos.reliable();
            lidar_qos.durability_volatile();

            sub_pcl_pc_ =
                this->create_subscription<sensor_msgs::msg::PointCloud2>(
                    lid_topic, lidar_qos, standard_pcl_cbk, lidar_sub_options);
        }
        /** add thread */

        // if (p_pre->lidar_type == AVIA)
        // {
        //     sub_pcl_livox_ =
        //     this->create_subscription<livox_ros_driver2::msg::CustomMsg>(lid_topic,
        //     20, livox_pcl_cbk);
        //     // sub_pcl_livox_ =
        //     this->create_subscription<livox_interfaces::msg::CustomMsg>(lid_topic,
        //     20, livox_pcl_cbk);
        // }
        // else
        // {
        //     sub_pcl_pc_ =
        //     this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic,
        //     rclcpp::SensorDataQoS(), standard_pcl_cbk);
        // }

        // fix points pub
        rclcpp::QoS cloud_qos(rclcpp::KeepLast(1));
        cloud_qos.best_effort();
        cloud_qos.durability_volatile();
        pubLaserCloudFull_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/cloud_registered_1", cloud_qos);
        pubLaserCloudFull_body_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/cloud_registered_body_1", cloud_qos);
        pubLaserCloudEffect_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/cloud_effected_1", cloud_qos);
        pubLaserCloudMap_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/Laser_map_1", cloud_qos);

        // pub livox scan
        pub_livox_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/nav2_scan", 10);

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, 200, imu_cbk);
        // pubLaserCloudFull_ =
        //     this->create_publisher<sensor_msgs::msg::PointCloud2>(
        //         "/cloud_registered_1", 1);
        // pubLaserCloudFull_body_ =
        //     this->create_publisher<sensor_msgs::msg::PointCloud2>(
        //         "/cloud_registered_body_1", 20);
        // pubLaserCloudEffect_ =
        //     this->create_publisher<sensor_msgs::msg::PointCloud2>(
        //         "/cloud_effected_1", 20);
        // pubLaserCloudMap_ =
        //     this->create_publisher<sensor_msgs::msg::PointCloud2>(
        //         "/Laser_map_1", 1);
        pubOdomAftMapped_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/Odometry_loc", 20);
        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path_1", 20);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // add base_link
        static_tf_broadcaster_ =
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
        publish_base_to_imu_static_tf();

        // add lidar_link
        publish_base_to_lidar_static_tf();

        /**add 2D map */
        auto occupancy_map_qos =
            rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().transient_local();
        occupancy_map_pub_ =
            this->create_publisher<nav_msgs::msg::OccupancyGrid>(
                "/map_2d", occupancy_map_qos);

        if (occupancy_enable_) {
            occupancy_running_.store(true);

            occupancy_thread_ =
                std::thread(&LaserMappingNode::occupancy_worker_loop, this);

            RCLCPP_INFO(this->get_logger(), "2D occupancy mapping started");
        }
        /**add 2D map */

        // add mode change
        mode_srv_ = create_service<std_srvs::srv::SetBool>(
            "/fastlio/set_mode",
            std::bind(&LaserMappingNode::set_mode_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        //------------------------------------------------------------------------------------------------------
        auto period_ms =
            std::chrono::milliseconds(static_cast<int64_t>(1000.0 / 20.0));

        // timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms,
        // std::bind(&LaserMappingNode::timer_callback, this));

        /** add thread */
        timer_ = this->create_wall_timer(
            period_ms, std::bind(&LaserMappingNode::timer_callback, this),
            timer_callback_group_);
        /** add thread */

        auto map_period_ms =
            std::chrono::milliseconds(static_cast<int64_t>(1000.0));
        map_pub_timer_ = rclcpp::create_timer(
            this, this->get_clock(), map_period_ms,
            std::bind(&LaserMappingNode::map_publish_callback, this));

        map_save_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "map_save",
            std::bind(&LaserMappingNode::map_save_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node init finished.");
    }

    ~LaserMappingNode() {
        /**add 2D map */
        if (occupancy_enable_) {
            /*
             * 先停止建图线程。
             * 建图线程退出前会发布最后一张/map，
             * 并更新latest_map_snapshot_。
             */
            occupancy_running_.store(false);
            occupancy_queue_cv_.notify_all();

            if (occupancy_thread_.joinable()) {
                occupancy_thread_.join();
            }
        }
        /**add 2D map */

#if SAVE_LOG
        fout_out.close();
        fout_pre.close();
        fclose(fp);
#endif
    }

    /** add 2D map */
private:
    static constexpr int OCC_TILE_SIZE = 128;

    struct AngularBin {
        bool has_obstacle = false;
        bool has_free = false;

        double obstacle_range = std::numeric_limits<double>::infinity();

        double free_range = 0.0;

        Eigen::Vector2d obstacle_world = Eigen::Vector2d::Zero();

        Eigen::Vector2d free_world = Eigen::Vector2d::Zero();
    };

    struct CompactPoint {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct OccupancyFrame {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        rclcpp::Time stamp{0, 0, RCL_ROS_TIME};

        // LiDAR -> odom
        Eigen::Matrix3d R_WL = Eigen::Matrix3d::Identity();
        Eigen::Vector3d T_WL = Eigen::Vector3d::Zero();

        // base_link -> odom
        Eigen::Matrix3d R_WB = Eigen::Matrix3d::Identity();
        Eigen::Vector3d T_WB = Eigen::Vector3d::Zero();

        std::vector<CompactPoint> points;
    };

    struct OccupancyCell {
        // -20：强烈空闲
        //   0：未知倾向
        // +20：强烈占用
        int8_t score = 0;

        uint8_t observed = 0;
    };

    struct TileKey {
        int32_t x = 0;
        int32_t y = 0;

        bool operator==(const TileKey &other) const noexcept {
            return x == other.x && y == other.y;
        }
    };

    struct TileKeyHash {
        std::size_t operator()(const TileKey &key) const noexcept {
            const uint64_t packed =
                (static_cast<uint64_t>(static_cast<uint32_t>(key.x)) << 32) |
                static_cast<uint32_t>(key.y);

            return std::hash<uint64_t>{}(packed);
        }
    };

    struct OccupancyTile {
        std::array<OccupancyCell, OCC_TILE_SIZE * OCC_TILE_SIZE> cells{};
    };

    struct OccupancyGridBounds {
        int min_x = 0;
        int max_x = 0;
        int min_y = 0;
        int max_y = 0;

        uint32_t width = 0;
        uint32_t height = 0;
    };

    std::unordered_map<TileKey, std::unique_ptr<OccupancyTile>, TileKeyHash>
        occupancy_tiles_;

    double      occupancy_publish_period_ = 1.0;
    double      occupancy_auto_save_period_ = 30.0;
    std::string occupancy_save_prefix_ = "/tmp/fastlio_2d_map";
    bool        occupancy_save_on_shutdown_ = true;

    std::mutex                                          latest_map_mutex_;
    std::shared_ptr<const nav_msgs::msg::OccupancyGrid> latest_map_snapshot_;

    std::atomic<bool> occupancy_running_{false};
    std::thread       occupancy_thread_;

    std::mutex                 occupancy_queue_mutex_;
    std::condition_variable    occupancy_queue_cv_;
    std::deque<OccupancyFrame> occupancy_queue_;

    std::size_t occupancy_queue_max_size_ = 2;

    std::atomic<uint64_t> occupancy_dropped_frames_{0};

    bool occupancy_enable_ = true;

    double occupancy_resolution_ = 0.05;

    double occupancy_min_range_ = 0.30;
    double occupancy_max_range_ = 15.0;

    double occupancy_ground_min_height_ = -0.15;
    double occupancy_obstacle_min_height_ = 0.10;
    double occupancy_obstacle_max_height_ = 1.80;

    double occupancy_robot_clear_radius_ = 0.35;

    int occupancy_input_frame_stride_ = 2;
    int occupancy_point_step_ = 2;
    int occupancy_angular_bins_ = 720;

    double occupancy_update_publish_period_ = 0.5;
    double occupancy_full_publish_period_ = 10.0;

    std::string occupancy_frame_id_ = "map";

    /**add 2D map */
    // 动态获取瓦片
    bool tile_bounds_initialized_ = false;

    int min_tile_x_ = 0;
    int max_tile_x_ = 0;
    int min_tile_y_ = 0;
    int max_tile_y_ = 0;

    bool map_geometry_dirty_ = true;
    bool full_map_published_ = false;

    // C++ 的整数除法对负数不是数学意义上的向下取整，所以需要单独处理
    static int floor_div(int value, int divisor) {
        int quotient = value / divisor;
        int remainder = value % divisor;

        if (remainder < 0) {
            --quotient;
        }

        return quotient;
    }

    static int positive_mod(int value, int divisor) {
        int result = value % divisor;

        if (result < 0) {
            result += divisor;
        }

        return result;
    }

    int world_to_global_cell(double value) const {
        return static_cast<int>(std::floor(value / occupancy_resolution_));
    }

    // 获取或创建瓦片
    OccupancyTile &get_or_create_tile(int tile_x, int tile_y) {
        const TileKey key{static_cast<int32_t>(tile_x),
                          static_cast<int32_t>(tile_y)};

        auto it = occupancy_tiles_.find(key);

        if (it != occupancy_tiles_.end()) {
            return *(it->second);
        }

        auto new_tile = std::make_unique<OccupancyTile>();

        OccupancyTile *tile_ptr = new_tile.get();

        occupancy_tiles_.emplace(key, std::move(new_tile));

        if (!tile_bounds_initialized_) {
            min_tile_x_ = max_tile_x_ = tile_x;
            min_tile_y_ = max_tile_y_ = tile_y;
            tile_bounds_initialized_ = true;
        } else {
            min_tile_x_ = std::min(min_tile_x_, tile_x);
            max_tile_x_ = std::max(max_tile_x_, tile_x);
            min_tile_y_ = std::min(min_tile_y_, tile_y);
            max_tile_y_ = std::max(max_tile_y_, tile_y);
        }

        // 地图尺寸或原点发生变化，必须重新发完整地图
        map_geometry_dirty_ = true;

        return *tile_ptr;
    }

    // 获取某个全局栅格
    OccupancyCell &get_or_create_cell(int global_x, int global_y) {
        const int      tile_x = floor_div(global_x, OCC_TILE_SIZE);
        const int      tile_y = floor_div(global_y, OCC_TILE_SIZE);
        const int      local_x = positive_mod(global_x, OCC_TILE_SIZE);
        const int      local_y = positive_mod(global_y, OCC_TILE_SIZE);
        OccupancyTile &tile = get_or_create_tile(tile_x, tile_y);

        const std::size_t index =
            static_cast<std::size_t>(local_y) * OCC_TILE_SIZE +
            static_cast<std::size_t>(local_x);

        return tile.cells[index];
    }

    bool dirty_region_valid_ = false;

    // 记录变化区域
    int dirty_min_x_ = 0;
    int dirty_max_x_ = 0;
    int dirty_min_y_ = 0;
    int dirty_max_y_ = 0;

    void mark_cell_dirty(int global_x, int global_y) {
        if (!dirty_region_valid_) {
            dirty_min_x_ = dirty_max_x_ = global_x;
            dirty_min_y_ = dirty_max_y_ = global_y;
            dirty_region_valid_ = true;
            return;
        }

        dirty_min_x_ = std::min(dirty_min_x_, global_x);
        dirty_max_x_ = std::max(dirty_max_x_, global_x);
        dirty_min_y_ = std::min(dirty_min_y_, global_y);
        dirty_max_y_ = std::max(dirty_max_y_, global_y);
    }

    // 更新空闲
    void update_free_cell(int global_x, int global_y) {
        OccupancyCell &cell = get_or_create_cell(global_x, global_y);

        cell.observed = 1;
        const int new_score = static_cast<int>(cell.score) - 1;
        cell.score = static_cast<int8_t>(std::max(-20, new_score));
        mark_cell_dirty(global_x, global_y);
    }

    // 更新障碍
    void update_occupied_cell(int global_x, int global_y) {
        OccupancyCell &cell = get_or_create_cell(global_x, global_y);

        cell.observed = 1;
        const int new_score = static_cast<int>(cell.score) + 4;
        cell.score = static_cast<int8_t>(std::min(20, new_score));
        mark_cell_dirty(global_x, global_y);
    }

    // Bresenham
    void raytrace_free(int start_x, int start_y, int end_x, int end_y,
                       bool include_endpoint) {
        int x = start_x;
        int y = start_y;

        const int dx = std::abs(end_x - start_x);
        const int dy = std::abs(end_y - start_y);

        const int step_x = start_x < end_x ? 1 : -1;
        const int step_y = start_y < end_y ? 1 : -1;

        int error = dx - dy;

        while (true) {
            const bool endpoint = x == end_x && y == end_y;

            if (!endpoint || include_endpoint) {
                update_free_cell(x, y);
            }

            if (endpoint) {
                break;
            }

            const int error_twice = error * 2;

            if (error_twice > -dy) {
                error -= dy;
                x += step_x;
            }

            if (error_twice < dx) {
                error += dx;
                y += step_y;
            }
        }
    }

    void enqueue_occupancy_frame() {
        if (!occupancy_enable_) {
            return;
        }

        static uint64_t input_frame_counter = 0;
        ++input_frame_counter;

        const int frame_stride = std::max(1, occupancy_input_frame_stride_);

        if (input_frame_counter % static_cast<uint64_t>(frame_stride) != 0) {
            return;
        }

        OccupancyFrame frame;

        frame.stamp = get_ros_time(lidar_end_time);

        const Eigen::Matrix3d R_WI = state_point.rot.toRotationMatrix();

        const Eigen::Matrix3d R_IL =
            state_point.offset_R_L_I.toRotationMatrix();

        // LiDAR到odom
        frame.R_WL = R_WI * R_IL;

        frame.T_WL = state_point.pos + R_WI * state_point.offset_T_L_I;

        // base_link到odom
        frame.R_WB = R_WI * IMU_R_wrt_BASE.transpose();

        frame.T_WB = state_point.pos - frame.R_WB * IMU_T_wrt_BASE;

        const int point_step = std::max(1, occupancy_point_step_);

        frame.points.reserve(
            feats_down_body->size() / static_cast<std::size_t>(point_step) + 1);

        for (std::size_t i = 0; i < feats_down_body->size();
             i += static_cast<std::size_t>(point_step)) {
            const PointType &point = feats_down_body->points[i];

            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z)) {
                continue;
            }

            const double horizontal_range = std::hypot(
                static_cast<double>(point.x), static_cast<double>(point.y));

            if (horizontal_range < occupancy_min_range_ ||
                horizontal_range > occupancy_max_range_) {
                continue;
            }

            frame.points.push_back(CompactPoint{point.x, point.y, point.z});
        }

        if (frame.points.empty()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(occupancy_queue_mutex_);

            while (occupancy_queue_.size() >= occupancy_queue_max_size_) {
                occupancy_queue_.pop_front();
                ++occupancy_dropped_frames_;
            }

            occupancy_queue_.push_back(std::move(frame));
        }

        occupancy_queue_cv_.notify_one();
    }

    // 压缩射线
    void integrate_occupancy_frame(const OccupancyFrame &frame) {
        const int bin_count = std::max(90, occupancy_angular_bins_);

        std::vector<AngularBin> bins(static_cast<std::size_t>(bin_count));

        constexpr double TWO_PI = 2.0 * M_PI;

        for (const CompactPoint &point : frame.points) {
            const Eigen::Vector3d point_lidar(point.x, point.y, point.z);

            const double horizontal_range =
                std::hypot(point_lidar.x(), point_lidar.y());

            if (horizontal_range < occupancy_min_range_ ||
                horizontal_range > occupancy_max_range_) {
                continue;
            }

            const Eigen::Vector3d point_world =
                frame.R_WL * point_lidar + frame.T_WL;

            // 转到base_link，得到相对机器人底面的高度
            const Eigen::Vector3d point_base =
                frame.R_WB.transpose() * (point_world - frame.T_WB);

            const double relative_height = point_base.z();

            double angle = std::atan2(point_lidar.y(), point_lidar.x());

            int bin_index = static_cast<int>(std::floor(
                (angle + M_PI) / TWO_PI * static_cast<double>(bin_count)));

            bin_index = std::clamp(bin_index, 0, bin_count - 1);

            AngularBin &bin = bins[static_cast<std::size_t>(bin_index)];

            if (relative_height >= occupancy_obstacle_min_height_ &&
                relative_height <= occupancy_obstacle_max_height_) {
                if (horizontal_range < bin.obstacle_range) {
                    bin.has_obstacle = true;
                    bin.obstacle_range = horizontal_range;

                    bin.obstacle_world = point_world.head<2>();
                }

                continue;
            }

            if (relative_height >= occupancy_ground_min_height_ &&
                relative_height < occupancy_obstacle_min_height_) {
                if (horizontal_range > bin.free_range) {
                    bin.has_free = true;
                    bin.free_range = horizontal_range;

                    bin.free_world = point_world.head<2>();
                }
            }
        }

        const int sensor_x = world_to_global_cell(frame.T_WL.x());

        const int sensor_y = world_to_global_cell(frame.T_WL.y());

        for (const AngularBin &bin : bins) {
            if (bin.has_obstacle) {
                const int end_x = world_to_global_cell(bin.obstacle_world.x());

                const int end_y = world_to_global_cell(bin.obstacle_world.y());

                raytrace_free(sensor_x, sensor_y, end_x, end_y, false);

                update_occupied_cell(end_x, end_y);

                continue;
            }

            if (bin.has_free) {
                const int end_x = world_to_global_cell(bin.free_world.x());

                const int end_y = world_to_global_cell(bin.free_world.y());

                raytrace_free(sensor_x, sensor_y, end_x, end_y, true);
            }
        }

        clear_robot_area(frame);
    }

    // 清除自身区域
    void clear_robot_area(const OccupancyFrame &frame) {
        const int center_x = world_to_global_cell(frame.T_WB.x());

        const int center_y = world_to_global_cell(frame.T_WB.y());

        const int radius_cells = static_cast<int>(
            std::ceil(occupancy_robot_clear_radius_ / occupancy_resolution_));

        const int radius_squared = radius_cells * radius_cells;

        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                if (dx * dx + dy * dy > radius_squared) {
                    continue;
                }

                update_free_cell(center_x + dx, center_y + dy);
            }
        }
    }

    // 独立地图线程
    void occupancy_worker_loop() {
        auto last_publish_time = std::chrono::steady_clock::now();

        bool map_changed = false;

        while (true) {
            OccupancyFrame frame;
            bool           has_frame = false;

            {
                std::unique_lock<std::mutex> lock(occupancy_queue_mutex_);

                occupancy_queue_cv_.wait_for(
                    lock, std::chrono::milliseconds(100), [this]() {
                        return !occupancy_running_.load() ||
                               !occupancy_queue_.empty();
                    });

                /*
                 * 优先把队列中的数据处理完成。
                 */
                if (!occupancy_queue_.empty()) {
                    frame = std::move(occupancy_queue_.front());

                    occupancy_queue_.pop_front();

                    has_frame = true;
                } else if (!occupancy_running_.load()) {
                    break;
                }
            }

            if (has_frame) {
                integrate_occupancy_frame(frame);
                map_changed = true;
            }

            const auto now = std::chrono::steady_clock::now();

            const double publish_elapsed =
                std::chrono::duration<double>(now - last_publish_time).count();

            /*
             * 只有地图发生变化并且达到发布周期，
             * 才构建完整OccupancyGrid。
             */
            if (map_changed && publish_elapsed >= occupancy_publish_period_) {
                if (publish_full_occupancy_map()) {
                    map_changed = false;
                    last_publish_time = now;
                }
            }
        }

        /*
         * 线程退出前发布最后一张地图。
         */
        if (!occupancy_tiles_.empty()) {
            publish_full_occupancy_map();
        }

        RCLCPP_INFO(this->get_logger(), "2D occupancy worker stopped");
    }

    // 计算所有瓦片覆盖的地图范围
    bool get_occupancy_grid_bounds(OccupancyGridBounds &bounds) const {
        if (!tile_bounds_initialized_ || occupancy_tiles_.empty()) {
            return false;
        }

        bounds.min_x = min_tile_x_ * OCC_TILE_SIZE;

        bounds.max_x = (max_tile_x_ + 1) * OCC_TILE_SIZE - 1;

        bounds.min_y = min_tile_y_ * OCC_TILE_SIZE;

        bounds.max_y = (max_tile_y_ + 1) * OCC_TILE_SIZE - 1;

        const int64_t width = static_cast<int64_t>(bounds.max_x) -
                              static_cast<int64_t>(bounds.min_x) + 1;

        const int64_t height = static_cast<int64_t>(bounds.max_y) -
                               static_cast<int64_t>(bounds.min_y) + 1;

        if (width <= 0 || height <= 0) {
            return false;
        }

        if (width > std::numeric_limits<uint32_t>::max() ||
            height > std::numeric_limits<uint32_t>::max()) {
            return false;
        }

        bounds.width = static_cast<uint32_t>(width);

        bounds.height = static_cast<uint32_t>(height);

        return true;
    }

    // 读取已有栅格
    const OccupancyCell *find_occupancy_cell(int global_x, int global_y) const {
        const int tile_x = floor_div(global_x, OCC_TILE_SIZE);

        const int tile_y = floor_div(global_y, OCC_TILE_SIZE);

        const TileKey key{static_cast<int32_t>(tile_x),
                          static_cast<int32_t>(tile_y)};

        const auto iterator = occupancy_tiles_.find(key);

        if (iterator == occupancy_tiles_.end()) {
            return nullptr;
        }

        const int local_x = positive_mod(global_x, OCC_TILE_SIZE);

        const int local_y = positive_mod(global_y, OCC_TILE_SIZE);

        const std::size_t index =
            static_cast<std::size_t>(local_y) * OCC_TILE_SIZE +
            static_cast<std::size_t>(local_x);

        return &iterator->second->cells[index];
    }

    // 地图编码
    int8_t encode_occupancy_cell(const OccupancyCell *cell) const {
        if (cell == nullptr || cell->observed == 0) {
            return -1;
        }

        if (cell->score >= 4) {
            return 100;
        }

        if (cell->score <= -2) {
            return 0;
        }

        return -1;
    }

    // 构造完整 OccupancyGrid
    bool build_full_occupancy_grid(nav_msgs::msg::OccupancyGrid &map) {
        OccupancyGridBounds bounds;

        if (!get_occupancy_grid_bounds(bounds)) {
            return false;
        }

        const uint64_t cell_count = static_cast<uint64_t>(bounds.width) *
                                    static_cast<uint64_t>(bounds.height);

        constexpr uint64_t MAX_MAP_CELLS = 100000000ULL;

        if (cell_count > MAX_MAP_CELLS) {
            RCLCPP_ERROR(this->get_logger(), "2D map too large: %u x %u",
                         bounds.width, bounds.height);

            return false;
        }

        map.header.stamp = this->get_clock()->now();

        /*
         * FAST-LIO内部地图使用odom坐标。
         * 建图时需要保证map -> odom为单位变换。
         */
        map.header.frame_id = occupancy_frame_id_;

        map.info.map_load_time = map.header.stamp;

        map.info.resolution = static_cast<float>(occupancy_resolution_);

        map.info.width = bounds.width;

        map.info.height = bounds.height;

        map.info.origin.position.x =
            static_cast<double>(bounds.min_x) * occupancy_resolution_;

        map.info.origin.position.y =
            static_cast<double>(bounds.min_y) * occupancy_resolution_;

        map.info.origin.position.z = 0.0;

        map.info.origin.orientation.x = 0.0;
        map.info.origin.orientation.y = 0.0;
        map.info.origin.orientation.z = 0.0;
        map.info.origin.orientation.w = 1.0;

        map.data.assign(static_cast<std::size_t>(cell_count),
                        static_cast<int8_t>(-1));

        /*
         * 只遍历已经分配的瓦片。
         */
        for (const auto &tile_entry : occupancy_tiles_) {
            const TileKey &tile_key = tile_entry.first;

            const OccupancyTile &tile = *tile_entry.second;

            const int tile_global_x =
                static_cast<int>(tile_key.x) * OCC_TILE_SIZE;

            const int tile_global_y =
                static_cast<int>(tile_key.y) * OCC_TILE_SIZE;

            for (int local_y = 0; local_y < OCC_TILE_SIZE; ++local_y) {
                const int global_y = tile_global_y + local_y;

                const int map_y = global_y - bounds.min_y;

                if (map_y < 0 || map_y >= static_cast<int>(bounds.height)) {
                    continue;
                }

                for (int local_x = 0; local_x < OCC_TILE_SIZE; ++local_x) {
                    const int global_x = tile_global_x + local_x;

                    const int map_x = global_x - bounds.min_x;

                    if (map_x < 0 || map_x >= static_cast<int>(bounds.width)) {
                        continue;
                    }

                    const std::size_t tile_index =
                        static_cast<std::size_t>(local_y) * OCC_TILE_SIZE +
                        static_cast<std::size_t>(local_x);

                    const std::size_t map_index =
                        static_cast<std::size_t>(map_y) * bounds.width +
                        static_cast<std::size_t>(map_x);

                    map.data[map_index] =
                        encode_occupancy_cell(&tile.cells[tile_index]);
                }
            }
        }

        return true;
    }

    // 发布rviz
    bool publish_full_occupancy_map() {
        // add mode change
        if (system_mode != SystemMode::MAPPING) return false;

        if (!occupancy_enable_ || !occupancy_map_pub_) {
            return false;
        }

        auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();

        if (!build_full_occupancy_grid(*map)) {
            return false;
        }

        occupancy_map_pub_->publish(*map);

        /*
         * 保存线程使用同一张不可修改的地图快照。
         */
        {
            std::lock_guard<std::mutex> lock(latest_map_mutex_);

            latest_map_snapshot_ = map;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Published 2D map: "
                             "%u x %u, resolution=%.3f, "
                             "origin=[%.2f, %.2f], tiles=%zu",
                             map->info.width, map->info.height,
                             map->info.resolution, map->info.origin.position.x,
                             map->info.origin.position.y,
                             occupancy_tiles_.size());

        return true;
    }

    bool save_occupancy_pgm(const nav_msgs::msg::OccupancyGrid &map,
                            const std::filesystem::path        &path) {
        std::ofstream output(path, std::ios::binary | std::ios::trunc);

        if (!output.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open PGM: %s",
                         path.string().c_str());

            return false;
        }

        output << "P5\n"
               << "# FAST-LIO 2D map\n"
               << map.info.width << " " << map.info.height << "\n255\n";

        /*
         * OccupancyGrid从左下角开始，
         * PGM从左上角开始，因此Y轴倒序写入。
         */
        for (int y = static_cast<int>(map.info.height) - 1; y >= 0; --y) {
            for (uint32_t x = 0; x < map.info.width; ++x) {
                const std::size_t index =
                    static_cast<std::size_t>(y) * map.info.width +
                    static_cast<std::size_t>(x);

                const int8_t value = map.data[index];

                uint8_t pixel = 205;

                if (value < 0) {
                    // 未知：灰色
                    pixel = 205;
                } else if (value >= 65) {
                    // 障碍：黑色
                    pixel = 0;
                } else {
                    // 空闲：白色
                    pixel = 254;
                }

                output.write(reinterpret_cast<const char *>(&pixel),
                             sizeof(pixel));
            }
        }

        output.flush();
        return output.good();
    }

    bool save_occupancy_yaml(const nav_msgs::msg::OccupancyGrid &map,
                             const std::filesystem::path        &yaml_path,
                             const std::filesystem::path        &pgm_path) {
        std::ofstream output(yaml_path, std::ios::trunc);

        if (!output.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open YAML: %s",
                         yaml_path.string().c_str());

            return false;
        }

        output << std::fixed << std::setprecision(6);

        output << "image: " << pgm_path.filename().string() << "\n";

        output << "mode: trinary\n";

        output << "resolution: " << map.info.resolution << "\n";

        output << "origin: [" << map.info.origin.position.x << ", "
               << map.info.origin.position.y << ", 0.0]\n";

        output << "negate: 0\n";

        output << "occupied_thresh: 0.65\n";

        output << "free_thresh: 0.25\n";

        output.flush();
        return output.good();
    }

    bool replace_file(const std::filesystem::path &temporary,
                      const std::filesystem::path &target) {
        std::error_code error;

        std::filesystem::remove(target, error);

        error.clear();

        std::filesystem::rename(temporary, target, error);

        if (error) {
            RCLCPP_ERROR(this->get_logger(), "Cannot replace file %s: %s",
                         target.string().c_str(), error.message().c_str());

            return false;
        }

        return true;
    }

    bool save_occupancy_snapshot(const nav_msgs::msg::OccupancyGrid &map) {
        std::filesystem::path base_path(occupancy_save_prefix_);

        if (base_path.has_extension()) {
            base_path.replace_extension();
        }

        const std::filesystem::path directory = base_path.parent_path();

        std::error_code error;

        if (!directory.empty()) {
            std::filesystem::create_directories(directory, error);

            if (error) {
                RCLCPP_ERROR(this->get_logger(),
                             "Cannot create map directory: %s",
                             error.message().c_str());

                return false;
            }
        }

        std::filesystem::path pgm_path = base_path;

        pgm_path += ".pgm";

        std::filesystem::path yaml_path = base_path;

        yaml_path += ".yaml";

        std::filesystem::path pgm_temp = pgm_path;

        pgm_temp += ".tmp";

        std::filesystem::path yaml_temp = yaml_path;

        yaml_temp += ".tmp";

        if (!save_occupancy_pgm(map, pgm_temp)) {
            return false;
        }

        if (!save_occupancy_yaml(map, yaml_temp, pgm_path)) {
            std::filesystem::remove(pgm_temp, error);

            return false;
        }

        if (!replace_file(pgm_temp, pgm_path)) {
            return false;
        }

        if (!replace_file(yaml_temp, yaml_path)) {
            return false;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Automatically saved 2D map: "
                    "%s and %s",
                    pgm_path.string().c_str(), yaml_path.string().c_str());

        return true;
    }

    // 使用service保存地图
    bool save_2d_map() {
        std::shared_ptr<const nav_msgs::msg::OccupancyGrid> snapshot;

        {
            std::lock_guard<std::mutex> lock(latest_map_mutex_);

            snapshot = latest_map_snapshot_;
        }

        if (!snapshot) {
            RCLCPP_ERROR(this->get_logger(), "No 2D map snapshot available");

            return false;
        }

        return save_occupancy_snapshot(*snapshot);
    }
    /** add 2D map */

private:
    // add mode change
    // false: mapping  true: localization
    void set_mode_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                           std_srvs::srv::SetBool::Response::SharedPtr res) {
        const auto old_mode = system_mode.load();

        // ========================================================
        // LOCALIZATION
        // ========================================================
        if (req->data) {
            system_mode = SystemMode::LOCALIZATION;

            RCLCPP_INFO(this->get_logger(), "FAST-LIO switch to LOCALIZATION");

            res->success = true;
            res->message = "FAST-LIO switched to LOCALIZATION";

            return;
        }

        // ========================================================
        // MAPPING
        // ========================================================

        /*
         * 如果本来已经是 MAPPING，
         * 不要再次执行 reset。
         *
         * 只有真正从 LOCALIZATION -> MAPPING
         * 才认为开始一次新的建图。
         */
        if (old_mode != SystemMode::MAPPING) {
            // reset期间继续保持LOCALIZATION，
            // timer_callback不会保存关键帧
            system_mode = SystemMode::LOCALIZATION;

#if SAVE_KEYFRAME

            if (sc_keyframe_save_en) {
                RCLCPP_INFO(this->get_logger(),
                            "Reset Scan Context keyframe database...");

                if (!reset_scan_context_keyframe_storage()) {
                    RCLCPP_ERROR(this->get_logger(),
                                 "Failed to reset Scan Context keyframes");

                    res->success = false;
                    res->message = "Failed to reset Scan Context keyframes";

                    return;
                }
            }

#endif
        }

        // reset完成后才真正打开建图模式
        system_mode = SystemMode::MAPPING;

        RCLCPP_INFO(this->get_logger(), "FAST-LIO switch to MAPPING");

        res->success = true;
        res->message = "FAST-LIO switched to MAPPING";
    }

    // add points position filter
    void filter_mapping_height(PointCloudXYZI::Ptr &cloud) {
        if (!cloud || cloud->empty()) {
            return;
        }

        PointCloudXYZI filtered;
        filtered.reserve(cloud->size());

        for (const auto &point : cloud->points) {
            // lidar 坐标 -> IMU 坐标
            const V3D p_lidar(point.x, point.y, point.z);

            const V3D p_imu =
                state_point.offset_R_L_I * p_lidar + state_point.offset_T_L_I;

            // IMU 坐标 -> base_link 坐标
            const V3D p_base = IMU_R_wrt_BASE * p_imu + IMU_T_wrt_BASE;

            const double x = p_base.x();
            const double y = p_base.y();
            const double z = p_base.z();

            // 只保留需要的高度范围
            // if (z < mapping_min_height || z > mapping_max_height) {
            //     continue;
            // }

            // 过滤腰部与手臂
            const bool chassis_inside_points = x > -0.60 && x < 0.60 &&
                                               y > -0.50 && y < 0.50 &&
                                               z > 0.30 && z < 1.80;

            if (chassis_inside_points) {
                continue;
            }

            // 设置使用的激光范围
            const double range_xy =
                std::sqrt(point.x * point.x +
                          point.y * point.y);  // 暂时使用2D计算 减少计算量

            if (range_xy < mapping_min_range || range_xy > mapping_max_range) {
                continue;
            }

            filtered.push_back(point);
        }

        filtered.width = filtered.size();
        filtered.height = 1;
        filtered.is_dense = cloud->is_dense;

        *cloud = std::move(filtered);
    }

    // add base_link
    void publish_base_to_imu_static_tf() {
        geometry_msgs::msg::TransformStamped trans;
        trans.header.stamp = this->get_clock()->now();
        trans.header.frame_id = "base_link";
        trans.child_frame_id = "imu_link";
        trans.transform.translation.x = IMU_T_wrt_BASE.x();
        trans.transform.translation.y = IMU_T_wrt_BASE.y();
        trans.transform.translation.z = IMU_T_wrt_BASE.z();

        Eigen::Quaterniond q_base_imu(IMU_R_wrt_BASE);
        q_base_imu.normalize();
        trans.transform.rotation.x = q_base_imu.x();
        trans.transform.rotation.y = q_base_imu.y();
        trans.transform.rotation.z = q_base_imu.z();
        trans.transform.rotation.w = q_base_imu.w();
        static_tf_broadcaster_->sendTransform(trans);

        RCLCPP_INFO(this->get_logger(),
                    "Published static TF: base_link -> imu_link");
    }

    // add lidar_link
    void publish_base_to_lidar_static_tf() {
        // LiDAR 坐标系到 base_link 坐标系的旋转
        const M3D R_BASE_LIDAR = IMU_R_wrt_BASE * Lidar_R_wrt_IMU;

        // LiDAR 原点在 base_link 坐标系中的位置
        const V3D T_BASE_LIDAR =
            IMU_T_wrt_BASE + IMU_R_wrt_BASE * Lidar_T_wrt_IMU;

        geometry_msgs::msg::TransformStamped trans;

        trans.header.stamp = this->get_clock()->now();
        trans.header.frame_id = "base_link";
        trans.child_frame_id = "livox_frame";

        trans.transform.translation.x = T_BASE_LIDAR.x();
        trans.transform.translation.y = T_BASE_LIDAR.y();
        trans.transform.translation.z = T_BASE_LIDAR.z();

        Eigen::Quaterniond q_base_lidar(R_BASE_LIDAR);
        q_base_lidar.normalize();

        trans.transform.rotation.x = q_base_lidar.x();
        trans.transform.rotation.y = q_base_lidar.y();
        trans.transform.rotation.z = q_base_lidar.z();
        trans.transform.rotation.w = q_base_lidar.w();

        static_tf_broadcaster_->sendTransform(trans);

        RCLCPP_INFO(this->get_logger(),
                    "Published static TF: base_link -> livox_frame");
    }

    // pub livox scan
    void publish_livox_scan() {
        auto   now = this->get_clock()->now();
        double current_time = now.seconds();

        static double last_scan_pub_time_ = 0.0;
        static double last_livox_scan_stamp_ = 0.0;

        if (current_time - last_scan_pub_time_ < scan_publish_period_) {
            return;
        }

        livox_ros_driver2::msg::CustomMsg::SharedPtr msg;
        {
            std::lock_guard<std::mutex> lock(livox_scan_mutex);

            if (!latest_livox_msg_) return;

            msg = latest_livox_msg_;
        }

        // 判断是不是新数据
        double msg_time = get_time_sec(msg->header.stamp);
        if (msg_time <= last_livox_scan_stamp_) {
            return;
        }
        last_livox_scan_stamp_ = msg_time;

        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = msg->header.stamp;
        scan.header.frame_id = "imu_link";
        scan.angle_min = -M_PI;
        scan.angle_max = M_PI;
        int beam_num = 720;
        scan.angle_increment = (scan.angle_max - scan.angle_min) / beam_num;
        scan.range_min = 0.1;
        scan.range_max = 30.0;
        scan.ranges.assign(beam_num, std::numeric_limits<float>::infinity());

        for (auto &p : msg->points) {
            if (p.line != 2 && p.line != 3) continue;

            V3D p_lidar(p.x, p.y, p.z);
            V3D p_imu =
                state_point.offset_R_L_I * p_lidar + state_point.offset_T_L_I;

            float x = p_imu(0);
            float y = p_imu(1);
            float z = p_imu(2);

            // 高度过滤
            if (z < -0.3 || z > 1.5 || (x < 0.3 && x > -1.0) || (y < 0.2 && y > -0.6)) continue;
            float range = sqrt(x * x + y * y);
            if (range < scan.range_min || range > scan.range_max) continue;

            float angle = atan2(y, x);
            int   index = (angle - scan.angle_min) / scan.angle_increment;

            if (index >= 0 && index < beam_num) {
                if (range < scan.ranges[index]) {
                    scan.ranges[index] = range;
                }
            }
        }

        pub_livox_scan_->publish(scan);

        last_scan_pub_time_ = current_time;
    }

    void timer_callback() {
        ThreadWallTimer cpu_wall_timer;

        if (sync_packages(Measures)) {
            if (flg_first_scan) {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                return;
            }

            double t0, t1, t2, t3, t4, t5, match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time = 0;
            t0 = omp_get_wtime();

            const double imu_start = omp_get_wtime();
            p_imu->Process(Measures, kf, feats_undistort);
            const double imu_end = omp_get_wtime();

            state_point = kf.get_x();

#if DEBUG_LOG
            // ================= DRIFT DEBUG: IMU预测状态 =================
            const state_ikfom state_imu_pred = state_point;
            const V3D         euler_imu_pred = SO3ToEuler(state_imu_pred.rot);
            // ===========================================================
#endif

            pos_lid =
                state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL)) {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            flg_EKF_inited =
                (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? false
                                                                         : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);

            // add points position filter
            filter_mapping_height(feats_down_body);

            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if (ikdtree.Root_Node == nullptr) {
                RCLCPP_INFO(this->get_logger(), "Initialize the map kdtree");
                if (feats_down_size > 5) {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for (int i = 0; i < feats_down_size; i++) {
                        pointBodyToWorld(&(feats_down_body->points[i]),
                                         &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                return;
            }
            int featsFromMapNum = ikdtree.validnum();
            kdtree_size_st = ikdtree.size();

            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<"
            // downsamp "<<feats_down_size<<" Map num:
            // "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5) {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\n");
                return;
            }

            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);

#if SAVE_LOG
            fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time
                     << " " << euler_cur.transpose() << " "
                     << state_point.pos.transpose() << " "
                     << ext_euler.transpose() << " "
                     << state_point.offset_T_L_I.transpose() << " "
                     << state_point.vel.transpose() << " "
                     << state_point.bg.transpose() << " "
                     << state_point.ba.transpose() << " " << state_point.grav
                     << endl;
#endif

            if (0)  // If you need to see map point, change to "if(1)"
            {
                PointVector().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage,
                                NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true;  //

            t2 = omp_get_wtime();

            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;

            const double update_start = omp_get_wtime();
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV,
                                                  solve_H_time);
            const double update_end = omp_get_wtime();

            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);

#if DEBUG_LOG
            // ====================== DRIFT DEBUG ======================
            static uint64_t drift_debug_count = 0;
            ++drift_debug_count;

            // 10帧打印一次，避免日志本身影响实时性
            if (drift_debug_count % 10 == 0) {
                // 激光更新前 -> 更新后的变化
                const V3D delta_pos = state_point.pos - state_imu_pred.pos;
                const V3D delta_vel = state_point.vel - state_imu_pred.vel;

                const V3D euler_after = SO3ToEuler(state_point.rot);
                const V3D delta_euler = euler_after - euler_imu_pred;

                // base_link最终位置
                V3D                base_position_world;
                Eigen::Quaterniond base_orientation_world;
                get_base_link_pose(base_position_world, base_orientation_world);

                // LiDAR -> IMU 当前估计外参
                const V3D ext_euler_debug =
                    SO3ToEuler(state_point.offset_R_L_I);

                std::cout << std::fixed << std::setprecision(6);

                std::cout << "\n========== FASTLIO DRIFT DEBUG ==========\n"

                          // -------- IMU预测 --------
                          << "[IMU PRED] pos = "
                          << state_imu_pred.pos.transpose() << "\n"

                          << "[IMU PRED] vel = "
                          << state_imu_pred.vel.transpose() << "\n"

                          << "[IMU PRED] rpy = " << euler_imu_pred.transpose()
                          << "\n"

                          // -------- 激光更新后 --------
                          << "[LIDAR OUT] pos = " << state_point.pos.transpose()
                          << "\n"

                          << "[LIDAR OUT] vel = " << state_point.vel.transpose()
                          << "\n"

                          << "[LIDAR OUT] rpy = " << euler_after.transpose()
                          << "\n"

                          // -------- 当前激光对EKF做了多少修正 --------
                          << "[LIDAR CORR] dpos = " << delta_pos.transpose()
                          << "  norm=" << delta_pos.norm() << "\n"

                          << "[LIDAR CORR] dvel = " << delta_vel.transpose()
                          << "  norm=" << delta_vel.norm() << "\n"

                          << "[LIDAR CORR] drpy = " << delta_euler.transpose()
                          << "\n"

                          // -------- IMU bias --------
                          << "[BIAS] bg = " << state_point.bg.transpose()
                          << "\n"

                          << "[BIAS] ba = " << state_point.ba.transpose()
                          << "\n"

                          // -------- 重力 --------
                          << "[GRAV] = " << state_point.grav
                          << "\n"

                          // -------- 激光匹配质量 --------
                          << "[MATCH] residual = " << res_mean_last
                          << " effective_points = " << effct_feat_num
                          << " down_points = " << feats_down_size
                          << "\n"

                          // -------- 在线外参 --------
                          << "[EXTRINSIC] T = "
                          << state_point.offset_T_L_I.transpose() << "\n"

                          << "[EXTRINSIC] RPY = " << ext_euler_debug.transpose()
                          << "\n"

                          // -------- 实际发布的base_link --------
                          << "[BASE_LINK] pos = "
                          << base_position_world.transpose() << "\n"

                          << "==========================================\n";
            }
            // =========================================================
#endif

            pos_lid =
                state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            const auto mode = system_mode.load();

            // add 2D map
            if (mode == SystemMode::MAPPING) {
                enqueue_occupancy_frame();
            }

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped_, tf_broadcaster_);

            // add map-odom static tf
            if (mode == SystemMode::MAPPING) {
                publish_map_to_odom(tf_broadcaster_);
            }

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();

#if SAVE_KEYFRAME
            if (mode == SystemMode::MAPPING) {
                try_save_scan_context_keyframe();
            }
#endif

            double pub_path_t0, pub_path_t1;
            double pub_lidar_t0, pub_lidar_t1;

            const double publish_start = omp_get_wtime();
            /******* Publish points *******/
            if (path_en) {
                pub_path_t0 = omp_get_wtime();
                publish_path(pubPath_);
                pub_path_t1 = omp_get_wtime();
            }

            pub_lidar_t0 = omp_get_wtime();
            double now = this->get_clock()->now().seconds();
            if (scan_pub_en) {
                if (now - last_cloud_pub_time_ >= cloud_publish_period_) {
                    publish_frame_world(pubLaserCloudFull_);
                    last_cloud_pub_time_ = now;
                }
            }
            pub_lidar_t1 = omp_get_wtime();

            // if (scan_pub_en) {
            //     pub_lidar_t0 = omp_get_wtime();
            //     publish_frame_world(pubLaserCloudFull_);
            //     pub_lidar_t1 = omp_get_wtime();
            // }

            if (scan_pub_en && scan_body_pub_en)
                publish_frame_body(pubLaserCloudFull_body_);
            if (effect_pub_en) publish_effect_world(pubLaserCloudEffect_);
            // if (map_pub_en) publish_map(pubLaserCloudMap_);

            // pub livox scan
            if (livox_scan_pub_en) {
                publish_livox_scan();
            }

            const double publish_end = omp_get_wtime();

            double pub_path = (pub_path_t1 - pub_path_t0) * 1000;
            double pub_lidar = (pub_lidar_t1 - pub_lidar_t0) * 1000;

            const double imu_ms = (imu_end - imu_start) * 1000.0;

            const double update_ms = (update_end - update_start) * 1000.0;

            const double map_incremental_ms = (t5 - t3) * 1000.0;

            const double pub_ms = (publish_end - publish_start) * 1000.0;

            if (imu_ms > 50.0 || update_ms > 50.0 ||
                map_incremental_ms > 50.0 || pub_ms > 50.0) {
                std::cerr << "[FASTLIO time callback SLOW]"
                          << " imu_ms=" << imu_ms << " update_ms=" << update_ms
                          << " map_incremental_ms=" << map_incremental_ms
                          << " pub_ms=" << pub_ms << " pub_path ms=" << pub_path
                          << " pub_lidar ms=" << pub_lidar
                          << " kdtree_size=" << ikdtree.size()
                          << " down_points=" << feats_down_size << std::endl;
            }

            /*** Debug variables ***/
            if (runtime_pos_log) {
                frame_num++;
                kdtree_size_end = ikdtree.size();
                aver_time_consu =
                    aver_time_consu * (frame_num - 1) / frame_num +
                    (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num +
                                (t_update_end - t_update_start) / frame_num;
                aver_time_match =
                    aver_time_match * (frame_num - 1) / frame_num +
                    (match_time) / frame_num;
                aver_time_incre =
                    aver_time_incre * (frame_num - 1) / frame_num +
                    (kdtree_incremental_time) / frame_num;
                aver_time_solve =
                    aver_time_solve * (frame_num - 1) / frame_num +
                    (solve_time + solve_H_time) / frame_num;
                aver_time_const_H_time =
                    aver_time_const_H_time * (frame_num - 1) / frame_num +
                    solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter++;
                printf(
                    "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f "
                    "ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map "
                    "incre: %0.6f ave total: %0.6f icp: %0.6f construct H: "
                    "%0.6f \n",
                    t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3,
                    aver_time_consu, aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
#if SAVE_LOG
                fout_out << setw(20)
                         << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " "
                         << state_point.pos.transpose() << " "
                         << ext_euler.transpose() << " "
                         << state_point.offset_T_L_I.transpose() << " "
                         << state_point.vel.transpose() << " "
                         << state_point.bg.transpose() << " "
                         << state_point.ba.transpose() << " "
                         << state_point.grav << " "
                         << feats_undistort->points.size() << endl;
                         dump_lio_state_to_log(fp);
#endif
            }
        }
    }

    void map_publish_callback() {
        if (map_pub_en) publish_map(pubLaserCloudMap_);
    }

    void map_save_callback_old(
        std_srvs::srv::Trigger::Request::ConstSharedPtr req,
        std_srvs::srv::Trigger::Response::SharedPtr     res) {
        RCLCPP_INFO(this->get_logger(), "Saving map to %s...",
                    map_file_path.c_str());
        if (pcd_save_en) {
            save_to_pcd();
            res->success = true;
            res->message = "Map saved.";
        } else {
            res->success = false;
            res->message = "Map save disabled.";
        }
    }

    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr,
                           std_srvs::srv::Trigger::Response::SharedPtr res) {
        bool save3d = false;
        bool save2d = false;

        RCLCPP_INFO(this->get_logger(), "Saving 2D&3D maps to %s...",
                    map_file_path.c_str());

        // 保存3D
        if (pcd_save_en) {
            save3d = save_to_pcd();
        }

        // 保存2D
        if (occupancy_enable_) {
            save2d = save_2d_map();
        }

        res->success = save3d && save2d;

        if (res->success) {
            res->message = "3D and 2D map saved.";
        } else {
            res->message = "Map save failed.";
        }
    }

private:
    // add thread
    // timer callback 使用独立回调组
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    // 激光订阅使用独立回调组
    rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pubLaserCloudMap_;

    // pub livox scan
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_livox_scan_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr      pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
        sub_pcl_livox_;
    // rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr
    // sub_pcl_livox_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // add base_link
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    /**add 2D map */
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        occupancy_map_pub_;
    /**add 2D map */

    // add mode change
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_srv_;

    rclcpp::TimerBase::SharedPtr                       timer_;
    rclcpp::TimerBase::SharedPtr                       map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    bool   effect_pub_en = false, map_pub_en = false;
    int    effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0,
                           aver_time_match = 0, aver_time_incre = 0,
                           aver_time_solve = 0, aver_time_const_H_time = 0;
    bool   flg_EKF_converged, EKF_stop_flg = 0;
    double epsi[23] = {0.001};

    // pub livox scan
    double scan_publish_period_ = 0.1;  // 秒

    // pub point cloud
    double cloud_publish_period_ = 0.1;  // 秒
    double last_cloud_pub_time_ = 0.0;

#if SAVE_LOG
    FILE    *fp;
    ofstream fout_pre, fout_out, fout_dbg;
#endif
};

int main(int argc, char **argv) {
    // rclcpp::init(argc, argv);

    // signal(SIGINT, SigHandle);

    // rclcpp::spin(std::make_shared<LaserMappingNode>());

    // if (rclcpp::ok())
    //     rclcpp::shutdown();

    // add thread
    rclcpp::init(argc, argv);
    signal(SIGINT, SigHandle);
    auto node = std::make_shared<LaserMappingNode>();
    /*
     * 两个执行线程已经足够测试：
     *
     * 线程1：默认回调组
     *        timer_callback / IMU / map timer
     *
     * 线程2：lidar_callback_group_
     *        激光接收和点云预处理
     */
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      4);
    executor.add_node(node);
    executor.spin();
    executor.remove_node(node);
    node.reset();
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en) {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (runtime_pos_log) {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;
        FILE          *fp2;
        string         log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(), "w");
        fprintf(fp2,
                "time_stamp, total time, scan point size, incremental time, "
                "search time, delete size, delete time, tree size st, tree "
                "size end, add point size, preprocess time\n");
        for (int i = 0; i < time_log_counter; i++) {
            fprintf(fp2, "%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",
                    T1[i], s_plot[i], int(s_plot2[i]), s_plot3[i], s_plot4[i],
                    int(s_plot5[i]), s_plot6[i], int(s_plot7[i]),
                    int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
