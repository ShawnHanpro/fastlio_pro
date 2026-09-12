#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
// #include <pcl/common/transforms.h>

#include <open3d/Open3D.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "open3d_conversions/open3d_conversions.h"
#include "open3d_registration/open3d_registration.h"

// add mode change
#include <std_srvs/srv/set_bool.hpp>

// add map server
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

#define PI 3.1415926

// add mode change
enum class SystemMode { MAPPING = 0, LOCALIZATION = 1 };
std::atomic<SystemMode> system_mode(SystemMode::MAPPING);

class KalmanFilter {
public:
    KalmanFilter()
        : processVar_(0.0),
          estimatedMeasVar_(0.0),
          posteriEstimate_(0.0),
          posteriErrorEstimate_(1.0) {}

    void KalmanFilterInit(double processVar, double estimatedMeasVar,
                          double posteriEstimate = 0.0,
                          double posteriErrorEstimate = 1.0) {
        processVar_ = processVar;
        estimatedMeasVar_ = estimatedMeasVar;
        posteriEstimate_ = posteriEstimate;
        posteriErrorEstimate_ = posteriErrorEstimate;
    }
    void inputLatestNoisyMeasurement(double measurement) {
        double prioriEstimate = posteriEstimate_;
        double prioriErrorEstimate = posteriErrorEstimate_ + processVar_;

        double denominator = prioriErrorEstimate + estimatedMeasVar_;

        // 防止除零导致 NaN
        if (std::abs(denominator) < 1e-10) {
            // 如果分母接近零，直接使用测量值
            posteriEstimate_ = measurement;
            posteriErrorEstimate_ = 1.0;
            return;
        }

        double blendingFactor = prioriErrorEstimate / denominator;
        posteriEstimate_ =
            prioriEstimate + blendingFactor * (measurement - prioriEstimate);
        posteriErrorEstimate_ = (1 - blendingFactor) * prioriErrorEstimate;
    }

    double getLatestEstimatedMeasurement() { return posteriEstimate_; }

private:
    double processVar_;
    double estimatedMeasVar_;
    double posteriEstimate_;
    double posteriErrorEstimate_;
};

class GloabalLocalization : public rclcpp::Node {
private:
    /* data */
public:
    GloabalLocalization();
    ~GloabalLocalization();

    /// @brief 初始化定位
    bool LocalizationInitialize();

    /// @brief 订阅fast_lio里程计信息
    void CallbackBaselink2Odom(
        const nav_msgs::msg::Odometry::SharedPtr baselink2odom);
    /// @brief 订阅在baselink下的点云
    void CallbackScan(
        const sensor_msgs::msg::PointCloud2::SharedPtr scan_in_baselink);

    /// @brief 订阅在初始位姿
    void CallbackInitialPose(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
            initialpose);

    void StartLoc();

    void Localization();

    /// @brief 欧拉角转mat3x3
    /// @param euler
    /// @return
    Eigen::Matrix3d Euler2Matrix3d(const Eigen::Vector3d euler);

    /// @brief 获取tf关系到矩阵
    /// @param frame_id
    /// @param child_frame_id
    /// @param matrix
    /// @return
    bool GetTfTransformToMatrix(std::string      frame_id,
                                std::string      child_frame_id,
                                Eigen::Matrix4d &matrix);

    /// @brief compute 3d distance between two points
    /// @param a
    /// @param b
    /// @return 距离值
    double ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

    // add mode change
    void set_mode_callback(const std_srvs::srv::SetBool::Request::SharedPtr req,
                           std_srvs::srv::SetBool::Response::SharedPtr res);
    bool LoadMap();

    // add map server
    bool    ChangeMapServerState(uint8_t transition_id);
    bool    StartMapServer();
    bool    StopMapServer();
    uint8_t GetMapServerState();

private:
    /// @brief 订阅baselink2odom,即fast_lio的里程计信息
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_baselink2odom_;

    /// @brief 订阅当前帧点云
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
        sub_scan_cur_;

    /// @brief 订阅初始位姿
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::
        SharedPtr sub_initialpose_;

    /// @brief baselink到odom的pose表达
    nav_msgs::msg::Odometry pose_baselink2odom_;

    /// @brief bselink到odom的变换矩阵表达
    Eigen::Matrix4d mat_baselink2odom_;
    /// @brief odom到map的矩阵
    Eigen::Matrix4d mat_odom2map_;
    Eigen::Matrix4d mat_odom2map_kalman_;
    /// @brief baselink到map = mat_odom2map * mat_baselink2odom
    Eigen::Matrix4d mat_baselink2map_;
    /// @brief initialpose初始位姿
    Eigen::Matrix4d mat_initialpose_;

    std::mutex lock_mat_odom2map_;

    /// @brief baselink和运动中心
    Eigen::Matrix4d mat_baselink2motionlink_;

    /// @brief imulink到baselink
    Eigen::Matrix4d mat_imulink2baselink_;

    /// @brief 初始位姿, x, y, z, roll, pitch, yaw (单位:度degrees)
    std::vector<double> initialpose_;

    /// @brief 原始地图点云
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_ori_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_coarse_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_fine_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_cur_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan_cur_;

    std::queue<open3d::geometry::PointCloud> que_pcd_scan_;
    int                                      queue_maxsize_;
    double                                   voxelsize_coarse_;
    double                                   voxelsize_fine_;

    /// @brief 定位配准fitness(overlap)阈值
    double threshold_fitness_;
    /// @brief 配准fitness(overlap)阈值
    double threshold_fitness_init_;

    std::thread thread_loc_;
    std::mutex  lock_scan_;
    std::mutex  lock_exit_;
    bool        flag_exit_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_baselink2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        pub_baselink2map_kalman_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_motionlink2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom2map_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom2map_kalman_;
    rclcpp::Time                                          timestamp_odom_;
    std::mutex                                            lock_timestamp_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan2map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_submap_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        pub_localization_3d_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        pub_localization_3d_confidence_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
        pub_localization_3d_delay_ms_;

    geometry_msgs::msg::PoseStamped localization_3d_;
    std_msgs::msg::Float32          localization_3d_confidence_;
    std_msgs::msg::Float32          localization_3d_delay_ms_;

    std::shared_ptr<tf2_ros::TransformBroadcaster>       br_odom2map_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    bool save_scan_;

    /// @brief 定位频率(定位间隔时间，多少秒1次)
    double loc_frequence_;

    /// @brief source点云最大点数量
    int maxpoints_source_ = 50000;
    /// @brief target点云最大点数量
    int maxpoints_target_ = 200000;

    /// @brief 初始化成功标志
    // bool loc_initialized_ = false;
    // add mode change
    std::atomic<bool> loc_initialized_{false};

    /// @brief 当前定位overlap，confidence
    double loc_fitness_;

    /// @brief 定位置信度阈值
    double confidence_loc_th_;

    /// 卡尔曼滤波器
    KalmanFilter kf_baselink_x_;
    KalmanFilter kf_baselink_y_;
    KalmanFilter kf_baselink_z_;
    KalmanFilter kalman_filter_odom2map_;

    // 0:kf_processVar 1:kf_estimatedMeasVar
    std::vector<double> kf_param_x_;
    std::vector<double> kf_param_y_;
    std::vector<double> kf_param_z_;

    /// @brief 对odom2map进行kalman滤波
    bool   filter_odom2map_ = false;
    double kalman_processVar2_ = 0.0;
    double kalman_estimatedMeasVar2_ = 0.0;

    /// 1202
    /// @brief 上次更新定位时的定位值
    Eigen::Vector3d last_loc_;
    // Eigen::Vector3d cur_loc_;
    /// @brief 更新地图子图的距离,超过则更新地图子图
    double dis_updatemap_;

    tf2_ros::Buffer                             tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // add mode change
    // std::atomic<bool> enable_localization(false);
    std::atomic<bool>                                  enable_localization;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mode_srv_;
    std::string                                        path_map_;
    std::atomic<uint64_t>                              scan_seq_{0};
    rclcpp::TimerBase::SharedPtr                       startup_mode_timer_;
    // ============================================================
    // 模式切换保护
    //
    // mode_switch_mutex_:
    //   防止多个 MAPPING / LOCALIZATION 请求同时修改状态机
    //
    // localization_work_mutex_:
    //   防止模式切换时 LoadMap() 与 Open3D ICP 同时访问定位地图
    // ============================================================
    std::mutex mode_switch_mutex_;
    std::mutex localization_work_mutex_;

    // add map server
    rclcpp::CallbackGroup::SharedPtr map_server_client_group_;
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr
        map_server_change_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
        map_server_get_state_client_;
};

GloabalLocalization::GloabalLocalization()
    : Node("global_loc_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)) {
    // add mode change
    enable_localization.store(false);
    mode_srv_ = create_service<std_srvs::srv::SetBool>(
        "/open3d/set_mode",
        std::bind(&GloabalLocalization::set_mode_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // add map server
    map_server_client_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    map_server_change_state_client_ =
        this->create_client<lifecycle_msgs::srv::ChangeState>(
            "/map_server/change_state", rmw_qos_profile_services_default,
            map_server_client_group_);
    map_server_get_state_client_ =
        this->create_client<lifecycle_msgs::srv::GetState>(
            "/map_server/get_state", rmw_qos_profile_services_default,
            map_server_client_group_);

    flag_exit_ = false;
    loc_initialized_ = false;
    mat_baselink2odom_ = Eigen::Matrix4d::Identity();
    mat_odom2map_ = Eigen::Matrix4d::Identity();
    mat_initialpose_ = Eigen::Matrix4d::Identity();
    last_loc_ = Eigen::Vector3d(0, 0, -5000);

    pcd_map_ori_.reset(new open3d::geometry::PointCloud);
    pcd_map_coarse_.reset(new open3d::geometry::PointCloud);
    pcd_map_cur_.reset(new open3d::geometry::PointCloud);
    pcd_scan_cur_.reset(new open3d::geometry::PointCloud);
    pcd_map_fine_.reset(new open3d::geometry::PointCloud);
    queue_maxsize_ = 5;

    pub_baselink2map_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/baselink2map", 10);
    pub_baselink2map_kalman_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/baselink2map_kalman", 10);
    pub_motionlink2map_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/motionlink2map", 10);
    pub_odom2map_ =
        this->create_publisher<nav_msgs::msg::Odometry>("/odom2map", 10);
    pub_odom2map_kalman_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom2map_kalman", 10);

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));

    map_qos.reliable();
    map_qos.transient_local();

    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_3d",
                                                                     map_qos);
    // pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map",
    // 1);
    pub_submap_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/submap", 1);
    pub_scan2map_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan2map", 1);
    pub_scan_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan", 1);
    pub_localization_3d_ =
        this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/localization_3d", 1);
    pub_localization_3d_confidence_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/localization_3d_confidence", 1);
    pub_localization_3d_delay_ms_ =
        this->create_publisher<std_msgs::msg::Float32>(
            "/localization_3d_delay_ms", 1);

    loc_frequence_ = 2.0;  //
    loc_fitness_ = 0.0;
    // 注册回调函数
    sub_baselink2odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry_loc", 50,
        std::bind(&GloabalLocalization::CallbackBaselink2Odom, this,
                  std::placeholders::_1));

    // sub_scan_cur_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    //     "/cloud_registered_1", 50,
    //     std::bind(&GloabalLocalization::CallbackScan, this,
    //     std::placeholders::_1));

    // 使用best effort
    rclcpp::QoS scan_qos(rclcpp::KeepLast(1));
    scan_qos.best_effort();
    scan_qos.durability_volatile();

    sub_scan_cur_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_1", scan_qos,
        std::bind(&GloabalLocalization::CallbackScan, this,
                  std::placeholders::_1));

    sub_initialpose_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 50,
        std::bind(&GloabalLocalization::CallbackInitialPose, this,
                  std::placeholders::_1));

    pose_baselink2odom_ = nav_msgs::msg::Odometry();
    pose_baselink2odom_.header.frame_id = "odom";
    pose_baselink2odom_.child_frame_id = "base_link";
    // geometry_msgs的Quaternion会被初始化为0,0,0,0,而不是正确的0,0,0,1
    pose_baselink2odom_.pose.pose.orientation.w = 1;
    RCLCPP_INFO(this->get_logger(),
                "pose baselink2odom:\nx: %f, y: %f, z: %f, qx: %f, \
                            qy: %f, qz: %f, qw: %f",
                pose_baselink2odom_.pose.pose.position.x,
                pose_baselink2odom_.pose.pose.position.y,
                pose_baselink2odom_.pose.pose.position.z,
                pose_baselink2odom_.pose.pose.orientation.x,
                pose_baselink2odom_.pose.pose.orientation.y,
                pose_baselink2odom_.pose.pose.orientation.z,
                pose_baselink2odom_.pose.pose.orientation.w);

    // 队列最大数量
    this->declare_parameter<int>("pcd_queue_maxsize", 5);
    this->declare_parameter<bool>("save_scan", false);
    /// 最大点数量限制
    this->declare_parameter<int>("maxpoints_source", 50000);
    this->declare_parameter<int>("maxpoints_target", 200000);

    // 定位间隔时间
    this->declare_parameter<double>("loc_frequence", 2.0);

    /// 定位阈值
    this->declare_parameter<double>("confidence_loc_th", 0.6);

    /// 卡尔曼参数
    this->declare_parameter<std::vector<double>>("kf_baselink2map/x",
                                                 std::vector<double>(2));
    this->declare_parameter<std::vector<double>>("kf_baselink2map/y",
                                                 std::vector<double>(2));
    this->declare_parameter<std::vector<double>>("kf_baselink2map/z",
                                                 std::vector<double>(2));

    this->declare_parameter<bool>("filter_odom2map", false);
    this->declare_parameter<double>("kalman_processVar2", 0.02);
    this->declare_parameter<double>("kalman_estimatedMeasVar2", 0.04);
    // voxelsize
    this->declare_parameter<double>("voxelsize_coarse", 0.1);
    this->declare_parameter<double>("voxelsize_fine", 0.05);
    this->declare_parameter<double>("threshold_fitness_init", 0.9);
    this->declare_parameter<double>("threshold_fitness", 0.9);
    this->declare_parameter<std::vector<double>>("initialpose",
                                                 std::vector<double>());
    this->declare_parameter<double>("dis_updatemap", 1);

    this->get_parameter("pcd_queue_maxsize", queue_maxsize_);
    this->get_parameter("save_scan", save_scan_);
    this->get_parameter("maxpoints_source", maxpoints_source_);
    this->get_parameter("maxpoints_target", maxpoints_target_);
    this->get_parameter("loc_frequence", loc_frequence_);
    this->get_parameter("confidence_loc_th", confidence_loc_th_);
    this->get_parameter("kf_baselink2map/x", kf_param_x_);
    this->get_parameter("kf_baselink2map/y", kf_param_y_);
    this->get_parameter("kf_baselink2map/z", kf_param_z_);
    this->get_parameter("filter_odom2map", filter_odom2map_);
    this->get_parameter("kalman_processVar2", kalman_processVar2_);
    this->get_parameter("kalman_estimatedMeasVar2", kalman_estimatedMeasVar2_);

    RCLCPP_INFO(this->get_logger(), "Kalman filter parameters:");
    RCLCPP_INFO(this->get_logger(), "  kf_x: [%.6f, %.6f], size: %zu",
                kf_param_x_.size() >= 1 ? kf_param_x_[0] : 0.0,
                kf_param_x_.size() >= 2 ? kf_param_x_[1] : 0.0,
                kf_param_x_.size());
    RCLCPP_INFO(this->get_logger(), "  kf_y: [%.6f, %.6f], size: %zu",
                kf_param_y_.size() >= 1 ? kf_param_y_[0] : 0.0,
                kf_param_y_.size() >= 2 ? kf_param_y_[1] : 0.0,
                kf_param_y_.size());
    RCLCPP_INFO(this->get_logger(), "  kf_z: [%.6f, %.6f], size: %zu",
                kf_param_z_.size() >= 1 ? kf_param_z_[0] : 0.0,
                kf_param_z_.size() >= 2 ? kf_param_z_[1] : 0.0,
                kf_param_z_.size());
    RCLCPP_INFO(this->get_logger(), "  filter_odom2map: %s",
                filter_odom2map_ ? "true" : "false");
    this->get_parameter("voxelsize_coarse", voxelsize_coarse_);
    this->get_parameter("voxelsize_fine", voxelsize_fine_);
    this->get_parameter("threshold_fitness_init", threshold_fitness_init_);
    this->get_parameter("threshold_fitness", threshold_fitness_);
    this->get_parameter("initialpose", initialpose_);
    this->get_parameter("dis_updatemap", dis_updatemap_);

    for (auto i : initialpose_) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    mat_initialpose_.block<3, 3>(0, 0) = Euler2Matrix3d(
        Eigen::Vector3d(initialpose_[3], initialpose_[4], initialpose_[5]));
    mat_initialpose_.block<3, 1>(0, 3) =
        Eigen::Vector3d(initialpose_[0], initialpose_[1], initialpose_[2]);

    // 读取地图

    this->declare_parameter<std::string>("path_map", "");
    this->get_parameter("path_map", path_map_);

    /**
        // std::string path_map = "";
        this->declare_parameter<std::string>("path_map", "");
        this->get_parameter("path_map", path_map_);
        open3d::io::ReadPointCloud(path_map_, *pcd_map_ori_);
        if (pcd_map_ori_ == nullptr || pcd_map_ori_->IsEmpty())
        {
            RCLCPP_ERROR(this->get_logger(), "read map from path: %s failed",
       path_map_.c_str()); rclcpp::shutdown();
        }

        if (!pcd_map_ori_->HasColors())
        {
            pcd_map_ori_->PaintUniformColor({1, 0, 0});
        }
        // pcd_map_ori_->PaintUniformColor({1, 0, 0});

        pcd_map_coarse_ = pcd_map_ori_->VoxelDownSample(voxelsize_coarse_);
        pcd_map_coarse_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_coarse_
       * 2, 30));

        /// publish map, 用粗地图可视化，减少资源占用
        sensor_msgs::msg::PointCloud2 pc2_map;
        open3d_conversions::open3dToRos(*pcd_map_coarse_, pc2_map);
        pc2_map.header.frame_id = "map";
        pc2_map.header.stamp = this->now();
        pub_map_->publish(pc2_map);

        pcd_map_fine_ = pcd_map_ori_->VoxelDownSample(voxelsize_fine_);
        pcd_map_fine_->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_fine_
       * 2, 30));
    */

    GetTfTransformToMatrix("base_link", "imu_link", mat_imulink2baselink_);
    std::cout << "mat_imulink2baselink_:\n"
              << mat_imulink2baselink_ << std::endl;

    GetTfTransformToMatrix("motion_link", "base_link",
                           mat_baselink2motionlink_);
    std::cout << "mat_baselink2motionlink_:\n"
              << mat_baselink2motionlink_ << std::endl;

    RCLCPP_WARN(this->get_logger(), "initialize finished");

    br_odom2map_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    StartLoc();

    // add mode change
    // 启动后自动进入 LOCALIZATION
    startup_mode_timer_ =
        this->create_wall_timer(std::chrono::seconds(1), [this]() {
            auto req = std::make_shared<std_srvs::srv::SetBool::Request>();

            auto res = std::make_shared<std_srvs::srv::SetBool::Response>();

            // true = LOCALIZATION
            req->data = true;

            // 尝试进入 LOCALIZATION
            set_mode_callback(req, res);

            if (!res->success) {
                // 本次失败，不取消 timer
                // 1 秒后会再次尝试
                RCLCPP_WARN(this->get_logger(),
                            "Startup LOCALIZATION failed, retry after 1s: %s",
                            res->message.c_str());

                return;
            }

            // 成功以后才停止 timer
            startup_mode_timer_->cancel();

            RCLCPP_INFO(this->get_logger(), "Startup mode: LOCALIZATION");
        });
}

GloabalLocalization::~GloabalLocalization() {
    lock_exit_.lock();
    flag_exit_ = true;
    lock_exit_.unlock();
}

Eigen::Matrix3d GloabalLocalization::Euler2Matrix3d(
    const Eigen::Vector3d euler) {
    Eigen::Matrix3d mat3d;
    // convert degrees to radians
    auto              eulerAngle = euler / 180 * M_PI;
    Eigen::AngleAxisd rollAngle(
        Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(
        Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(
        Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitZ()));
    mat3d = rollAngle * pitchAngle * yawAngle;
    return mat3d;
}
bool GloabalLocalization::GetTfTransformToMatrix(std::string frame_id,
                                                 std::string child_frame_id,
                                                 Eigen::Matrix4d &matrix) {
    // 获取pose
    geometry_msgs::msg::TransformStamped pose_;
    try {
        pose_ = tf_buffer_.lookupTransform(frame_id, child_frame_id,
                                           rclcpp::Time(0));
    } catch (tf2::TransformException &e) {
        RCLCPP_ERROR(this->get_logger(), "[GetTransformMatrix]: %s", e.what());
        return false;
    }

    Eigen::Vector3d translation = Eigen::Vector3d(
        pose_.transform.translation.x, pose_.transform.translation.y,
        pose_.transform.translation.z);
    Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();

    quat = Eigen::Quaterniond(
        pose_.transform.rotation.w, pose_.transform.rotation.x,
        pose_.transform.rotation.y, pose_.transform.rotation.z);
    Eigen::Matrix3d rotation = quat.matrix();

    matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = rotation;
    matrix.matrix().block<3, 1>(0, 3) = translation;
    return true;
}

// void GloabalLocalization::CallbackBaselink2Odom(const
// nav_msgs::msg::Odometry::SharedPtr baselink2odom)
// {
//     auto odom_cbk_s = std::chrono::high_resolution_clock::now();
//     lock_timestamp_.lock();
//     timestamp_odom_ = baselink2odom->header.stamp;
//     lock_timestamp_.unlock();
//     Eigen::Isometry3d mat_current = Eigen::Isometry3d::Identity();
//     tf2::fromMsg(baselink2odom->pose.pose, mat_current);
//     auto mat_imulink2odom = mat_current.matrix();

//     mat_baselink2odom_ = mat_imulink2odom * mat_imulink2baselink_.inverse();

//     Eigen::Isometry3d Isometry3d_baselink2map;
//     mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_;
//     Isometry3d_baselink2map.matrix() = mat_baselink2map_;
//     nav_msgs::msg::Odometry baselink2map;
//     baselink2map.pose.pose = tf2::toMsg(Isometry3d_baselink2map);
//     baselink2map.header.frame_id = "map";
//     baselink2map.child_frame_id = "base_link";
//     baselink2map.header.stamp = baselink2odom->header.stamp;
//     pub_baselink2map_->publish(baselink2map);

//     Eigen::Isometry3d Isometry3d_odom2map;
//     Isometry3d_odom2map.matrix() = mat_odom2map_;
//     nav_msgs::msg::Odometry odom2map;
//     odom2map.pose.pose = tf2::toMsg(Isometry3d_odom2map);
//     odom2map.header.frame_id = "map";
//     odom2map.child_frame_id = "odom";
//     odom2map.header.stamp = baselink2odom->header.stamp;
//     pub_odom2map_->publish(odom2map);

//     /// 发布tf关系
//     geometry_msgs::msg::TransformStamped transform_odom2map;
//     transform_odom2map.header.frame_id = "map";
//     transform_odom2map.child_frame_id = "odom";
//     transform_odom2map.header.stamp = baselink2odom->header.stamp;
//     transform_odom2map.transform.translation.x =
//     odom2map.pose.pose.position.x; transform_odom2map.transform.translation.y
//     = odom2map.pose.pose.position.y;
//     transform_odom2map.transform.translation.z =
//     odom2map.pose.pose.position.z; transform_odom2map.transform.rotation =
//     odom2map.pose.pose.orientation;
//     br_odom2map_->sendTransform(transform_odom2map);

void GloabalLocalization::CallbackBaselink2Odom(
    const nav_msgs::msg::Odometry::SharedPtr baselink2odom) {
    if (!baselink2odom) {
        return;
    }

    lock_timestamp_.lock();
    timestamp_odom_ = baselink2odom->header.stamp;
    lock_timestamp_.unlock();

    Eigen::Isometry3d mat_current = Eigen::Isometry3d::Identity();

    tf2::fromMsg(baselink2odom->pose.pose, mat_current);

    // ============================================================
    // FAST-LIO 当前已经直接发布：
    //
    // odom -> base_link
    //
    // 所以这里不需要再经过 imu_link 外参转换
    // ============================================================
    mat_baselink2odom_ = mat_current.matrix();

    // add mode change
    if (!enable_localization.load()) {
        return;
    }
    Eigen::Matrix4d mat_odom2map_snapshot;
    {
        std::lock_guard<std::mutex> lock(lock_mat_odom2map_);
        mat_odom2map_snapshot = mat_odom2map_;
    }

    Eigen::Isometry3d Isometry3d_baselink2map;

    mat_baselink2map_ = mat_odom2map_snapshot * mat_baselink2odom_;

    // add mode change
    if (!loc_initialized_.load()) {
        return;
    }

    Isometry3d_baselink2map.matrix() = mat_baselink2map_;

    nav_msgs::msg::Odometry baselink2map;

    baselink2map.pose.pose = tf2::toMsg(Isometry3d_baselink2map);

    baselink2map.header.frame_id = "map";
    baselink2map.child_frame_id = "base_link";
    baselink2map.header.stamp = baselink2odom->header.stamp;

    pub_baselink2map_->publish(baselink2map);

    // ============================================================
    // map -> odom
    // ============================================================

    Eigen::Isometry3d Isometry3d_odom2map;
    Isometry3d_odom2map.matrix() = mat_odom2map_snapshot;

    nav_msgs::msg::Odometry odom2map;
    odom2map.pose.pose = tf2::toMsg(Isometry3d_odom2map);

    odom2map.header.frame_id = "map";
    odom2map.child_frame_id = "odom";
    odom2map.header.stamp = baselink2odom->header.stamp;

    pub_odom2map_->publish(odom2map);

    geometry_msgs::msg::TransformStamped transform;

    transform.header.frame_id = "map";
    transform.child_frame_id = "odom";
    // transform.header.stamp = baselink2odom->header.stamp;
    transform.header.stamp = baselink2odom->header.stamp;

    transform.transform.translation.x = odom2map.pose.pose.position.x;
    transform.transform.translation.y = odom2map.pose.pose.position.y;
    transform.transform.translation.z = odom2map.pose.pose.position.z;

    transform.transform.rotation = odom2map.pose.pose.orientation;

    br_odom2map_->sendTransform(transform);

    /// 卡尔曼滤波 - 只在定位初始化完成后执行
    if (loc_initialized_) {
        Eigen::Matrix4d mat_baselink2map_kalman = Eigen::Matrix4d::Identity();

        if (filter_odom2map_) {
            Eigen::Isometry3d Isometry3d_odom2map_kalman;
            Isometry3d_odom2map_kalman.matrix() = mat_odom2map_kalman_;
            nav_msgs::msg::Odometry odom2map_kalman;
            odom2map_kalman.pose.pose = tf2::toMsg(Isometry3d_odom2map_kalman);
            odom2map_kalman.header.frame_id = "map";
            odom2map_kalman.child_frame_id = "odom_kalman";
            odom2map_kalman.header.stamp = baselink2odom->header.stamp;
            pub_odom2map_kalman_->publish(odom2map_kalman);

            kf_baselink_z_.inputLatestNoisyMeasurement(
                (mat_odom2map_kalman_ * mat_baselink2odom_)(2, 3));
            mat_baselink2map_kalman = mat_odom2map_kalman_ * mat_baselink2odom_;
        } else {
            double input_x = mat_baselink2map_(0, 3);
            double input_y = mat_baselink2map_(1, 3);
            double input_z = mat_baselink2map_(2, 3);

            kf_baselink_x_.inputLatestNoisyMeasurement(input_x);
            kf_baselink_y_.inputLatestNoisyMeasurement(input_y);
            kf_baselink_z_.inputLatestNoisyMeasurement(input_z);
            mat_baselink2map_kalman = mat_baselink2map_;

            RCLCPP_DEBUG(this->get_logger(), "KF input: x=%.3f, y=%.3f, z=%.3f",
                         input_x, input_y, input_z);
        }

        double filtered_z = kf_baselink_z_.getLatestEstimatedMeasurement();

        // 验证结果是否有效（检查 NaN）
        if (std::isnan(filtered_z)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Kalman filter returned NaN (input was: "
                                 "%.3f), using unfiltered value",
                                 mat_baselink2map_kalman(2, 3));
            mat_baselink2map_kalman(2, 3) = mat_baselink2map_(2, 3);
        } else {
            mat_baselink2map_kalman(2, 3) = filtered_z;
        }
        Eigen::Isometry3d Isometry3d_baselink2map_kalman;
        Isometry3d_baselink2map_kalman.matrix() = mat_baselink2map_kalman;
        nav_msgs::msg::Odometry baselink2map_kalman;
        baselink2map_kalman.pose.pose =
            tf2::toMsg(Isometry3d_baselink2map_kalman);
        baselink2map_kalman.header.frame_id = "map";
        // baselink2map_kalman.child_frame_id = "base_link_kalman";
        baselink2map_kalman.header.stamp = baselink2odom->header.stamp;
        pub_baselink2map_kalman_->publish(baselink2map_kalman);

        // Eigen::Matrix4d mat_motionlink2map = mat_baselink2map_kalman *
        // mat_baselink2motionlink_.inverse(); Eigen::Isometry3d
        // Isometry3d_motionlink2map; Isometry3d_motionlink2map.matrix() =
        // mat_motionlink2map; nav_msgs::msg::Odometry motionlink2map;
        // motionlink2map.pose.pose = tf2::toMsg(Isometry3d_motionlink2map);
        // motionlink2map.header.frame_id = "map";
        // // baselink2map_kalman.child_frame_id = "base_link_kalman";
        // motionlink2map.header.stamp = baselink2odom->header.stamp;
        // pub_motionlink2map_->publish(motionlink2map);

        // /// 发布tf关系
        // geometry_msgs::msg::TransformStamped transform;
        // transform.header.frame_id = "map";
        // transform.child_frame_id = "motion_link";
        // transform.header.stamp = baselink2odom->header.stamp;
        // transform.transform.translation.x =
        // motionlink2map.pose.pose.position.x;
        // transform.transform.translation.y =
        // motionlink2map.pose.pose.position.y;
        // transform.transform.translation.z =
        // motionlink2map.pose.pose.position.z; transform.transform.rotation =
        // motionlink2map.pose.pose.orientation;
        // br_odom2map_->sendTransform(transform);

        // ============================================================
        // motion_link 当前与 base_link 重合
        //
        // TF 树中已经存在：
        // base_link -> motion_link
        //
        // 因此这里不能再发布 map -> motion_link TF。
        // 如果其他程序需要 /motionlink2map Odometry，
        // 可以直接使用 base_link 在 map 下的位姿。
        // ============================================================

        Eigen::Matrix4d mat_motionlink2map = mat_baselink2map_kalman;

        Eigen::Isometry3d Isometry3d_motionlink2map =
            Eigen::Isometry3d::Identity();

        Isometry3d_motionlink2map.matrix() = mat_motionlink2map;

        nav_msgs::msg::Odometry motionlink2map;

        motionlink2map.pose.pose = tf2::toMsg(Isometry3d_motionlink2map);

        motionlink2map.header.frame_id = "map";
        motionlink2map.child_frame_id = "motion_link";
        motionlink2map.header.stamp = baselink2odom->header.stamp;

        pub_motionlink2map_->publish(motionlink2map);

        // 注意：这里不要再发布 map -> motion_link TF
        //
        // 正确 TF 链由 TF2 自动组合：
        //
        // map -> odom -> base_link -> motion_link

        localization_3d_confidence_.data = loc_fitness_;
        pub_localization_3d_confidence_->publish(localization_3d_confidence_);
        localization_3d_delay_ms_.data =
            (this->now() - baselink2odom->header.stamp).seconds() * 1000.0;
        pub_localization_3d_delay_ms_->publish(localization_3d_delay_ms_);
        localization_3d_.header.frame_id = "map";
        localization_3d_.header.stamp = baselink2odom->header.stamp;
        localization_3d_.pose = motionlink2map.pose.pose;
        pub_localization_3d_->publish(localization_3d_);
    }
}
void GloabalLocalization::CallbackScan(
    const sensor_msgs::msg::PointCloud2::SharedPtr scan_in_baselink) {
    // add mode change
    if (!enable_localization.load()) {
        return;
    }

    auto cbk_s = std::chrono::high_resolution_clock::now();
    open3d::geometry::PointCloud pcd_recieved;
    // 单帧转换为open3d，几百us
    sensor_msgs::msg::PointCloud2::ConstSharedPtr const_scan_ptr =
        scan_in_baselink;
    open3d_conversions::rosToOpen3d(const_scan_ptr, pcd_recieved);
    // 入队列
    // pcd_recieved
    if (que_pcd_scan_.size() >= static_cast<size_t>(queue_maxsize_)) {
        std::queue<open3d::geometry::PointCloud> que_temp;
        lock_scan_.lock();
        pcd_scan_cur_->Clear();
        while (!que_pcd_scan_.empty()) {
            *pcd_scan_cur_ += que_pcd_scan_.front();
            que_temp.push(que_pcd_scan_.front());
            que_pcd_scan_.pop();
        }
        lock_scan_.unlock();
        while (!que_temp.empty()) {
            que_pcd_scan_.push(que_temp.front());
            que_temp.pop();
        }
        // 丢弃一个最旧的数据
        que_pcd_scan_.pop();
    }
    // 放入最新数据
    que_pcd_scan_.push(pcd_recieved);

    // add mode change
    scan_seq_.fetch_add(1, std::memory_order_relaxed);

    auto cbk_e = std::chrono::high_resolution_clock::now();
}

// add mode change
uint64_t last_scan_seq = 0;
bool     GloabalLocalization::LocalizationInitialize() {

    // 获取锁期间可能已经切换到了MAPPING
    if (!enable_localization.load()) {
        return false;
    }

        /// 裁剪后的地图
    std::shared_ptr<open3d::geometry::PointCloud> map_coarse_crop(
            new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> map_fine_crop(
            new open3d::geometry::PointCloud);

    /// 当前环境感知子图点云
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan(
            new open3d::geometry::PointCloud);
    /// 环境感知子图转换到地图坐标系
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan2map(
            new open3d::geometry::PointCloud);

    /// 用于配准的source target
    std::shared_ptr<open3d::geometry::PointCloud> source(
            new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target(
            new open3d::geometry::PointCloud);

    /// cropbox,用于裁剪地图和当前环境感知子图
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_map(
            new open3d::geometry::OrientedBoundingBox);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_scan(
            new open3d::geometry::OrientedBoundingBox);

    /// 当前baselink到odom(camera_init)和map坐标系的关系
    Eigen::Matrix4d mat_baselink2odom_cur = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_baselink2map_cur = Eigen::Matrix4d::Identity();

    /// 固定感知子图/历史地图子图大小
    OBB_map->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_map->color_ = Eigen::Vector3d(1, 0.5, 0);
    OBB_scan->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_scan->color_ = Eigen::Vector3d(0, 1, 0);

    double fitness_initial;  /// overlap
    double loc_cost = 0;     /// 定位耗时(ms)
    int    count_success = 0;
    while (rclcpp::ok()) {
            // add mode change
        // =====================================================
        // 模式已经切回Mapping，立即取消定位初始化
        // =====================================================
        if (!enable_localization.load()) {
                RCLCPP_INFO(this->get_logger(),
                            "Localization initialization cancelled");

                return false;
        }
        const uint64_t current_seq = scan_seq_.load(std::memory_order_relaxed);
        if (current_seq == last_scan_seq) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
        }
        last_scan_seq = current_seq;

        auto loc_s =
            std::chrono::high_resolution_clock::now();  /// 开始定位计时
        lock_scan_.lock();
        if (pcd_scan_cur_->IsEmpty()) {
                lock_scan_.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
        } else {
                /// 获取最新关系
            mat_baselink2odom_cur = mat_baselink2odom_;
            mat_baselink2map_cur = mat_baselink2map_;
            *pcd_scan = *pcd_scan_cur_;
            lock_scan_.unlock();
            lock_mat_odom2map_.lock();

            /// 将cropbox转换到对应位置进行裁剪点云
            OBB_map->center_ = mat_baselink2map_cur.block<3, 1>(0, 3);
            OBB_map->R_ = mat_baselink2map_cur.block<3, 3>(0, 0);
            OBB_scan->center_ = mat_baselink2odom_cur.block<3, 1>(0, 3);
            OBB_scan->R_ = mat_baselink2odom_cur.block<3, 3>(0, 0);
            *map_fine_crop = *pcd_map_fine_->Crop(*OBB_map);

            /// 配准计时
            auto reg0_s = std::chrono::high_resolution_clock::now();

            Eigen::Matrix4d reg_matrix = Eigen::Matrix4d::Identity();
            reg_matrix = mat_odom2map_;

            *target = *map_fine_crop;
            open3d::utility::LogInfo(
                    "before sample, target size: {}, has normal: {}",
                    target->points_.size(),
                target->HasNormals() ? "true" : "false");
            if (target->points_.size() >
                static_cast<size_t>(maxpoints_target_)) {
                    target = target->RandomDownSample(double(maxpoints_target_) /
                                                      target->points_.size());
            }
            open3d::utility::LogInfo(
                    "after sample, target size: {}, has normal: {}",
                    target->points_.size(),
                target->HasNormals() ? "true" : "false");

            source = pcd_scan->Crop(*OBB_scan);
            open3d::utility::LogInfo("source size: {}, has normal: {}",
                                         source->points_.size(),
                                     source->HasNormals() ? "true" : "false");
            if (source->points_.size() >
                static_cast<size_t>(maxpoints_source_)) {
                    source = source->RandomDownSample(double(maxpoints_source_) /
                                                      source->points_.size());
            }
            open3d::utility::LogInfo("source size: {}, has normal: {}",
                                         source->points_.size(),
                                     source->HasNormals() ? "true" : "false");

            source->Transform(reg_matrix);
            *pcd_scan2map = *source;

            // auto multiScale_reg_matrix =
            // pcd_tools::RegistrationMultiScaleIcp(source, target,
            // voxelsize_fine_, 1, {1, 2, 4});
            auto multiScale_reg_matrix = pcd_tools::RegistrationMultiScaleIcp(
                    source, target, voxelsize_fine_, 1, {1, 2, 3});
            reg_matrix = multiScale_reg_matrix * reg_matrix;
            source->Transform(multiScale_reg_matrix);
            auto eva_result_coarse =
                open3d::pipelines::registration::EvaluateRegistration(
                        *source, *target, voxelsize_fine_ * 3);
            open3d::utility::LogInfo("eva fitness: {}",
                                         eva_result_coarse.fitness_);
            fitness_initial = eva_result_coarse.fitness_;
            *pcd_scan2map = *source;

            mat_odom2map_ = reg_matrix;
            lock_mat_odom2map_.unlock();
            auto loc_e =
                std::chrono::high_resolution_clock::now();  /// 结束定位计时
            loc_cost = std::chrono::duration_cast<std::chrono::microseconds>(
                           loc_e - loc_s)
                           .count() /
                       1000.0;
            RCLCPP_INFO(this->get_logger(), "localization cost: %f ms",
                            loc_cost);

            if (fitness_initial > threshold_fitness_init_) {
                    count_success += 1;
                    /// 连续两次定位成功后定位初始化成功
                    if (count_success >= 2) {
                        // break;
                    RCLCPP_INFO(this->get_logger(),
                                    "Localization initialization success");

                    return true;
                }
            } else {
                    count_success = 0;
            }
        }
    }

    return false;
        // open3d::utility::LogInfo("\n\n\nlocalization initialize
    // success!!!!\n\n\n");
}
void GloabalLocalization::Localization() {
    RCLCPP_INFO(this->get_logger(), "wait for Odometry_loc");
    // 等待接收到第一条里程计消息（通过检查timestamp是否有效）
    while (rclcpp::ok()) {
        // add mode change
        // 没进入定位模式，不初始化
        if (!enable_localization) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            continue;
        }
        if (timestamp_odom_.seconds() != 0.0) {
            break;
        }

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Waiting for Odometry_loc...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "Received Odometry_loc");

// 旧的初始化
#if 0
    RCLCPP_INFO(this->get_logger(), "wait for cloud_registered_1");
    // 等待接收到第一条点云消息（通过检查pcd_scan_cur_是否为空）
    while (rclcpp::ok())
    {
        lock_scan_.lock();
        bool has_scan = !pcd_scan_cur_->IsEmpty();
        lock_scan_.unlock();
        if (has_scan)
        {
            break;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for cloud_registered_1...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "Received cloud_registered_1");

    // initialize
    /****初始化定位****/
    mat_odom2map_ = mat_initialpose_; /// 初始位姿，从目前是从配置文件给
    LocalizationInitialize();

    /// 卡尔曼滤波初始化
    /// 使用当前 baselink2map 位置初始化卡尔曼滤波器
    Eigen::Matrix4d init_baselink2map = mat_odom2map_ * mat_baselink2odom_;
    double init_x = init_baselink2map(0, 3);
    double init_y = init_baselink2map(1, 3);
    double init_z = init_baselink2map(2, 3);

    RCLCPP_INFO(this->get_logger(), "Initializing Kalman filters with position: x=%.3f, y=%.3f, z=%.3f",
                init_x, init_y, init_z);

    // 检查参数数组大小是否有效
    if (kf_param_x_.size() >= 2 && kf_param_y_.size() >= 2 && kf_param_z_.size() >= 2)
    {
        kf_baselink_x_.KalmanFilterInit(kf_param_x_[0], kf_param_x_[1], init_x, 1);
        kf_baselink_y_.KalmanFilterInit(kf_param_y_[0], kf_param_y_[1], init_y, 1);
        kf_baselink_z_.KalmanFilterInit(kf_param_z_[0], kf_param_z_[1], init_z, 1);
        RCLCPP_INFO(this->get_logger(), "Kalman filters initialized: x[%.6f,%.6f], y[%.6f,%.6f], z[%.6f,%.6f]",
                    kf_param_x_[0], kf_param_x_[1], kf_param_y_[0], kf_param_y_[1],
                    kf_param_z_[0], kf_param_z_[1]);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid Kalman filter parameters! x_size=%zu, y_size=%zu, z_size=%zu",
                     kf_param_x_.size(), kf_param_y_.size(), kf_param_z_.size());
        RCLCPP_ERROR(this->get_logger(), "Kalman filters will NOT be initialized - using default values");
    }

    kalman_filter_odom2map_.KalmanFilterInit(kalman_processVar2_, kalman_estimatedMeasVar2_, init_z, 1);

    loc_initialized_ = true; /// 初始化成功

    RCLCPP_INFO(this->get_logger(), "Localization initialization complete, Kalman filters ready");
#endif

    double fitness = 0;
    auto   coordinate_ori =
        open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    auto coordinate_loc =
        open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    auto coordinate_OBB_scan =
        open3d::geometry::TriangleMesh::CreateCoordinateFrame(2.0);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan(
        new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scancrop(
        new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan2map(
        new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> source(
        new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target(
        new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> map_coarse_crop(
        new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> map_fine_crop(
        new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> pcd_submap(
        new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_map(
        new open3d::geometry::OrientedBoundingBox);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_scan(
        new open3d::geometry::OrientedBoundingBox);
    OBB_map->color_ = Eigen::Vector3d(1, 0.5, 0);
    OBB_map->extent_ = Eigen::Vector3d(60, 60, 40);

    OBB_scan->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_scan->color_ = Eigen::Vector3d(0, 1, 0);
    rclcpp::Time time_current = timestamp_odom_;
    rclcpp::Time time_last = time_current - rclcpp::Duration(3, 0);

    RCLCPP_INFO(this->get_logger(), "time_last: %f", time_last.seconds());
    RCLCPP_INFO(this->get_logger(), "time_current: %f", time_current.seconds());
    int scan_count = 0;

    std::string save_path =
        "/home/carlos/mount/E/lixin/data/yq_bag/scan_submap/";

    double time_diff_loc = 5;  /// 前后两次定位的时间差(s)
    std::chrono::high_resolution_clock::time_point
        time_last_loc;  /// 上次定位的完成时间点
    std::chrono::high_resolution_clock::time_point
           time_this_loc;  /// 当前定位的开始时间点
    double loc_cost = 0;   /// 定位耗时(ms)
    while (rclcpp::ok()) {
        // add mode change
        // ============================================
        // 1. Mapping模式
        // ============================================
        if (!enable_localization.load()) {
            RCLCPP_INFO(this->get_logger(), "mapping... stop open3d");
            std::this_thread::sleep_for(std::chrono::milliseconds(5000));

            continue;
        }

        // ============================================
        // 2. Localization模式，但是尚未初始化
        // ============================================
        if (!loc_initialized_.load()) {
            // ----------------------------------------------------
            // 2.1 等待FAST-LIO Odometry
            // ----------------------------------------------------

            rclcpp::Time odom_stamp;

            {
                std::lock_guard<std::mutex> lock(lock_timestamp_);

                odom_stamp = timestamp_odom_;
            }

            if (odom_stamp.seconds() == 0.0) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                                     2000, "Waiting for Odometry_loc...");

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                continue;
            }

            // ----------------------------------------------------
            // 2.2 等待新的定位点云
            //
            // set_mode_callback()进入LOCALIZATION之前
            // 已经把旧点云清空了
            //
            // 所以这里等到的是本次定位模式的新点云
            // ----------------------------------------------------

            bool has_scan = false;

            {
                std::lock_guard<std::mutex> lock(lock_scan_);

                has_scan = !pcd_scan_cur_->IsEmpty();
            }

            if (!has_scan) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                                     2000, "Waiting for localization scan...");

                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                continue;
            }

            RCLCPP_INFO(this->get_logger(),
                        "Start localization initialization");

            // ============================================================
            // 从全局定位初始化开始，一直到 Kalman 初始化结束、
            // loc_initialized_=true，全程作为一次不可分割的定位工作。
            // ============================================================
            std::unique_lock<std::mutex> init_work_lock(
                localization_work_mutex_);

            // 拿到锁以后重新确认模式
            if (!enable_localization.load()) {
                continue;
            }

            // ------------------------------------------------------------
            // 每次重新进入 LOCALIZATION，从 initialpose 开始
            // ------------------------------------------------------------
            {
                std::lock_guard<std::mutex> lock(lock_mat_odom2map_);

                // mat_initialpose_:
                //     T_map_base
                //
                // mat_baselink2odom_:
                //     T_odom_base
                //
                // 所以需要求：
                //
                // T_map_odom =
                //     T_map_base * inverse(T_odom_base)
                //
                mat_odom2map_ = mat_initialpose_ * mat_baselink2odom_.inverse();

                mat_baselink2map_ = mat_initialpose_;
            }

            // ------------------------------------------------------------
            // Open3D 全局定位初始化
            //
            // 注意：LocalizationInitialize() 内部不要再锁
            // localization_work_mutex_，否则会死锁。
            // ------------------------------------------------------------
            if (!LocalizationInitialize()) {
                continue;
            }

            // 初始化过程中可能被切到 MAPPING
            if (!enable_localization.load()) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Localization mode cancelled during initialization");

                continue;
            }

            // ------------------------------------------------------------
            // Kalman 初始化
            // ------------------------------------------------------------
            Eigen::Matrix4d init_baselink2map =
                mat_odom2map_ * mat_baselink2odom_;

            double init_x = init_baselink2map(0, 3);
            double init_y = init_baselink2map(1, 3);
            double init_z = init_baselink2map(2, 3);

            RCLCPP_INFO(this->get_logger(),
                        "Initializing Kalman filters with position: "
                        "x=%.3f, y=%.3f, z=%.3f",
                        init_x, init_y, init_z);

            if (kf_param_x_.size() >= 2 && kf_param_y_.size() >= 2 &&
                kf_param_z_.size() >= 2) {
                kf_baselink_x_.KalmanFilterInit(kf_param_x_[0], kf_param_x_[1],
                                                init_x, 1);

                kf_baselink_y_.KalmanFilterInit(kf_param_y_[0], kf_param_y_[1],
                                                init_y, 1);

                kf_baselink_z_.KalmanFilterInit(kf_param_z_[0], kf_param_z_[1],
                                                init_z, 1);
            }

            kalman_filter_odom2map_.KalmanFilterInit(
                kalman_processVar2_, kalman_estimatedMeasVar2_, init_z, 1);

            // ------------------------------------------------------------
            // 正常 ICP 状态复位
            // ------------------------------------------------------------
            last_loc_ = Eigen::Vector3d(0, 0, -5000);

            {
                std::lock_guard<std::mutex> lock(lock_timestamp_);
                time_current = timestamp_odom_;
            }

            time_last = time_current - rclcpp::Duration(3, 0);

            loc_cost = 0.0;

            // ============================================================
            // 最后才宣布定位初始化成功
            // ============================================================
            loc_initialized_.store(true);

            RCLCPP_INFO(this->get_logger(),
                        "Localization initialization complete, "
                        "normal ICP localization started");

            continue;
        }

        lock_timestamp_.lock();
        time_current = timestamp_odom_;
        lock_timestamp_.unlock();
        auto time_diff_frame = time_current.seconds() - time_last.seconds();
        time_last = time_current;
        if (std::fabs(time_diff_frame) < 1e-6) {
            loc_cost = 0.0;
            continue;
        }

        time_this_loc = std::chrono::high_resolution_clock::now();
        time_diff_loc = std::chrono::duration_cast<std::chrono::microseconds>(
                            time_this_loc - time_last_loc)
                                .count() /
                            1000000.0 +
                        loc_cost / 1000.0;

        if (time_diff_loc < loc_frequence_) {
            int wait_time = int((loc_frequence_ - time_diff_loc) * 1000);
            open3d::utility::LogInfo(
                "\n\ntime_this_loc: {}, time_last: {},\ntime_diff: {} s, sleep "
                "{} ms",
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    time_this_loc.time_since_epoch())
                        .count() /
                    1000.0,
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    time_last_loc.time_since_epoch())
                        .count() /
                    1000.0,
                time_diff_loc, wait_time);
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
        } else {
            open3d::utility::LogInfo(
                "\n\ntime_diff:{} s, localization right now", time_diff_loc);
        }

        // ============================================================
        // sleep期间可能已经被切换到MAPPING
        // ============================================================
        if (!enable_localization.load()) {
            continue;
        }

        // ============================================================
        // 开始一次完整ICP前获得工作锁
        //
        // MAPPING切换会等待这里释放。
        // ============================================================
        std::unique_lock<std::mutex> work_lock(localization_work_mutex_);

        // ============================================================
        // 非常重要：获得锁以后必须再检查一次
        //
        // 防止：
        // 线程检查到 LOCALIZATION
        //       ↓
        // MAPPING正好切换
        //       ↓
        // 当前线程才拿到mutex
        // ============================================================
        if (!enable_localization.load() || !loc_initialized_.load()) {
            continue;
        }

        auto loc_s =
            std::chrono::high_resolution_clock::now();  /// 开始定位计时

        lock_scan_.lock();
        if (pcd_scan_cur_->IsEmpty()) {
            lock_scan_.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        } else {
            /// 是否对odom2map进行kalman滤波
            if (filter_odom2map_) {
                kalman_filter_odom2map_.inputLatestNoisyMeasurement(
                    mat_odom2map_(2, 3));
                kalman_filter_odom2map_.inputLatestNoisyMeasurement(
                    mat_odom2map_(2, 3));  /// 两次
                mat_odom2map_kalman_ = mat_odom2map_;
                mat_odom2map_kalman_(2, 3) =
                    kalman_filter_odom2map_.getLatestEstimatedMeasurement();
            }
            Eigen::Matrix4d mat_baselink2odom_cur = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d mat_baselink2map_cur = Eigen::Matrix4d::Identity();

            mat_baselink2odom_cur = mat_baselink2odom_;
            mat_baselink2map_cur = mat_baselink2map_;
            *pcd_scan = *pcd_scan_cur_;
            lock_scan_.unlock();
            Eigen::Vector3d cur_loc(mat_baselink2map_cur(0, 3),
                                    mat_baselink2map_cur(1, 3),
                                    mat_baselink2map_cur(2, 3));
            auto            dis_motion = ComputeMotionDis(last_loc_, cur_loc);
            if (dis_motion > dis_updatemap_) {
                auto submap_s = std::chrono::high_resolution_clock::now();

                open3d::utility::LogInfo(
                    "\n***\n****\n***\n\n\nlast map update loc: x: {}, y: {}, z{},\n\
                now loc: x: {}, y: {}, z{}, 3d distance: {}, now needpdate submap",
                    last_loc_.x(), last_loc_.y(), last_loc_.z(), cur_loc.x(),
                    cur_loc.y(), cur_loc.z(), dis_motion);
                last_loc_ = cur_loc;
                OBB_map->center_ = mat_baselink2map_cur.block<3, 1>(0, 3);
                OBB_map->R_ = mat_baselink2map_cur.block<3, 3>(0, 0);

                /// 粗地图和精地图
                *map_fine_crop = *pcd_map_fine_->Crop(*OBB_map);

                auto submap_e = std::chrono::high_resolution_clock::now();
                auto submap_cost =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        submap_e - submap_s)
                        .count() /
                    1000.0;
                RCLCPP_INFO(this->get_logger(), "submap_cost: %f ms",
                            submap_cost);
            }

            OBB_scan->center_ = mat_baselink2odom_cur.block<3, 1>(0, 3);
            OBB_scan->R_ = mat_baselink2odom_cur.block<3, 3>(0, 0);

            auto reg0_s = std::chrono::high_resolution_clock::now();

            Eigen::Matrix4d reg_matrix = Eigen::Matrix4d::Identity();

            lock_mat_odom2map_.lock();
            reg_matrix = mat_odom2map_;

            *target = *map_fine_crop;
            open3d::utility::LogInfo(
                "before sample, target size: {}, has normal: {}",
                target->points_.size(),
                target->HasNormals() ? "true" : "false");
            if (target->points_.size() >
                static_cast<size_t>(maxpoints_target_)) {
                target = target->RandomDownSample(double(maxpoints_target_) /
                                                  target->points_.size());
            }
            open3d::utility::LogInfo(
                "after sample, target size: {}, has normal: {}",
                target->points_.size(),
                target->HasNormals() ? "true" : "false");

            source = pcd_scan->Crop(*OBB_scan);
            open3d::utility::LogInfo("source size: {}, maxpoints_source_: {}",
                                     source->points_.size(), maxpoints_source_);
            source = source->VoxelDownSample(voxelsize_fine_);
            open3d::utility::LogInfo("source size after voxel downsample: {}",
                                     source->points_.size());
            if (source->points_.size() >
                static_cast<size_t>(maxpoints_source_)) {
                source = source->RandomDownSample(double(maxpoints_source_) /
                                                  source->points_.size());
            }
            open3d::utility::LogInfo("after prerpocess: {}",
                                     source->points_.size());

            auto reg_result2 = pcd_tools::RegistrationIcp(
                source, target, voxelsize_fine_ * 2, reg_matrix, 1);
            reg_matrix = reg_result2.transformation_ * reg_matrix;
            auto eva_result2 =
                open3d::pipelines::registration::EvaluateRegistration(
                    *source, *target, voxelsize_fine_ * 4, reg_matrix);
            /// 给发布的置信度赋值
            loc_fitness_ = eva_result2.fitness_;
            open3d::utility::LogInfo("reg_result.fitness: {}, eva fitness: {}",
                                     reg_result2.fitness_,
                                     eva_result2.fitness_);
            /// 超过阈值才更新,防止因配准结果有问题而导致定位出问题
            if (loc_fitness_ > threshold_fitness_) {
                mat_odom2map_ = reg_matrix;
            }
            lock_mat_odom2map_.unlock();

            // save_path
            if (save_scan_) {
                pcd_scan->Transform(mat_baselink2odom_cur.inverse());
                pcd_scan2map->Transform(mat_baselink2map_cur.inverse());
                open3d::io::WritePointCloud(
                    save_path + std::to_string(scan_count) + "_ori.ply",
                    *pcd_scan);
                open3d::io::WritePointCloud(
                    save_path + std::to_string(scan_count) + "_crop.ply",
                    *pcd_scan2map);
                scan_count += 1;
            }

            auto loc_e =
                std::chrono::high_resolution_clock::now();  /// 结束定位计时
            time_last_loc = loc_e;
            loc_cost = std::chrono::duration_cast<std::chrono::microseconds>(
                           loc_e - loc_s)
                           .count() /
                       1000.0;
            RCLCPP_INFO(this->get_logger(), "localization cost: %f ms",
                        loc_cost);
        }
    }
}

void GloabalLocalization::StartLoc() {
    thread_loc_ = std::thread(&GloabalLocalization::Localization, this);
}

void GloabalLocalization::CallbackInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr
        initialpose) {
    if (!initialpose) {
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received new initial pose: x=%.3f y=%.3f",
                initialpose->pose.pose.position.x,
                initialpose->pose.pose.position.y);

    Eigen::Quaterniond rotation_q(initialpose->pose.pose.orientation.w,
                                  initialpose->pose.pose.orientation.x,
                                  initialpose->pose.pose.orientation.y,
                                  initialpose->pose.pose.orientation.z);

    Eigen::Matrix4d new_initialpose = Eigen::Matrix4d::Identity();

    new_initialpose.block<3, 3>(0, 0) = rotation_q.normalized().matrix();

    new_initialpose.block<3, 1>(0, 3) = Eigen::Vector3d(
        initialpose->pose.pose.position.x, initialpose->pose.pose.position.y,
        initialpose->pose.pose.position.z);

    // ============================================================
    // 等待当前正在执行的 ICP 完成
    // 防止这里修改 mat_odom2map_ 时 ICP 同时操作
    // ============================================================
    std::lock_guard<std::mutex> work_lock(localization_work_mutex_);

    {
        std::lock_guard<std::mutex> lock(lock_mat_odom2map_);

        // /initialpose 表示的是：
        //
        // T_map_base
        //
        mat_initialpose_ = new_initialpose;

        // 当前 FAST-LIO 提供的是：
        //
        // T_odom_base
        //
        // 因此：
        //
        // T_map_odom =
        //     T_map_base * inverse(T_odom_base)
        //
        mat_odom2map_ = new_initialpose * mat_baselink2odom_.inverse();

        // 当前机器人在 map 下的位置就是 initialpose
        mat_baselink2map_ = new_initialpose;
    }

    // ============================================================
    // 关键：
    // 强制重新进入 LocalizationInitialize()
    // ============================================================
    loc_initialized_.store(false);

    loc_fitness_ = 0.0;

    last_loc_ = Eigen::Vector3d(0, 0, -5000);

    RCLCPP_INFO(this->get_logger(),
                "Initial pose updated, restart localization initialization");
}

double GloabalLocalization::ComputeMotionDis(const Eigen::Vector3d &a,
                                             const Eigen::Vector3d &b) {
    return std::sqrt(std::pow(a.x() - b.x(), 2) + std::pow(a.y() - b.y(), 2) +
                     std::pow(a.z() - b.z(), 2));
}

// add mode change
void GloabalLocalization::set_mode_callback(
    const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr      res) {
    // ============================================================
    // 同一时刻只允许一个模式切换操作
    //
    // 防止：
    // LOCALIZATION请求还没结束
    // MAPPING请求又同时进来
    // ============================================================
    std::lock_guard<std::mutex> mode_lock(mode_switch_mutex_);

    // ============================================================
    // LOCALIZATION
    // ============================================================
    if (req->data) {
        // 已经在定位模式，不重复操作
        if (enable_localization.load()) {
            res->success = true;
            res->message = "Already in localization mode";
            return;
        }

        // 1. 加载最新保存的3D地图
        {
            std::lock_guard<std::mutex> work_lock(localization_work_mutex_);

            if (!LoadMap()) {
                enable_localization.store(false);

                res->success = false;
                res->message = "Failed to load localization map";

                return;
            }
        }

        // 2. 启动map_server  configure阶段重新读取最新2D_map.yaml
        if (!StartMapServer()) {
            enable_localization.store(false);

            res->success = false;
            res->message = "Failed to start map_server";

            return;
        }

        // 3. 清理上一次定位残留点云
        {
            std::lock_guard<std::mutex> lock(lock_scan_);

            while (!que_pcd_scan_.empty()) {
                que_pcd_scan_.pop();
            }

            pcd_scan_cur_->Clear();
        }

        // 4. 重置定位状态
        loc_initialized_.store(false);

        last_loc_ = Eigen::Vector3d(0, 0, -5000);

        // 5. 发布刚加载的新地图
        sensor_msgs::msg::PointCloud2 pc2_map;

        open3d_conversions::open3dToRos(*pcd_map_coarse_, pc2_map);

        pc2_map.header.frame_id = "map";
        pc2_map.header.stamp = this->now();

        pub_map_->publish(pc2_map);

        // 6. 最后才真正开启Open3D定位
        enable_localization.store(true);

        RCLCPP_INFO(this->get_logger(), "Open3D switch to LOCALIZATION");

        res->success = true;
        res->message = "Localization mode enabled";

        return;
    }

    // ============================================================
    // MAPPING
    // ============================================================

    enable_localization.store(false);

    // ============================================================
    // 等待当前正在进行的Open3D定位计算结束
    //
    // 如果ICP正在运行：
    //   这里会等待ICP结束
    //
    // 如果没有ICP运行：
    //   会立即获得锁然后立即释放
    //
    // 这一层保证从这里往后：
    //   Open3D定位线程已经不再使用 pcd_map_fine_ 等地图数据
    // ============================================================
    { std::lock_guard<std::mutex> work_lock(localization_work_mutex_); }

    loc_initialized_.store(false);

    // 停止2D静态地图
    if (!StopMapServer()) {
        res->success = false;
        res->message = "Failed to stop map_server";

        RCLCPP_ERROR(this->get_logger(),
                     "Failed to switch to MAPPING: map_server stop failed");

        return;
    }

    RCLCPP_INFO(this->get_logger(), "Open3D switch to MAPPING");

    res->success = true;
    res->message = "Mapping mode enabled";
}

// add mode change
bool GloabalLocalization::LoadMap() {
    auto map = std::make_shared<open3d::geometry::PointCloud>();

    if (!open3d::io::ReadPointCloud(path_map_, *map) || map->IsEmpty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s",
                     path_map_.c_str());

        return false;
    }

    pcd_map_ori_ = map;

    if (!pcd_map_ori_->HasColors()) {
        pcd_map_ori_->PaintUniformColor({1, 0, 0});
    }

    pcd_map_coarse_ = pcd_map_ori_->VoxelDownSample(voxelsize_coarse_);

    pcd_map_coarse_->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(voxelsize_coarse_ * 2, 30));

    pcd_map_fine_ = pcd_map_ori_->VoxelDownSample(voxelsize_fine_);

    pcd_map_fine_->EstimateNormals(
        open3d::geometry::KDTreeSearchParamHybrid(voxelsize_fine_ * 2, 30));

    RCLCPP_INFO(this->get_logger(), "Reloaded localization map: %zu points",
                pcd_map_ori_->points_.size());

    return true;
}

// add map server
bool GloabalLocalization::ChangeMapServerState(uint8_t transition_id) {
    if (!map_server_change_state_client_->wait_for_service(
            std::chrono::seconds(2))) {
        RCLCPP_ERROR(this->get_logger(),
                     "map_server change_state service not available");

        return false;
    }

    auto request =
        std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();

    request->transition.id = transition_id;

    auto future = map_server_change_state_client_->async_send_request(request);

    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        RCLCPP_ERROR(this->get_logger(), "map_server state transition timeout");

        return false;
    }

    auto response = future.get();

    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(),
                     "map_server state transition failed, id=%d",
                     transition_id);

        return false;
    }

    return true;
}

bool GloabalLocalization::StartMapServer() {
    auto state = GetMapServerState();

    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        return true;
    }

    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        if (!ChangeMapServerState(
                lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)) {
            return false;
        }

        state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    }

    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        return ChangeMapServerState(
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    }

    return false;
}

bool GloabalLocalization::StopMapServer() {
    auto state = GetMapServerState();

    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        return true;
    }

    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        if (!ChangeMapServerState(
                lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE)) {
            return false;
        }

        state = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
    }

    if (state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        return ChangeMapServerState(
            lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    }

    return false;
}

uint8_t GloabalLocalization::GetMapServerState() {
    if (!map_server_get_state_client_->wait_for_service(
            std::chrono::seconds(2))) {
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    auto future = map_server_get_state_client_->async_send_request(request);

    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    return future.get()->current_state.id;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GloabalLocalization>();

    // 使用多线程执行器，可以指定线程数
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),
                                                      2);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}