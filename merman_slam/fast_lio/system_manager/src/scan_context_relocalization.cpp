#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <functional>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <scancontext_ros2/Scancontext.h>

namespace {
constexpr double kPi = 3.14159265358979323846;

double normalize_angle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    geometry_msgs::msg::Quaternion msg;
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    msg.w = q.w();
    return msg;
}
}  // namespace

class ScanContextRelocalization : public rclcpp::Node {
public:
    ScanContextRelocalization()
        : Node("scan_context_relocalization") {
        declare_parameters();
        read_parameters();
        configure_scan_context();

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/global_relocalization_pose",
            rclcpp::QoS(1).reliable().transient_local());

        candidates_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/global_relocalization/candidates",
            rclcpp::QoS(1).reliable().transient_local());

        aligned_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/global_relocalization/aligned_cloud",
            rclcpp::QoS(1).reliable().transient_local());

        if (publish_initialpose_) {
            initialpose_pub_ =
                this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "/initialpose", rclcpp::QoS(1).reliable());
        }

        lidar_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            lidar_topic_, rclcpp::SensorDataQoS(),
            std::bind(&ScanContextRelocalization::lidar_callback, this,
                      std::placeholders::_1));

        relocalization_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/global_relocalization",
            std::bind(&ScanContextRelocalization::relocalize_callback, this,
                      std::placeholders::_1, std::placeholders::_2));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        if (!load_database()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Scan Context database initialization failed");
        }

        RCLCPP_INFO(this->get_logger(),
                    "scan_context_relocalization started");
        RCLCPP_INFO(this->get_logger(), "database_mode: %s",
                    database_mode_.c_str());
        if (database_mode_ == "keyframes") {
            RCLCPP_INFO(this->get_logger(), "keyframe_root: %s",
                        keyframe_root_.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "map_path: %s", map_path_.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "lidar_topic: %s", lidar_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "database candidates: %zu",
                    candidates_.size());
        RCLCPP_INFO(this->get_logger(), "publish_initialpose: %s",
                    publish_initialpose_ ? "true" : "false");
    }

private:
    struct CandidatePose {
        double x{0.0};
        double y{0.0};
        double z{0.0};
        double qx{0.0};
        double qy{0.0};
        double qz{0.0};
        double qw{1.0};
        double yaw{0.0};
        std::size_t local_points{0};
        std::string pcd_file;
    };

    struct MatchResult {
        std::size_t database_index{0};
        double distance{std::numeric_limits<double>::max()};
        int sector_shift{0};
        double yaw{0.0};
    };

    void declare_parameters() {
        this->declare_parameter<std::string>(
            "map_path",
            "/home/niic/slam_nav/merman_common/maps/3D_map.pcd");
        this->declare_parameter<std::string>("database_mode", "keyframes");
        this->declare_parameter<std::string>(
            "keyframe_root",
            "/home/niic/slam_nav/merman_common/maps/scan_context");
        this->declare_parameter<std::string>("keyframe_poses_file", "poses.txt");
        this->declare_parameter<std::string>("lidar_topic", "/livox/lidar");
        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("fallback_lidar_frame", "imu_link");

        this->declare_parameter<double>("database_grid_step", 2.0);
        this->declare_parameter<double>("database_margin", 0.0);
        this->declare_parameter<double>("map_base_z", 0.0);
        this->declare_parameter<int>("min_local_map_points", 80);
        this->declare_parameter<double>("map_voxel_size", 0.20);
        this->declare_parameter<double>("query_voxel_size", 0.20);
        this->declare_parameter<double>("query_min_range", 0.50);
        this->declare_parameter<double>("query_max_range", 20.0);
        this->declare_parameter<int>("min_query_points", 150);
        this->declare_parameter<double>("tf_timeout_sec", 0.30);

        this->declare_parameter<double>("sc_lidar_height", 2.0);
        this->declare_parameter<int>("sc_pc_num_ring", 20);
        this->declare_parameter<int>("sc_pc_num_sector", 60);
        this->declare_parameter<double>("sc_pc_max_radius", 20.0);
        this->declare_parameter<double>("sc_search_ratio", 0.10);
        this->declare_parameter<int>("sc_knn_candidates", 20);
        this->declare_parameter<int>("sc_top_k", 5);
        this->declare_parameter<double>("sc_distance_threshold", 0.35);
        this->declare_parameter<bool>("require_sc_threshold", false);

        // Scan Context returns the sector shift needed to rotate the map
        // descriptor onto the query descriptor. For a base-frame query cloud,
        // map->base yaw is normally the negative of that shift.
        this->declare_parameter<double>("yaw_sign", -1.0);
        this->declare_parameter<double>("yaw_offset", 0.0);

        this->declare_parameter<bool>("publish_initialpose", false);
        this->declare_parameter<double>("initialpose_xy_variance", 4.0);
        this->declare_parameter<double>("initialpose_z_variance", 1.0);
        this->declare_parameter<double>("initialpose_rp_variance", 1.0);
        this->declare_parameter<double>("initialpose_yaw_variance", 0.5);
    }

    void read_parameters() {
        map_path_ = this->get_parameter("map_path").as_string();
        database_mode_ = this->get_parameter("database_mode").as_string();
        keyframe_root_ = this->get_parameter("keyframe_root").as_string();
        keyframe_poses_file_ =
            this->get_parameter("keyframe_poses_file").as_string();
        lidar_topic_ = this->get_parameter("lidar_topic").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        fallback_lidar_frame_ =
            this->get_parameter("fallback_lidar_frame").as_string();

        database_grid_step_ =
            this->get_parameter("database_grid_step").as_double();
        database_margin_ = this->get_parameter("database_margin").as_double();
        map_base_z_ = this->get_parameter("map_base_z").as_double();
        min_local_map_points_ =
            this->get_parameter("min_local_map_points").as_int();
        map_voxel_size_ = this->get_parameter("map_voxel_size").as_double();
        query_voxel_size_ = this->get_parameter("query_voxel_size").as_double();
        query_min_range_ = this->get_parameter("query_min_range").as_double();
        query_max_range_ = this->get_parameter("query_max_range").as_double();
        min_query_points_ = this->get_parameter("min_query_points").as_int();
        tf_timeout_sec_ = this->get_parameter("tf_timeout_sec").as_double();

        sc_lidar_height_ = this->get_parameter("sc_lidar_height").as_double();
        sc_pc_num_ring_ = this->get_parameter("sc_pc_num_ring").as_int();
        sc_pc_num_sector_ = this->get_parameter("sc_pc_num_sector").as_int();
        sc_pc_max_radius_ =
            this->get_parameter("sc_pc_max_radius").as_double();
        sc_search_ratio_ = this->get_parameter("sc_search_ratio").as_double();
        sc_knn_candidates_ =
            this->get_parameter("sc_knn_candidates").as_int();
        sc_top_k_ = this->get_parameter("sc_top_k").as_int();
        sc_distance_threshold_ =
            this->get_parameter("sc_distance_threshold").as_double();
        require_sc_threshold_ =
            this->get_parameter("require_sc_threshold").as_bool();

        yaw_sign_ = this->get_parameter("yaw_sign").as_double();
        yaw_offset_ = this->get_parameter("yaw_offset").as_double();

        publish_initialpose_ =
            this->get_parameter("publish_initialpose").as_bool();
        initialpose_xy_variance_ =
            this->get_parameter("initialpose_xy_variance").as_double();
        initialpose_z_variance_ =
            this->get_parameter("initialpose_z_variance").as_double();
        initialpose_rp_variance_ =
            this->get_parameter("initialpose_rp_variance").as_double();
        initialpose_yaw_variance_ =
            this->get_parameter("initialpose_yaw_variance").as_double();
    }

    void configure_scan_context() {
        sc_manager_.LIDAR_HEIGHT = sc_lidar_height_;
        sc_manager_.PC_NUM_RING = sc_pc_num_ring_;
        sc_manager_.PC_NUM_SECTOR = sc_pc_num_sector_;
        sc_manager_.PC_MAX_RADIUS = sc_pc_max_radius_;
        sc_manager_.PC_UNIT_SECTORANGLE =
            360.0 / static_cast<double>(sc_pc_num_sector_);
        sc_manager_.PC_UNIT_RINGGAP =
            sc_pc_max_radius_ / static_cast<double>(sc_pc_num_ring_);
        sc_manager_.SEARCH_RATIO = sc_search_ratio_;
        sc_manager_.SC_DIST_THRES = sc_distance_threshold_;
    }

    double yaw_from_quaternion(double x, double y, double z, double w) const {
        const double siny_cosp = 2.0 * (w * z + x * y);
        const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    bool build_database_tree() {
        if (candidates_.empty() || database_ringkeys_.empty()) {
            return false;
        }

        database_tree_ = std::make_unique<InvKeyTree>(
            sc_manager_.PC_NUM_RING, database_ringkeys_, 10);
        database_ready_ = true;
        return true;
    }

    bool load_keyframe_database() {
        namespace fs = std::filesystem;

        const fs::path root(keyframe_root_);
        const fs::path poses_path = root / keyframe_poses_file_;

        std::ifstream poses(poses_path);
        if (!poses.is_open()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Cannot open keyframe poses file: %s",
                         poses_path.string().c_str());
            return false;
        }

        candidates_.clear();
        database_descriptors_.clear();
        database_ringkeys_.clear();

        std::string line;
        std::size_t parsed_lines = 0;
        std::size_t loaded_keyframes = 0;

        RCLCPP_INFO(this->get_logger(),
                    "Loading Scan Context keyframe database: %s",
                    poses_path.string().c_str());

        while (std::getline(poses, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }

            std::istringstream iss(line);
            std::size_t id = 0;
            double stamp_sec = 0.0;
            CandidatePose candidate;
            std::string relative_pcd;

            if (!(iss >> id >> stamp_sec >> candidate.x >> candidate.y >>
                  candidate.z >> candidate.qx >> candidate.qy >> candidate.qz >>
                  candidate.qw >> relative_pcd)) {
                RCLCPP_WARN(this->get_logger(),
                            "Skip malformed keyframe pose line: %s",
                            line.c_str());
                continue;
            }
            parsed_lines++;

            const fs::path pcd_path = root / relative_pcd;
            pcl::PointCloud<SCPointType>::Ptr raw(
                new pcl::PointCloud<SCPointType>());
            if (pcl::io::loadPCDFile<SCPointType>(pcd_path.string(), *raw) != 0) {
                RCLCPP_WARN(this->get_logger(),
                            "Skip missing/bad keyframe PCD: %s",
                            pcd_path.string().c_str());
                continue;
            }

            pcl::PointCloud<SCPointType>::Ptr usable(
                new pcl::PointCloud<SCPointType>());
            usable->reserve(raw->size());
            for (const auto &p : raw->points) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
                    !std::isfinite(p.z)) {
                    continue;
                }
                const double range = std::hypot(p.x, p.y);
                if (range < query_min_range_ || range > sc_pc_max_radius_) {
                    continue;
                }
                usable->push_back(p);
            }

            if (map_voxel_size_ > 0.0 && !usable->empty()) {
                pcl::PointCloud<SCPointType>::Ptr downsampled(
                    new pcl::PointCloud<SCPointType>());
                pcl::VoxelGrid<SCPointType> voxel;
                voxel.setLeafSize(map_voxel_size_, map_voxel_size_,
                                  map_voxel_size_);
                voxel.setInputCloud(usable);
                voxel.filter(*downsampled);
                usable = downsampled;
            }

            if (static_cast<int>(usable->size()) < min_local_map_points_) {
                RCLCPP_WARN(this->get_logger(),
                            "Skip keyframe %zu: only %zu usable points",
                            id, usable->size());
                continue;
            }

            Eigen::MatrixXd descriptor = sc_manager_.makeScancontext(*usable);
            Eigen::MatrixXd ringkey =
                sc_manager_.makeRingkeyFromScancontext(descriptor);

            candidate.yaw = yaw_from_quaternion(
                candidate.qx, candidate.qy, candidate.qz, candidate.qw);
            candidate.local_points = usable->size();
            candidate.pcd_file = relative_pcd;

            candidates_.push_back(candidate);
            database_descriptors_.push_back(std::move(descriptor));
            database_ringkeys_.push_back(eig2stdvec(ringkey));
            loaded_keyframes++;
        }

        if (!build_database_tree()) {
            RCLCPP_ERROR(this->get_logger(),
                         "No valid keyframes were loaded from %s",
                         poses_path.string().c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Scan Context keyframe database ready: parsed=%zu loaded=%zu",
                    parsed_lines, loaded_keyframes);
        return true;
    }

    bool load_database() {
        database_ready_ = false;
        if (database_mode_ == "keyframes") {
            return load_keyframe_database();
        }
        if (database_mode_ == "static_grid") {
            return load_map_and_build_database();
        }

        RCLCPP_ERROR(this->get_logger(),
                     "Unknown database_mode='%s' (use keyframes or static_grid)",
                     database_mode_.c_str());
        return false;
    }

    bool load_map_and_build_database() {
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map(
            new pcl::PointCloud<pcl::PointXYZ>());

        if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path_, *raw_map) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map: %s",
                         map_path_.c_str());
            return false;
        }

        if (raw_map->empty()) {
            RCLCPP_ERROR(this->get_logger(), "Global map is empty");
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr finite_map(
            new pcl::PointCloud<pcl::PointXYZ>());
        finite_map->reserve(raw_map->size());

        float min_x = std::numeric_limits<float>::max();
        float min_y = std::numeric_limits<float>::max();
        float min_z = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float max_y = std::numeric_limits<float>::lowest();
        float max_z = std::numeric_limits<float>::lowest();

        for (const auto &p : raw_map->points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
                !std::isfinite(p.z)) {
                continue;
            }

            finite_map->push_back(p);
            min_x = std::min(min_x, p.x);
            min_y = std::min(min_y, p.y);
            min_z = std::min(min_z, p.z);
            max_x = std::max(max_x, p.x);
            max_y = std::max(max_y, p.y);
            max_z = std::max(max_z, p.z);
        }

        if (finite_map->empty()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Global map contains no finite points");
            return false;
        }

        map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        if (map_voxel_size_ > 0.0) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setLeafSize(map_voxel_size_, map_voxel_size_,
                              map_voxel_size_);
            voxel.setInputCloud(finite_map);
            voxel.filter(*map_cloud_);
        } else {
            *map_cloud_ = *finite_map;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Loaded map: raw=%zu finite=%zu voxel=%zu",
                    raw_map->size(), finite_map->size(), map_cloud_->size());
        RCLCPP_INFO(this->get_logger(),
                    "Map bounds: X[%.3f, %.3f] Y[%.3f, %.3f] Z[%.3f, %.3f]",
                    min_x, max_x, min_y, max_y, min_z, max_z);

        if (database_grid_step_ <= 0.0) {
            RCLCPP_ERROR(this->get_logger(),
                         "database_grid_step must be > 0");
            return false;
        }

        map_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
        map_kdtree_->setInputCloud(map_cloud_);

        candidates_.clear();
        database_descriptors_.clear();
        database_ringkeys_.clear();

        const double x_begin = static_cast<double>(min_x) + database_margin_;
        const double x_end = static_cast<double>(max_x) - database_margin_;
        const double y_begin = static_cast<double>(min_y) + database_margin_;
        const double y_end = static_cast<double>(max_y) - database_margin_;

        const double z_extent = std::max(
            std::abs(static_cast<double>(min_z) - map_base_z_),
            std::abs(static_cast<double>(max_z) - map_base_z_));
        const double kd_radius =
            std::sqrt(sc_pc_max_radius_ * sc_pc_max_radius_ +
                      z_extent * z_extent) + 0.05;

        std::size_t tested_grid_cells = 0;

        RCLCPP_INFO(this->get_logger(),
                    "Building Scan Context database: step=%.2f m, radius=%.2f m",
                    database_grid_step_, sc_pc_max_radius_);

        for (double x = x_begin; x <= x_end + 1e-6;
             x += database_grid_step_) {
            for (double y = y_begin; y <= y_end + 1e-6;
                 y += database_grid_step_) {
                tested_grid_cells++;

                pcl::PointXYZ center;
                center.x = static_cast<float>(x);
                center.y = static_cast<float>(y);
                center.z = static_cast<float>(map_base_z_);

                std::vector<int> indices;
                std::vector<float> sqr_dists;
                map_kdtree_->radiusSearch(center, kd_radius, indices,
                                          sqr_dists);

                pcl::PointCloud<SCPointType> local_cloud;
                local_cloud.reserve(indices.size());

                for (const int index : indices) {
                    const auto &p = map_cloud_->points[index];
                    const double dx = static_cast<double>(p.x) - x;
                    const double dy = static_cast<double>(p.y) - y;
                    const double planar_range = std::hypot(dx, dy);

                    if (planar_range > sc_pc_max_radius_) {
                        continue;
                    }

                    SCPointType local_point;
                    local_point.x = static_cast<float>(dx);
                    local_point.y = static_cast<float>(dy);
                    local_point.z =
                        static_cast<float>(static_cast<double>(p.z) - map_base_z_);
                    local_point.intensity = 0.0f;
                    local_cloud.push_back(local_point);
                }

                if (static_cast<int>(local_cloud.size()) <
                    min_local_map_points_) {
                    continue;
                }

                Eigen::MatrixXd descriptor =
                    sc_manager_.makeScancontext(local_cloud);
                Eigen::MatrixXd ringkey =
                    sc_manager_.makeRingkeyFromScancontext(descriptor);

                CandidatePose candidate;
                candidate.x = x;
                candidate.y = y;
                candidate.z = map_base_z_;
                candidate.yaw = 0.0;
                candidate.local_points = local_cloud.size();

                candidates_.push_back(candidate);
                database_descriptors_.push_back(std::move(descriptor));
                database_ringkeys_.push_back(eig2stdvec(ringkey));
            }
        }

        if (candidates_.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                         "No Scan Context database candidates were created. "
                         "Check map, grid step, radius and min_local_map_points.");
            return false;
        }

        database_tree_ = std::make_unique<InvKeyTree>(
            sc_manager_.PC_NUM_RING, database_ringkeys_, 10);

        database_ready_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Scan Context database ready: grid_cells=%zu, "
                    "valid_candidates=%zu",
                    tested_grid_cells, candidates_.size());
        return true;
    }

    void lidar_callback(
        const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
        if (!msg) {
            return;
        }

        std::lock_guard<std::mutex> lock(lidar_mutex_);
        latest_lidar_msg_ = msg;
    }

    pcl::PointCloud<SCPointType>::Ptr build_query_cloud(
        const livox_ros_driver2::msg::CustomMsg &msg) {
        const std::string source_frame =
            msg.header.frame_id.empty() ? fallback_lidar_frame_
                                        : msg.header.frame_id;

        bool need_transform = source_frame != base_frame_;
        tf2::Transform lidar_to_base;
        lidar_to_base.setIdentity();

        if (need_transform) {
            try {
                const auto tf_msg = tf_buffer_->lookupTransform(
                    base_frame_, source_frame, tf2::TimePointZero,
                    tf2::durationFromSec(tf_timeout_sec_));

                tf2::Quaternion q(
                    tf_msg.transform.rotation.x,
                    tf_msg.transform.rotation.y,
                    tf_msg.transform.rotation.z,
                    tf_msg.transform.rotation.w);
                tf2::Vector3 t(tf_msg.transform.translation.x,
                               tf_msg.transform.translation.y,
                               tf_msg.transform.translation.z);
                lidar_to_base = tf2::Transform(q, t);
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(),
                             "TF %s -> %s failed: %s",
                             source_frame.c_str(), base_frame_.c_str(),
                             e.what());
                return nullptr;
            }
        }

        pcl::PointCloud<SCPointType>::Ptr cloud(
            new pcl::PointCloud<SCPointType>());
        cloud->reserve(msg.points.size());

        for (const auto &p : msg.points) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
                !std::isfinite(p.z)) {
                continue;
            }

            tf2::Vector3 v(p.x, p.y, p.z);
            if (need_transform) {
                v = lidar_to_base * v;
            }

            const double planar_range = std::hypot(v.x(), v.y());
            if (planar_range < query_min_range_ ||
                planar_range > query_max_range_ ||
                planar_range > sc_pc_max_radius_) {
                continue;
            }

            SCPointType point;
            point.x = static_cast<float>(v.x());
            point.y = static_cast<float>(v.y());
            point.z = static_cast<float>(v.z());
            point.intensity = static_cast<float>(p.reflectivity);
            cloud->push_back(point);
        }

        if (cloud->empty()) {
            return cloud;
        }

        if (query_voxel_size_ > 0.0) {
            pcl::PointCloud<SCPointType>::Ptr filtered(
                new pcl::PointCloud<SCPointType>());
            pcl::VoxelGrid<SCPointType> voxel;
            voxel.setLeafSize(query_voxel_size_, query_voxel_size_,
                              query_voxel_size_);
            voxel.setInputCloud(cloud);
            voxel.filter(*filtered);
            return filtered;
        }

        return cloud;
    }

    std::vector<MatchResult> query_database(
        const pcl::PointCloud<SCPointType> &query_cloud) {
        std::vector<MatchResult> results;

        if (!database_tree_ || candidates_.empty()) {
            return results;
        }

        Eigen::MatrixXd query_descriptor =
            sc_manager_.makeScancontext(query_cloud);
        Eigen::MatrixXd query_ringkey =
            sc_manager_.makeRingkeyFromScancontext(query_descriptor);
        const std::vector<float> query_key = eig2stdvec(query_ringkey);

        const std::size_t knn_count = std::min<std::size_t>(
            static_cast<std::size_t>(std::max(1, sc_knn_candidates_)),
            candidates_.size());

        std::vector<std::size_t> candidate_indexes(knn_count);
        std::vector<float> out_dists_sqr(knn_count);

        nanoflann::KNNResultSet<float> knn_result(knn_count);
        knn_result.init(candidate_indexes.data(), out_dists_sqr.data());
        database_tree_->index->findNeighbors(
            knn_result, query_key.data(), nanoflann::SearchParams(10));

        results.reserve(knn_count);

        for (std::size_t i = 0; i < knn_count; ++i) {
            const std::size_t database_index = candidate_indexes[i];
            if (database_index >= database_descriptors_.size()) {
                continue;
            }

            Eigen::MatrixXd candidate_descriptor =
                database_descriptors_[database_index];
            auto distance_result = sc_manager_.distanceBtnScanContext(
                query_descriptor, candidate_descriptor);

            MatchResult result;
            result.database_index = database_index;
            result.distance = distance_result.first;
            result.sector_shift = distance_result.second;

            const double yaw_diff =
                static_cast<double>(result.sector_shift) *
                sc_manager_.PC_UNIT_SECTORANGLE * kPi / 180.0;
            const auto &candidate = candidates_[database_index];
            result.yaw = normalize_angle(
                candidate.yaw + yaw_sign_ * yaw_diff + yaw_offset_);

            results.push_back(result);
        }

        std::sort(results.begin(), results.end(),
                  [](const MatchResult &a, const MatchResult &b) {
                      return a.distance < b.distance;
                  });

        if (results.size() > static_cast<std::size_t>(std::max(1, sc_top_k_))) {
            results.resize(static_cast<std::size_t>(std::max(1, sc_top_k_)));
        }

        return results;
    }

    geometry_msgs::msg::Pose make_pose(const MatchResult &match) const {
        geometry_msgs::msg::Pose pose;
        const auto &candidate = candidates_[match.database_index];
        pose.position.x = candidate.x;
        pose.position.y = candidate.y;
        pose.position.z = candidate.z;
        pose.orientation = yaw_to_quaternion(match.yaw);
        return pose;
    }

    void publish_candidates(const std::vector<MatchResult> &matches,
                            const rclcpp::Time &stamp) {
        geometry_msgs::msg::PoseArray array;
        array.header.stamp = stamp;
        array.header.frame_id = map_frame_;

        for (const auto &match : matches) {
            array.poses.push_back(make_pose(match));
        }

        candidates_pub_->publish(array);
    }

    void publish_aligned_cloud(
        const pcl::PointCloud<SCPointType> &query_cloud,
        const MatchResult &best,
        const rclcpp::Time &stamp) {
        const auto &candidate = candidates_[best.database_index];
        const double c = std::cos(best.yaw);
        const double s = std::sin(best.yaw);

        pcl::PointCloud<SCPointType> aligned;
        aligned.reserve(query_cloud.size());

        for (const auto &p : query_cloud.points) {
            SCPointType out = p;
            out.x = static_cast<float>(
                c * static_cast<double>(p.x) -
                s * static_cast<double>(p.y) + candidate.x);
            out.y = static_cast<float>(
                s * static_cast<double>(p.x) +
                c * static_cast<double>(p.y) + candidate.y);
            out.z = static_cast<float>(
                static_cast<double>(p.z) + candidate.z);
            aligned.push_back(out);
        }

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(aligned, msg);
        msg.header.stamp = stamp;
        msg.header.frame_id = map_frame_;
        aligned_cloud_pub_->publish(msg);
    }

    void publish_initialpose(const geometry_msgs::msg::Pose &pose,
                             const rclcpp::Time &stamp) {
        if (!publish_initialpose_ || !initialpose_pub_) {
            return;
        }

        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = map_frame_;
        msg.pose.pose = pose;
        msg.pose.covariance.fill(0.0);
        msg.pose.covariance[0] = initialpose_xy_variance_;
        msg.pose.covariance[7] = initialpose_xy_variance_;
        msg.pose.covariance[14] = initialpose_z_variance_;
        msg.pose.covariance[21] = initialpose_rp_variance_;
        msg.pose.covariance[28] = initialpose_rp_variance_;
        msg.pose.covariance[35] = initialpose_yaw_variance_;

        initialpose_pub_->publish(msg);
    }

    void relocalize_callback(
        const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        // ========================================================
        // 每次全局重定位前刷新数据库
        // ========================================================

        if (database_mode_ == "keyframes") {
            RCLCPP_INFO(
                this->get_logger(),
                "Reload Scan Context keyframe database before relocalization");

            if (!load_database()) {
                response->success = false;
                response->message =
                    "Failed to reload Scan Context keyframe database";

                return;
            }
        }

        if (!database_ready_) {
            response->success = false;
            response->message = "Scan Context database is not ready";
            return;
        }

        livox_ros_driver2::msg::CustomMsg::SharedPtr lidar_msg;
        {
            std::lock_guard<std::mutex> lock(lidar_mutex_);
            lidar_msg = latest_lidar_msg_;
        }

        if (!lidar_msg) {
            response->success = false;
            response->message = "No Livox scan received yet";
            return;
        }

        const auto query_start = std::chrono::steady_clock::now();
        auto query_cloud = build_query_cloud(*lidar_msg);
        if (!query_cloud) {
            response->success = false;
            response->message = "Failed to transform/query LiDAR cloud";
            return;
        }

        if (static_cast<int>(query_cloud->size()) < min_query_points_) {
            std::ostringstream oss;
            oss << "Too few query points: " << query_cloud->size()
                << " < " << min_query_points_;
            response->success = false;
            response->message = oss.str();
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Global relocalization query: raw=%zu filtered=%zu",
                    lidar_msg->points.size(), query_cloud->size());

        const auto matches = query_database(*query_cloud);
        if (matches.empty()) {
            response->success = false;
            response->message = "Scan Context returned no candidates";
            return;
        }

        const auto &best = matches.front();
        const auto &best_candidate = candidates_[best.database_index];

        if (require_sc_threshold_ &&
            best.distance > sc_distance_threshold_) {
            std::ostringstream oss;
            oss << "Best Scan Context distance " << best.distance
                << " exceeds threshold " << sc_distance_threshold_;
            response->success = false;
            response->message = oss.str();
            return;
        }

        const rclcpp::Time stamp = this->now();
        publish_candidates(matches, stamp);

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = map_frame_;
        pose_msg.pose = make_pose(best);
        pose_pub_->publish(pose_msg);

        publish_aligned_cloud(*query_cloud, best, stamp);
        publish_initialpose(pose_msg.pose, stamp);

        for (std::size_t i = 0; i < matches.size(); ++i) {
            const auto &m = matches[i];
            const auto &candidate = candidates_[m.database_index];
            RCLCPP_INFO(
                this->get_logger(),
                "SC candidate[%zu]: db=%zu x=%.2f y=%.2f yaw=%.1f deg "
                "keyframe_yaw=%.1f deg distance=%.4f local_points=%zu",
                i, m.database_index, candidate.x, candidate.y,
                m.yaw * 180.0 / kPi,
                candidate.yaw * 180.0 / kPi,
                m.distance, candidate.local_points);
        }

        const auto query_end = std::chrono::steady_clock::now();
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(query_end - query_start)
                .count();

        RCLCPP_INFO(
            this->get_logger(),
            "Scan Context result: x=%.3f y=%.3f yaw=%.2f deg distance=%.4f "
            "time=%.2f ms",
            best_candidate.x, best_candidate.y,
            best.yaw * 180.0 / kPi, best.distance, elapsed_ms);

        std::ostringstream oss;
        oss << "SC coarse pose x=" << best_candidate.x
            << " y=" << best_candidate.y
            << " yaw_deg=" << best.yaw * 180.0 / kPi
            << " distance=" << best.distance;
        response->success = true;
        response->message = oss.str();
    }

private:
    std::string map_path_;
    std::string database_mode_{"keyframes"};
    std::string keyframe_root_;
    std::string keyframe_poses_file_{"poses.txt"};
    std::string lidar_topic_;
    std::string map_frame_;
    std::string base_frame_;
    std::string fallback_lidar_frame_;

    double database_grid_step_{2.0};
    double database_margin_{0.0};
    double map_base_z_{0.0};
    int min_local_map_points_{80};
    double map_voxel_size_{0.20};
    double query_voxel_size_{0.20};
    double query_min_range_{0.50};
    double query_max_range_{20.0};
    int min_query_points_{150};
    double tf_timeout_sec_{0.30};

    double sc_lidar_height_{2.0};
    int sc_pc_num_ring_{20};
    int sc_pc_num_sector_{60};
    double sc_pc_max_radius_{20.0};
    double sc_search_ratio_{0.10};
    int sc_knn_candidates_{20};
    int sc_top_k_{5};
    double sc_distance_threshold_{0.35};
    bool require_sc_threshold_{false};
    double yaw_sign_{-1.0};
    double yaw_offset_{0.0};

    bool publish_initialpose_{false};
    double initialpose_xy_variance_{4.0};
    double initialpose_z_variance_{1.0};
    double initialpose_rp_variance_{1.0};
    double initialpose_yaw_variance_{0.5};

    SCManager sc_manager_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr map_kdtree_;

    std::vector<CandidatePose> candidates_;
    std::vector<Eigen::MatrixXd> database_descriptors_;
    KeyMat database_ringkeys_;
    std::unique_ptr<InvKeyTree> database_tree_;
    bool database_ready_{false};

    std::mutex lidar_mutex_;
    livox_ros_driver2::msg::CustomMsg::SharedPtr latest_lidar_msg_;

    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr
        lidar_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr relocalization_srv_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr candidates_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        aligned_cloud_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        initialpose_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanContextRelocalization>());
    rclcpp::shutdown();
    return 0;
}
