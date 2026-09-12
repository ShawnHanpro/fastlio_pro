#pragma once
#include <vector>
#include <cstdint>
#include <string>
#include <Eigen/Dense>
#include <functional>
// #include <sensor_msgs/msg/image.hpp>


struct LaserScan {

    //! System time when first range was measured in nanoseconds
    int64_t start_time;
    //! System time when last range was measured in nanoseconds
    int64_t end_time;

    float angle_min;            // start angle of the scan [rad]
    float angle_max;            // end angle of the scan [rad]
    float angle_increment;      // angular distance between measurements [rad]

    float time_increment;       // time between measurements [seconds] - if your scanner
                                // is moving, this will be used in interpolating position
                                // of 3d points
    float scan_time;            // time between scans [seconds]

    float range_min;            // minimum range value [m]
    float range_max;            // maximum range value [m]

    std::vector<float> ranges;  // range data [m]
    std::vector<float> intensities;  // intensity data [device-specific units]

    int relo_value = 0;  // 0: not relo; 1: match_full_map relo; 2: small_area relo
};

struct Image {
    uint32_t height;                // 图像高度（行数）
    uint32_t width;                 // 图像宽度（列数）
    std::string encoding;           // 像素编码格式 (如 "rgb8", "16UC1")
    uint8_t is_bigendian;           // 大小端标志 (0 为小端, 1 为大端)
    uint32_t step;                  // 单行数据的字节长度
    std::vector<uint8_t> data;      // 实际的像素数据缓冲区 (大小为 step * height)
};

struct Imu {
    int64_t time;
    float   roll;
    float   pitch;
    float   yaw;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
};

struct Odometry {
    int64_t time;
    float   x;
    float   y;
    float   yaw;
};

enum LidarType {
    LIVOX = 0,
};

enum class CommandSource
{
    UNKNOWN = 0,
    LOCAL,
    CLOUD,
    SBUS,
    TCP,
    MQTT,
    ROS2_TOPIC
};

struct CommunicationConfig
{
    bool enable_ros2_topic = true;
    bool enable_mqtt = false;
    bool enable_tcp = false;
    bool enable_sbus = false;

    std::string robot_sn = "unknown";

    std::string ros_node_name = "merman_communication";

    std::string robot_status_topic = "/merman/chassis/robot_status";
    std::string nav_state_topic = "/merman/chassis/nav_state";
    std::string slam_state_topic = "/merman/chassis/slam_state";

    std::string local_cmd_topic = "/merman/control/local_cmd";
    std::string cloud_cmd_topic = "/merman/control/cloud_cmd";
    std::string sbus_cmd_topic = "/merman/control/sbus_cmd";

    std::string sub_pose_topic = "/localization_pose";
    std::string sub_cmd_vel_topic = "/cmd_vel";
    std::string sub_audio_done_topic = "/audio_done";
    std::string sub_pause_task_topic = "/pause_task";
    
    std::string start_task_service = "/start_task";
    std::string stop_task_service = "/stop_task";

    std::string follow_waypoints_action = "/follow_waypoints";
    std::string navigation_frame = "map";

    std::string pub_cmd_vel_topic = "/cmd_vel_nav";
    std::string pub_audio_play_topic = "/station_arrival";
    std::string pose_arrived_service = "/merman_action/pose_arrival";

    int follow_waypoints_wait_timeout_sec = 2;

    int status_upload_period_ms = 200;
    int pose_log_throttle_ms = 2000;

    int qos_depth = 10;
    std::string qos_reliability = "reliable";
};

struct TaskConfig {
    std::string waypoints_file = "../waypoints/waypoints.yaml";

    double rotation_angle_tolerance_deg = 3.0;
    double rotation_kp = 1.5;
    double rotation_max_angular_speed_rad_s = 0.3;
    double rotation_min_angular_speed_rad_s = 0.1;
    int rotation_control_period_ms = 200;
    int rotation_stable_count_required = 4;
    int rotation_max_pose_failure_count = 5;
    int rotation_timeout_sec = 30;
};

struct Pose2D
{
    double x = 0.0;
    double y = 0.0;

    // 对应 JSON 中的 theta
    double theta = 0.0;
};

struct MotionStatus
{
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
};


/*
{
  "robotSn": "sn-sewpg-8s7kP2g9xR5d1m6",
  "topic": "robot_status",
  "data": {
    "pose": {
        "x": 0.0,
        "y": 0.0,
        "theta": 0.0
    },
    "map_id": 0,
    "motion": {
        "linear_velocity": 0.0,
        "angular_velocity": 0.0
    }
  },
  "timestamp": 1779430837872
}
*/
struct RobotStatus
{
    // 对应 JSON 中的 robotSn
    std::string robot_sn;

    // 默认事件名称
    std::string topic = "robot_status";

    Pose2D pose;

    int32_t map_id = 0;

    MotionStatus motion;

    float battery;

    // 毫秒时间戳
    int64_t timestamp_ms = 0;

    // 业务层已经生成完整 JSON 时可以直接使用
    std::string raw_json;
};

struct ChassisState
{
    double vx = 0.0;
    double vy = 0.0;
    double wz = 0.0;

    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_yaw = 0.0;

    bool enabled = false;
    bool emergency_stop = false;

    int64_t timestamp_ms = 0;

    std::string raw_json;
};

struct CommunicationCommand
{
    CommandSource source = CommandSource::UNKNOWN;
    std::string name;
    std::string payload_json;
    int64_t timestamp_ms = 0;

    std::string type;
    std::string payload;
};

using CommandCallback = std::function<void(const CommunicationCommand&)>;

struct BatteryData {
    double voltage = 0.0;       // V
    double current = 0.0;       // A, positive/negative follows battery protocol
    double remain_ah = 0.0;     // Ah
    double avg_temp = 0.0;      // Celsius
    double env_temp = 0.0;      // Celsius
    double soc = 0.0;           // Percent
    double soh = 0.0;           // Percent
    int cycles = 0;
    double backup_time = 0.0;
    int work_mode = 0;
    int alarm = 0;
    int protect = 0;
    int fault_status = 0;
};

struct BatteryConfig {
    // RS485 serial config
    std::string port = "/dev/ttyTHS1";
    int baudrate = 9600;
    int slave_id = 1;
    double serial_timeout_sec = 1.0;

    // Loop config. Same meaning as Python publish_rate, but no ROS publishing here.
    double poll_rate_hz = 1.0;

    // Battery register range: 0x1000 ~ 0x100C
    uint16_t start_register = 0x1000;
    uint16_t register_count = 13;

    // LED board config, same RS485 bus
    bool led_enabled = true;
    int led_board_id = 2;
    bool led_update_on_change_only = true;
    double led_heartbeat_sec = 30.0;

    // SOC thresholds for 5-level battery LED bar
    double level1_max_soc = 28.0;
    double level2_max_soc = 46.0;
    double level3_max_soc = 64.0;
    double level4_max_soc = 82.0;

    // Optional RGB status LED
    bool rgb_status_enabled = false;
    double rgb_low_soc_threshold = 20.0;

    // Optional: turn off level LED when stop() is called.
    bool led_off_on_stop = false;
};

struct NavigationWaypoint
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    // 以下字段只在业务层使用
    int32_t waypoint_id = 0;
    int32_t audio_id = 0;
};
