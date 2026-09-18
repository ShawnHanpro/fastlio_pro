#pragma once

#include <QtWidgets>
#include <optional>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

class MapWidget;

class ChassisPanel : public QWidget {
    Q_OBJECT
public:
    explicit ChassisPanel(QWidget *parent = nullptr);
    ~ChassisPanel() override;

    // 可按需修改
    static constexpr const char *SRV_SYSTEM_MODE  = "/system_mode"; // true=定位 false=建图
    static constexpr const char *SRV_SAVE_MAP     = "/map_save";
    static constexpr const char *CMD_VEL_TOPIC    = "/cmd_vel";
    static constexpr const char *POSE_TOPIC       = "/baselink2map"; // open3d_loc 发布的 map->base_link 位姿
    static constexpr double LIN_VEL = 0.3;
    static constexpr double ANG_VEL = 0.8;
    static constexpr int DEFAULT_DOMAIN = 98;
    // 域切换时 sed 更新的启动脚本、需 systemctl restart 的自启动服务
    static const QList<QString> DOMAIN_SCRIPTS;
    static const QList<QString> SYSTEMD_UNITS;

protected:
    bool eventFilter(QObject *watched, QEvent *event) override;

private:
    void initRos(int domain);
    void resetRos();
    void applyDomain(int domain);
    void setMode(bool localization);
    void saveMap();
    void publishTwist(double v, double w);
    // 导览点/途经点采集，对应 merman_common/waypoints/save_waypoints.py
    void recordWaypoint(const QString &type); // "task" 导览点 / "via" 途经点
    void editWaypoint(int id); // 按点位 id 用当前位姿覆盖其 x/y/theta，其余内容不动
    void insertWaypoint(int id); // 当前位姿作为新点插到 id 位置，原 id 起的点位依次 +1
    void deleteWaypoint(int id); // 删除 id 点位，之后的点位依次 -1
    void showPose();
    void clearWaypoints();
    bool currentPose(double &x, double &y, double &theta);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr mode_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_cli_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    std::mutex pose_mutex_;
    nav_msgs::msg::Odometry last_pose_;
    bool has_pose_ = false;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread spin_thread_;

    // ---- 服务调用与 DOMAIN 切换的防卡死设计 ----
    // GUI 线程上没有任何阻塞等待:服务请求只异步发出,
    // 由 QTimer 轮询 future;响应在网络上丢失时由超时兜底报错,
    // 而不是界面无声挂死或被 wait_for_service 强占数秒。
    QTimer *mode_watchdog_ = nullptr; // 100ms 轮询 /system_mode 响应
    QTimer *save_watchdog_ = nullptr; // 200ms 轮询 /map_save 响应
    QElapsedTimer mode_elapsed_;      // 在途请求已等待的时长
    QElapsedTimer save_elapsed_;
    // FutureAndRequestId = async_send_request 的返回类型,继承自 shared_future,
    // 既能 wait_for 轮询,又能用于超时后 remove_pending_request;
    // 其默认构造被删除,用 optional 持有表示"当前无在途请求"
    std::optional<rclcpp::Client<std_srvs::srv::SetBool>::FutureAndRequestId> mode_future_;
    std::optional<rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId> save_future_;
    bool pending_mode_ = false;   // 有在途模式请求,期间拒绝重复点击
    bool pending_mode_localization_ = false; // 在途请求的目标模式
    bool pending_save_ = false;
    bool domain_switching_ = false; // DOMAIN 后台切换期间,所有 ROS 操作直接拒绝
    std::thread domain_thread_;     // resetRos/initRos 放后台,GUI 不扛 DDS 销毁耗时
    QPushButton *dom_btn_ = nullptr;

    QTimer *vel_timer_;
    MapWidget *map_; // 左侧 2D 地图显示(建图 /map_2d,定位 /map)
    QLineEdit *domain_edit_;
    QLineEdit *lin_edit_;
    QLineEdit *ang_edit_;
    QLineEdit *wp_id_edit_; // 待操作的点位 id
    QComboBox *wp_type_combo_; // 插入点位时的类型
    QLabel *mode_label_;
    QPushButton *save_btn_;
    QLabel *status_;
    double v_ = 0.0, w_ = 0.0;
    double lin_vel_ = LIN_VEL, ang_vel_ = ANG_VEL; // 应用按钮可实时修改
};
