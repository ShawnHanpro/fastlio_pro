#pragma once

#include <QtWidgets>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

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

    QTimer *vel_timer_;
    QLineEdit *domain_edit_;
    QLineEdit *lin_edit_;
    QLineEdit *ang_edit_;
    QLabel *mode_label_;
    QPushButton *save_btn_;
    QLabel *status_;
    double v_ = 0.0, w_ = 0.0;
    double lin_vel_ = LIN_VEL, ang_vel_ = ANG_VEL; // 应用按钮可实时修改
};
