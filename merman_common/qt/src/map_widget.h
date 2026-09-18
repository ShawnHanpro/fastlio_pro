#pragma once

#include <QtWidgets>
#include <atomic>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// 2D 栅格地图显示:
//   建图模式订阅 FAST-LIO 的 /map_2d(地图变化时每秒刷新),
//   定位模式显示 map_server 的 /map(已保存的完整地图),
//   两路都按 transient_local 订阅,晚加入也能收到最近一帧,
//   显示时取 header 时间戳较新的一帧。无图时只画黑屏。
// 叠加机器人位姿箭头、/nav2_scan 实时激光(rviz 风格)
// 和 waypoints.yaml 里的点位。
// 滚轮缩放、左键拖拽平移、双击恢复自适应全图。
//
// 位姿来源与地图来源配套:
//   显示 /map   (定位)用 /baselink2map(open3d, 已存地图系);
//   显示 /map_2d (建图)用 /Odometry_loc(FAST-LIO, 建图同源同系)。
class MapWidget : public QWidget {
    Q_OBJECT
public:
    explicit MapWidget(QWidget *parent = nullptr);

    // 挂到面板的 ROS 节点上(initRos 之后调用);换 DOMAIN 前先 detachNode
    void attachNode(rclcpp::Node::SharedPtr node);
    void detachNode();

    // 重读点位文件并刷新显示;文件不存在则清空点位
    void reloadWaypoints(const QString &path);

signals:
    // 连续收到 /map_2d 时间戳前进的新帧时发出。FAST-LIO 只在建图模式发布
    // /map_2d,面板据此在 /system_mode 服务不可用时也能正确显示"建图中"。
    void mappingActive();

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;

private:
    struct Wp {
        int id;
        QString type; // task 导览点 / via 途经点
        double x = 0, y = 0, theta = 0;
    };

    QPointF worldToScreen(double x, double y) const;
    QPointF screenToWorld(const QPointF &p) const;
    void fitView();               // 自适应显示整张地图
    void requestUpdate();         // spin 线程安全触发重绘(限频)
    void emitMappingIfActive(const nav_msgs::msg::OccupancyGrid &msg);
    void rebuildImage(std::shared_ptr<const nav_msgs::msg::OccupancyGrid> msg,
                      const QString &topic);
    void drawWaypoints(QPainter &painter);
    void drawRobot(QPainter &painter);
    void drawScan(QPainter &painter);
    // 按当前显示的地图来源取配套位姿,见类注释;无可用位姿返回 false
    bool robotPose(double &x, double &y, double &theta);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;  // /map_2d 建图
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr gmap_sub_; // /map 定位
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;      // /baselink2map
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;      // /Odometry_loc
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;  // /nav2_scan

    std::mutex map_mutex_;
    std::shared_ptr<const nav_msgs::msg::OccupancyGrid> map_2d_; // 最新建图帧
    std::shared_ptr<const nav_msgs::msg::OccupancyGrid> gmap_;   // 最新定位地图

    // 由当前显示消息转出的图像:行已上下翻转(行 0 对应 y 最大),可与 drawImage 直接配合
    QImage image_;
    std::shared_ptr<const nav_msgs::msg::OccupancyGrid> image_src_;
    QString image_topic_;

    std::mutex pose_mutex_;
    bool has_pose_ = false;   // /baselink2map, 定位地图系
    double pose_x_ = 0, pose_y_ = 0, pose_theta_ = 0;
    bool has_odom_ = false;   // /Odometry_loc, 建图(camera_init/odom)系
    double odom_x_ = 0, odom_y_ = 0, odom_theta_ = 0;

    std::mutex scan_mutex_;
    std::shared_ptr<const sensor_msgs::msg::LaserScan> scan_; // 最新一帧激光
    long long scan_recv_ms_ = 0; // 收到时刻(本机单调时钟),用于超时清除残影

    std::atomic<long long> last_invoke_ms_{0}; // requestUpdate 限频用
    bool seen_map2d_ = false;       // 已收过 /map_2d(首帧可能是 transient_local 锁存旧图)
    long long last_map2d_ns_ = 0;   // 上一帧 /map_2d 的 header 时间戳
    std::atomic<long long> last_map_signal_ms_{0}; // mappingActive 信号限频
    QList<Wp> waypoints_;

    // 视图:scale 为像素/米,offset 为世界原点(0,0)在部件上的像素位置
    double scale_ = 20.0;
    QPointF offset_ = QPointF(40, 40);
    bool user_view_ = false; // 用户滚轮/拖拽后停止自适应,双击恢复
    bool panning_ = false;
    QPointF last_mouse_;
};
