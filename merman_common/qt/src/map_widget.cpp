#include "map_widget.h"

#include <cmath>

static constexpr const char *MAP_TOPIC  = "/map_2d"; // FAST-LIO 建图时发布
static constexpr const char *GMAP_TOPIC = "/map";    // map_server 定位时发布
static constexpr const char *POSE_TOPIC = "/baselink2map";
static constexpr const char *ODOM_TOPIC = "/Odometry_loc"; // FAST-LIO 里程计(建图系)
static constexpr const char *SCAN_TOPIC = "/nav2_scan";    // 实时激光(imu_link 系)

// imu_link 相对 base_link 的平移(mid360.yaml base_to_imu, 旋转为单位阵):
// scan 点先平移到 base_link 再叠加机器人位姿
static constexpr double IMU_OFF_X = 0.25;
static constexpr double IMU_OFF_Y = 0.192;

// 四分位数转平面角,同 chassis_panel.cpp 的 quaternionToTheta
static double quaternionToTheta(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// 两个消息谁的 header 时间戳更新;时间戳都为 0 时取 a
static bool newerThan(const nav_msgs::msg::OccupancyGrid &a,
                      const nav_msgs::msg::OccupancyGrid &b) {
    if (b.header.stamp.sec == 0 && b.header.stamp.nanosec == 0) return true;
    if (a.header.stamp.sec == 0 && a.header.stamp.nanosec == 0) return false;
    if (a.header.stamp.sec != b.header.stamp.sec)
        return a.header.stamp.sec > b.header.stamp.sec;
    return a.header.stamp.nanosec >= b.header.stamp.nanosec;
}

// /map_2d 只有 FAST-LIO 处于建图模式才发布,但 transient_local 会把最后一帧
// 锁存投递给晚加入的订阅者,单帧不能证明建图进行中;只有时间戳在前进的
// 第二帧起才发 mappingActive。建图中每秒一帧,信号限频 2 秒足够。
void MapWidget::emitMappingIfActive(const nav_msgs::msg::OccupancyGrid &msg) {
    const long long ns = static_cast<long long>(msg.header.stamp.sec) * 1000000000LL +
                         msg.header.stamp.nanosec;
    const bool advanced = seen_map2d_ && ns > last_map2d_ns_;
    seen_map2d_ = true;
    last_map2d_ns_ = ns;
    if (!advanced) return;
    const auto now = std::chrono::steady_clock::now();
    const long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                             now.time_since_epoch()).count();
    if (ms - last_map_signal_ms_.load() < 2000) return;
    last_map_signal_ms_.store(ms);
    emit mappingActive(); // spin 线程 emit,Qt 自动排队投递到 GUI 线程
}

MapWidget::MapWidget(QWidget *parent) : QWidget(parent) {
    setMinimumSize(400, 300);
    setMouseTracking(false);
    setFocusPolicy(Qt::ClickFocus); // 滚轮缩放时避免被父级控件抢焦点
}

void MapWidget::attachNode(rclcpp::Node::SharedPtr node) {
    // 必须带 transient_local,否则晚启动收不到发布端锁存的最近一帧。
    // 注意可靠性要分开选:
    //   /map    (map_server)     发布端 RELIABLE    -> 订阅用 RELIABLE。
    //     Fast DDS 对 best_effort+transient_local 的订阅者不投递锁存帧,
    //     而 map_server 只在激活时发一次,订阅用 best_effort 会一帧都收不到。
    //   /map_2d (FAST-LIO 建图)   发布端 BEST_EFFORT -> 订阅只能用 BEST_EFFORT,
    //     用 RELIABLE 会 QoS 不兼容;建图时每秒都有新帧,锁存帧收不到无影响。
    rclcpp::QoS gmap_qos(rclcpp::KeepLast(1));
    gmap_qos.reliable();
    gmap_qos.transient_local();

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.best_effort();
    map_qos.transient_local();

    map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        MAP_TOPIC, map_qos, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                map_2d_ = msg;
            }
            emitMappingIfActive(*msg);
            requestUpdate();
        });
    gmap_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        GMAP_TOPIC, gmap_qos, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(map_mutex_);
            gmap_ = msg;
            requestUpdate();
        });
    pose_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        POSE_TOPIC, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pose_x_ = msg->pose.pose.position.x;
            pose_y_ = msg->pose.pose.position.y;
            pose_theta_ = quaternionToTheta(msg->pose.pose.orientation);
            has_pose_ = true;
            requestUpdate(); // 内部限频,位姿高频也不会刷爆 GUI
        });
    // 建图模式下的位姿,与 /map_2d 同源同系(camera_init)
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        ODOM_TOPIC, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            odom_x_ = msg->pose.pose.position.x;
            odom_y_ = msg->pose.pose.position.y;
            odom_theta_ = quaternionToTheta(msg->pose.pose.orientation);
            has_odom_ = true;
            requestUpdate();
        });
    // 实时激光:传感器数据不需要锁存,best_effort 即可
    scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
        SCAN_TOPIC, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
            const auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(scan_mutex_);
            scan_ = msg;
            scan_recv_ms_ =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch())
                    .count();
            requestUpdate();
        });
}

void MapWidget::detachNode() {
    map_sub_.reset();
    gmap_sub_.reset();
    pose_sub_.reset();
    odom_sub_.reset();
    scan_sub_.reset();
    // 清缓存,切 DOMAIN 后不显示上一个域的旧图/旧位姿/旧激光
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map_2d_.reset();
        gmap_.reset();
    }
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        has_pose_ = false;
        has_odom_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        scan_.reset();
        scan_recv_ms_ = 0;
    }
    seen_map2d_ = false;
    last_map2d_ns_ = 0;
    last_map_signal_ms_ = 0;
    image_ = QImage();
    image_src_.reset();
    requestUpdate();
}

void MapWidget::reloadWaypoints(const QString &path) {
    waypoints_.clear();
    QFile f(path);
    if (f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        // 与 chassis_panel.cpp 一致的轻量解析:块以 "- id: N" 开始
        static const QRegularExpression id_re("^\\s*-?\\s*id:\\s*(\\d+)");
        static const QRegularExpression field_re("^\\s*(type|x|y|theta)\\s*:\\s*(\\S+)");
        const auto lines = QString::fromUtf8(f.readAll()).split('\n');
        for (const auto &line : lines) {
            const auto m = id_re.match(line);
            if (m.hasMatch()) {
                Wp wp;
                wp.id = m.captured(1).toInt();
                wp.type = "task";
                waypoints_.append(wp);
                continue;
            }
            if (waypoints_.isEmpty()) continue;
            const auto fm = field_re.match(line);
            if (!fm.hasMatch()) continue;
            Wp &wp = waypoints_.last();
            const QString &key = fm.captured(1);
            const QString &val = fm.captured(2);
            if (key == "type") wp.type = val;
            else if (key == "x") wp.x = val.toDouble();
            else if (key == "y") wp.y = val.toDouble();
            else if (key == "theta") wp.theta = val.toDouble();
        }
    }
    // 可能从后台 DOMAIN 切换线程调用,排队回 GUI 线程重绘
    QMetaObject::invokeMethod(this, [this] { update(); }, Qt::QueuedConnection);
}

QPointF MapWidget::worldToScreen(double x, double y) const {
    return QPointF(offset_.x() + x * scale_, offset_.y() - y * scale_);
}

QPointF MapWidget::screenToWorld(const QPointF &p) const {
    return QPointF((p.x() - offset_.x()) / scale_,
                   (offset_.y() - p.y()) / scale_);
}

void MapWidget::fitView() {
    if (!image_src_ || image_src_->info.width == 0 || image_src_->info.height == 0)
        return;
    const auto &info = image_src_->info;
    const double w_m = info.width * info.resolution;
    const double h_m = info.height * info.resolution;
    const double sx = (width() - 60) / w_m;
    const double sy = (height() - 60) / h_m;
    scale_ = std::max(0.5, std::min(sx, sy));
    const double cx = info.origin.position.x + w_m / 2.0;
    const double cy = info.origin.position.y + h_m / 2.0;
    offset_ = QPointF(width() / 2.0 - cx * scale_, height() / 2.0 + cy * scale_);
    update();
}

void MapWidget::requestUpdate() {
    // 位姿 20~50Hz,地图 1Hz,统一限频到 ~12Hz,避免 GUI 线程过载
    const auto now = std::chrono::steady_clock::now();
    const long long ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch())
            .count();
    if (ms - last_invoke_ms_.load() < 80) return;
    last_invoke_ms_.store(ms);
    QMetaObject::invokeMethod(this, [this] { update(); }, Qt::QueuedConnection);
}

// OccupancyGrid -> 8bit 索引色 QImage,逐行 memcpy 并上下翻转
void MapWidget::rebuildImage(
    std::shared_ptr<const nav_msgs::msg::OccupancyGrid> msg, const QString &topic) {
    const int w = static_cast<int>(msg->info.width);
    const int h = static_cast<int>(msg->info.height);
    if (w <= 0 || h <= 0 ||
        msg->data.size() < static_cast<size_t>(w) * static_cast<size_t>(h)) {
        image_ = QImage();
        image_src_.reset();
        return;
    }

    image_ = QImage(w, h, QImage::Format_Indexed8);
    // 0=白(自由) -> 100=黑(占用);-1(int8 转 uchar 后是 255)及其它非法值=深灰(未知)
    QVector<QRgb> table(256);
    for (int i = 0; i <= 100; ++i) {
        const int g = 255 - i * 255 / 100;
        table[i] = qRgb(g, g, g);
    }
    for (int i = 101; i < 256; ++i) table[i] = qRgb(40, 40, 40);
    image_.setColorTable(table);

    const auto *src = reinterpret_cast<const uchar *>(msg->data.data());
    for (int row = 0; row < h; ++row)
        std::memcpy(image_.scanLine(h - 1 - row), src + static_cast<size_t>(row) * w,
                    static_cast<size_t>(w));

    image_src_ = msg;
    image_topic_ = topic;
    if (!user_view_) fitView();
}

void MapWidget::drawWaypoints(QPainter &painter) {
    painter.setRenderHint(QPainter::Antialiasing, true);
    QFont f = painter.font();
    f.setPointSize(8);
    painter.setFont(f);
    const int r = 5;
    for (const auto &wp : waypoints_) {
        const QPointF p = worldToScreen(wp.x, wp.y);
        if (p.x() < -20 || p.y() < -20 || p.x() > width() + 20 ||
            p.y() > height() + 20)
            continue;
        if (wp.type == "via") { // 途经点:橙色方块
            painter.setPen(QPen(QColor("#e67e22"), 2));
            painter.setBrush(QBrush(QColor(230, 126, 34, 200)));
            painter.drawRect(QRectF(p.x() - r, p.y() - r, 2 * r, 2 * r));
        } else { // 导览点:蓝色圆
            painter.setPen(QPen(QColor("#2980b9"), 2));
            painter.setBrush(QBrush(QColor(41, 128, 185, 200)));
            painter.drawEllipse(p, r, r);
        }
        painter.setPen(Qt::white);
        painter.drawText(QRectF(p.x() + r + 2, p.y() - 8, 40, 16),
                        Qt::AlignLeft | Qt::AlignVCenter,
                        QString::number(wp.id));
        painter.setPen(QColor(60, 60, 60));
        painter.drawText(QRectF(p.x() + r + 1, p.y() - 9, 40, 16),
                        Qt::AlignLeft | Qt::AlignVCenter,
                        QString::number(wp.id));
    }
}

void MapWidget::drawRobot(QPainter &painter) {
    double x, y, theta;
    if (!robotPose(x, y, theta)) return;
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.save();
    painter.translate(worldToScreen(x, y));
    // 屏幕 y 向下,世界 CCW 角对应 Qt 顺时针 rotate 取负
    painter.rotate(-theta * 180.0 / M_PI);
    painter.setPen(QPen(Qt::red, 2));
    painter.setBrush(QBrush(QColor(231, 76, 60, 220)));
    QPolygonF arrow;
    arrow << QPointF(14, 0) << QPointF(-7, 7) << QPointF(-3, 0) << QPointF(-7, -7);
    painter.drawPolygon(arrow);
    painter.restore();
}

// 按当前显示的地图来源取配套位姿:
//   显示 /map_2d(建图)用 /Odometry_loc;显示 /map(定位)用 /baselink2map。
// 无地图时尽量显示:优先定位位姿,退而求其次用建图位姿。
bool MapWidget::robotPose(double &x, double &y, double &theta) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (image_topic_ == MAP_TOPIC) {
        if (!has_odom_) return false;
        x = odom_x_;
        y = odom_y_;
        theta = odom_theta_;
        return true;
    }
    if (has_pose_) {
        x = pose_x_;
        y = pose_y_;
        theta = pose_theta_;
        return true;
    }
    if (has_odom_) {
        x = odom_x_;
        y = odom_y_;
        theta = odom_theta_;
        return true;
    }
    return false;
}

// 实时激光叠加(rviz 风格):只画最新一帧,超时不清除残影。
// scan 在 imu_link 系:先平移到 base_link,再旋转平移到地图系。
void MapWidget::drawScan(QPainter &painter) {
    std::shared_ptr<const sensor_msgs::msg::LaserScan> scan;
    long long recv_ms = 0;
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        scan = scan_;
        recv_ms = scan_recv_ms_;
    }
    if (!scan || recv_ms == 0) return;
    const auto now = std::chrono::steady_clock::now();
    const long long now_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch())
            .count();
    if (now_ms - recv_ms > 1500) return; // 数据断流,不留残影

    double px, py, pth;
    if (!robotPose(px, py, pth)) return;

    const double cos_t = std::cos(pth);
    const double sin_t = std::sin(pth);
    QVector<QPointF> pts;
    pts.reserve(static_cast<int>(scan->ranges.size()));
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        const float r = scan->ranges[i];
        if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max)
            continue;
        const double a = scan->angle_min + i * scan->angle_increment;
        // imu_link -> base_link(外参旋转为单位阵,只做平移)
        const double bx = r * std::cos(a) + IMU_OFF_X;
        const double by = r * std::sin(a) + IMU_OFF_Y;
        // base_link -> map
        const double wx = px + cos_t * bx - sin_t * by;
        const double wy = py + sin_t * bx + cos_t * by;
        pts.append(worldToScreen(wx, wy));
    }
    if (pts.isEmpty()) return;

    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(46, 204, 113)); // 绿色,避开红箭头/蓝橙点位
    for (const auto &p : pts)
        painter.drawRect(QRectF(p.x() - 1, p.y() - 1, 2.5, 2.5));
}

void MapWidget::paintEvent(QPaintEvent *) {
    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);

    // 两路地图取 header 时间戳较新的一帧
    std::shared_ptr<const nav_msgs::msg::OccupancyGrid> sel;
    QString topic;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (map_2d_ && gmap_) {
            if (newerThan(*map_2d_, *gmap_)) {
                sel = map_2d_;
                topic = MAP_TOPIC;
            } else {
                sel = gmap_;
                topic = GMAP_TOPIC;
            }
        } else if (map_2d_) {
            sel = map_2d_;
            topic = MAP_TOPIC;
        } else if (gmap_) {
            sel = gmap_;
            topic = GMAP_TOPIC;
        }
    }

    if (!sel) {
        painter.setPen(QColor(120, 120, 120));
        painter.drawText(rect(), Qt::AlignCenter,
                         "无地图数据\n建图中显示 /map_2d,定位中显示 /map");
        // 无图时也画激光和位姿箭头(等价 rviz 不依赖地图显示 scan)
        drawScan(painter);
        drawRobot(painter);
        return;
    }

    if (image_src_ != sel) rebuildImage(sel, topic);
    if (image_.isNull()) return;

    const auto &info = image_src_->info;
    const QPointF tl = worldToScreen(info.origin.position.x,
                                     info.origin.position.y +
                                         info.height * info.resolution);
    const QSizeF sz(info.width * info.resolution * scale_,
                    info.height * info.resolution * scale_);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    painter.drawImage(QRectF(tl, sz), image_);

    drawScan(painter);
    drawWaypoints(painter);
    drawRobot(painter);

    // 左下角地图信息
    painter.setRenderHint(QPainter::SmoothPixmapTransform, false);
    painter.setPen(QColor(180, 180, 180));
    painter.drawText(10, height() - 20,
                     QString("%1  %2x%3 @ %4m")
                         .arg(image_topic_)
                         .arg(info.width)
                         .arg(info.height)
                         .arg(info.resolution));
}

void MapWidget::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    if (!user_view_) fitView();
}

void MapWidget::wheelEvent(QWheelEvent *event) {
    user_view_ = true;
    // 缩放保持光标下的世界点不动
    const QPointF world = screenToWorld(event->position());
    const double f = std::pow(1.15, event->angleDelta().y() / 120.0);
    scale_ = std::max(0.05, std::min(2000.0, scale_ * f));
    const QPointF pos = event->position();
    offset_ = QPointF(pos.x() - world.x() * scale_, pos.y() + world.y() * scale_);
    update();
}

void MapWidget::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        user_view_ = true;
        panning_ = true;
        last_mouse_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
    }
}

void MapWidget::mouseMoveEvent(QMouseEvent *event) {
    if (!panning_) return;
    offset_ += event->pos() - last_mouse_;
    last_mouse_ = event->pos();
    update();
}

void MapWidget::mouseReleaseEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton && panning_) {
        panning_ = false;
        setCursor(Qt::ArrowCursor);
    }
}

void MapWidget::mouseDoubleClickEvent(QMouseEvent *event) {
    QWidget::mouseDoubleClickEvent(event);
    user_view_ = false;
    fitView();
}
