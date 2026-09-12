#include "chassis_panel.h"

#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <cmath>
#include <cstdlib>

static const QString home = qEnvironmentVariable("HOME");
const QList<QString> ChassisPanel::DOMAIN_SCRIPTS = {
    home + "/slam_nav/merman_ros2_bridge/scripts/bringup_chassis.sh",
    home + "/slam_nav/merman_ros2_bridge/scripts/bringup_chassis_essential.sh",
};
const QList<QString> ChassisPanel::SYSTEMD_UNITS = {"merman-bringup.service"};

// 与 save_waypoints.py 同一份点位文件：merman_common/waypoints/waypoints.yaml
// 二进制在 merman_common/bin 下，用相对路径定位，不依赖工作空间的绝对位置
static QString waypointsFile() {
    return QCoreApplication::applicationDirPath() + "/../waypoints/waypoints.yaml";
}

// 四元数转平面角 theta，同 save_waypoints.py 的 quaternion_to_theta
static double quaternionToTheta(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// 保留 4 位小数并去掉末尾多余的 0，与 yaml 输出风格一致
static QString fmt(double v) {
    QString s = QString::number(v, 'f', 4);
    while (s.endsWith('0')) s.chop(1);
    if (s.endsWith('.')) s.chop(1);
    return s;
}

// 取现有 yaml 里最大的 id + 1，同 save_waypoints.py 的 next_waypoint_id
static int nextWaypointId(const QString &path) {
    int max_id = 0;
    QFile f(path);
    if (f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        static const QRegularExpression re("^\\s*-?\\s*id:\\s*(\\d+)");
        const auto lines = QString::fromUtf8(f.readAll()).split('\n');
        for (const auto &line : lines) {
            const auto m = re.match(line);
            if (m.hasMatch()) max_id = std::max(max_id, m.captured(1).toInt());
        }
    }
    return max_id + 1;
}

ChassisPanel::ChassisPanel(QWidget *parent) : QWidget(parent) {
    setWindowTitle("Merman 底盘控制");

    auto *mode_combo_ = new QComboBox;
    mode_combo_->addItem("建图");
    mode_combo_->addItem("定位");
    mode_combo_->setMinimumHeight(34);
    auto *apply_mode_btn = new QPushButton("确认模式");
    save_btn_ = new QPushButton("保存地图");
    save_btn_->setEnabled(false); // 只有确认处于建图模式后才能保存，防止覆盖已有地图
    mode_label_ = new QLabel("当前模式: 未知");
    auto *svc_row = new QHBoxLayout;
    svc_row->addWidget(mode_combo_);
    svc_row->addWidget(apply_mode_btn);
    svc_row->addWidget(save_btn_);

    bool envOk = false;
    int cur = qEnvironmentVariable("ROS_DOMAIN_ID").toInt(&envOk);
    if (!envOk || cur < 0 || cur > 100) cur = DEFAULT_DOMAIN;
    domain_edit_ = new QLineEdit(QString::number(cur));
    domain_edit_->setValidator(new QIntValidator(0, 100, this));
    domain_edit_->setFixedWidth(56);
    auto *dom_btn = new QPushButton("应用");
    auto *dom_row = new QHBoxLayout;
    dom_row->addWidget(new QLabel("DOMAIN_ID"));
    dom_row->addWidget(domain_edit_);
    dom_row->addWidget(dom_btn);
    dom_row->addStretch();
    dom_row->addWidget(mode_label_);

    vel_timer_ = new QTimer(this);
    vel_timer_->setInterval(100);
    connect(vel_timer_, &QTimer::timeout, this, [this] { publishTwist(v_, w_); });

    auto *pad = new QGridLayout;
    struct BtnDef { const char *txt; int r, c; double v, w; };
    // v/w 存方向系数，实际速度 = 输入框的线速度/角速度 * 系数
    const BtnDef defs[] = {
        {"↑",  0, 1,  1, 0},
        {"←",  1, 0,  0,  1},
        {"↓",  1, 1, -1, 0},
        {"→",  1, 2,  0, -1},
    };
    for (const auto &d : defs) {
        auto *b = new QPushButton(d.txt);
        b->setFixedSize(56, 44);
        b->setProperty("v", d.v);
        b->setProperty("w", d.w);
        b->installEventFilter(this);
        pad->addWidget(b, d.r, d.c);
    }
    // 轮盘靠左，右侧两行手动设置线速度/角速度
    lin_edit_ = new QLineEdit(QString::number(LIN_VEL));
    lin_edit_->setFixedWidth(64);
    auto *lin_btn = new QPushButton("应用");
    auto *lin_row = new QHBoxLayout;
    lin_row->addWidget(new QLabel("线速度"));
    lin_row->addWidget(lin_edit_);
    lin_row->addWidget(lin_btn);
    ang_edit_ = new QLineEdit(QString::number(ANG_VEL));
    ang_edit_->setFixedWidth(64);
    auto *ang_btn = new QPushButton("应用");
    auto *ang_row = new QHBoxLayout;
    ang_row->addWidget(new QLabel("角速度"));
    ang_row->addWidget(ang_edit_);
    ang_row->addWidget(ang_btn);
    auto *vel_col = new QVBoxLayout;
    vel_col->addLayout(lin_row);
    vel_col->addLayout(ang_row);
    vel_col->addStretch();
    auto *pad_row = new QHBoxLayout;
    pad_row->addLayout(pad);
    pad_row->addLayout(vel_col);
    pad_row->addStretch();
    auto *pose_btn = new QPushButton("当前位姿");
    auto *task_btn = new QPushButton("导览点(t)");
    auto *via_btn = new QPushButton("途经点(v)");
    auto *clear_btn = new QPushButton("清空点位");
    auto *wp_row = new QHBoxLayout;
    for (auto *b : {pose_btn, task_btn, via_btn, clear_btn}) {
        b->setMinimumHeight(34);
        wp_row->addWidget(b);
    }

    status_ = new QLabel("就绪");

    auto *root = new QVBoxLayout(this);
    root->addLayout(dom_row);
    root->addLayout(svc_row);
    root->addLayout(pad_row);
    root->addLayout(wp_row);
    root->addWidget(status_);

    connect(apply_mode_btn, &QPushButton::clicked, this, [this, mode_combo_] {
        setMode(mode_combo_->currentText() == "定位");
    });
    connect(save_btn_, &QPushButton::clicked, this, [this] { saveMap(); });
    connect(task_btn, &QPushButton::clicked, this, [this] { recordWaypoint("task"); });
    connect(via_btn, &QPushButton::clicked, this, [this] { recordWaypoint("via"); });
    connect(pose_btn, &QPushButton::clicked, this, &ChassisPanel::showPose);
    connect(clear_btn, &QPushButton::clicked, this, &ChassisPanel::clearWaypoints);
    connect(dom_btn, &QPushButton::clicked, this, [this] {
        bool ok = false;
        int id = domain_edit_->text().toInt(&ok);
        if (ok && id >= 0 && id <= 100) applyDomain(id);
    });
    connect(domain_edit_, &QLineEdit::returnPressed, dom_btn, &QPushButton::click);
    connect(lin_btn, &QPushButton::clicked, this, [this] {
        bool ok = false;
        const double v = lin_edit_->text().toDouble(&ok);
        if (ok && v > 0 && v <= 3.0) {
            lin_vel_ = v;
            status_->setText(QString("线速度已设为 %1 m/s").arg(v));
        } else {
            status_->setText("线速度输入无效，请输入 0~3 m/s");
        }
    });
    connect(ang_btn, &QPushButton::clicked, this, [this] {
        bool ok = false;
        const double w = ang_edit_->text().toDouble(&ok);
        if (ok && w > 0 && w <= 5.0) {
            ang_vel_ = w;
            status_->setText(QString("角速度已设为 %1 rad/s").arg(w));
        } else {
            status_->setText("角速度输入无效，请输入 0~5 rad/s");
        }
    });
    connect(lin_edit_, &QLineEdit::returnPressed, lin_btn, &QPushButton::click);
    connect(ang_edit_, &QLineEdit::returnPressed, ang_btn, &QPushButton::click);

    initRos(cur);
}

ChassisPanel::~ChassisPanel() { resetRos(); }

bool ChassisPanel::eventFilter(QObject *watched, QEvent *event) {
    auto *b = qobject_cast<QPushButton *>(watched);
    if (b && !b->property("v").isNull()) {
        if (event->type() == QEvent::MouseButtonPress) {
            v_ = lin_vel_ * b->property("v").toDouble();
            w_ = ang_vel_ * b->property("w").toDouble();
            status_->setText(v_ != 0 ? QString("线速度 %1 m/s").arg(v_)
                                     : QString("角速度 %1 rad/s").arg(w_));
            vel_timer_->start();
        } else if (event->type() == QEvent::MouseButtonRelease ||
                   event->type() == QEvent::Leave) {
            vel_timer_->stop();
            publishTwist(0, 0);
            status_->setText("停止");
        }
    }
    return QWidget::eventFilter(watched, event);
}

void ChassisPanel::publishTwist(double v, double w) {
    if (!cmd_pub_) return;
    geometry_msgs::msg::Twist t;
    t.linear.x = v;
    t.angular.z = w;
    cmd_pub_->publish(t);
}

void ChassisPanel::setMode(bool localization) {
    status_->setText(localization ? "切换定位模式..." : "切换建图模式...");
    if (!mode_cli_->wait_for_service(std::chrono::seconds(2))) {
        status_->setText(QString("服务不可用: %1").arg(SRV_SYSTEM_MODE));
        return;
    }
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = localization;
    mode_cli_->async_send_request(req,
        [this, localization](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture f) {
            bool ok = f.valid() && f.get()->success;
            QMetaObject::invokeMethod(status_, [this, localization, ok] {
                if (!ok) { status_->setText("模式切换失败"); return; }
                mode_label_->setText(localization ? "当前模式: 定位" : "当前模式: 建图");
                save_btn_->setEnabled(!localization); // 仅建图模式可保存地图
                status_->setText(localization ? "已切换定位模式" : "已切换建图模式");
            }, Qt::QueuedConnection);
        });
}

void ChassisPanel::saveMap() {
    status_->setText("保存地图...");
    if (!save_cli_->wait_for_service(std::chrono::seconds(2))) {
        status_->setText(QString("服务不可用: %1").arg(SRV_SAVE_MAP));
        return;
    }
    save_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
            bool ok = f.valid() && f.get()->success;
            QString msg = f.valid() ? QString::fromStdString(f.get()->message) : QString();
            QMetaObject::invokeMethod(status_, [this, ok, msg] {
                status_->setText(QString("%1%2").arg(ok ? "保存地图成功" : "保存地图失败")
                                                  .arg(msg.isEmpty() ? "" : ": " + msg));
            }, Qt::QueuedConnection);
        });
}

void ChassisPanel::recordWaypoint(const QString &type) {
    double x, y, theta;
    if (!currentPose(x, y, theta)) {
        status_->setText(QString("未收到位姿数据: %1").arg(POSE_TOPIC));
        return;
    }

    const QString path = waypointsFile();
    const int id = nextWaypointId(path);
    const bool existed = QFileInfo::exists(path);

    QFile f(path);
    if (!f.open(QIODevice::Append | QIODevice::Text)) {
        status_->setText(QString("无法写入点位文件: %1").arg(path));
        return;
    }
    if (!existed) {
        f.write("points:\n");
    } else if (f.size() > 0) {
        f.seek(f.size() - 1);
        char last = '\n';
        f.getChar(&last);
        f.seek(f.size());
        if (last != '\n') f.write("\n");
    }
    f.write(QString("- id: %1\n"
                    "  type: %2\n"
                    "  x: %3\n"
                    "  y: %4\n"
                    "  theta: %5\n")
                .arg(id).arg(type, fmt(x), fmt(y), fmt(theta))
                .toUtf8());
    f.close();

    status_->setText(QString("已保存点位 id=%1 type=%2: x=%3, y=%4, theta=%5 (%6)")
                         .arg(id)
                         .arg(type, fmt(x), fmt(y), fmt(theta), path));
}

void ChassisPanel::showPose() {
    double x, y, theta;
    if (!currentPose(x, y, theta)) {
        status_->setText(QString("未收到位姿数据: %1").arg(POSE_TOPIC));
        return;
    }
    status_->setText(QString("当前位姿: x=%1, y=%2, theta=%3")
                         .arg(fmt(x), fmt(y), fmt(theta)));
}

void ChassisPanel::clearWaypoints() {
    const QString path = waypointsFile();
    if (!QFileInfo::exists(path)) {
        status_->setText(QString("点位文件不存在，无需清除: %1").arg(path));
        return;
    }
    if (QFile::remove(path))
        status_->setText(QString("已清除点位文件: %1").arg(path));
    else
        status_->setText(QString("清除点位文件失败: %1").arg(path));
}

bool ChassisPanel::currentPose(double &x, double &y, double &theta) {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    if (!has_pose_) return false;
    const auto &p = last_pose_.pose.pose;
    x = p.position.x;
    y = p.position.y;
    theta = quaternionToTheta(p.orientation);
    return true;
}

void ChassisPanel::initRos(int domain) {
    has_pose_ = false;
    node_ = rclcpp::Node::make_shared("merman_chassis_panel");
    cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(CMD_VEL_TOPIC, 10);
    mode_cli_ = node_->create_client<std_srvs::srv::SetBool>(SRV_SYSTEM_MODE);
    save_cli_ = node_->create_client<std_srvs::srv::Trigger>(SRV_SAVE_MAP);
    pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        POSE_TOPIC, 10,
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            last_pose_ = *msg;
            has_pose_ = true;
        });
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this] { executor_->spin(); });
}

void ChassisPanel::resetRos() {
    if (executor_) executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    cmd_pub_.reset();
    mode_cli_.reset();
    save_cli_.reset();
    pose_sub_.reset();
    if (executor_ && node_) executor_->remove_node(node_);
    executor_.reset();
    node_.reset();
}

void ChassisPanel::applyDomain(int domain) {
    setenv("ROS_DOMAIN_ID", std::to_string(domain).c_str(), 1);
    resetRos();
    rclcpp::shutdown();
    rclcpp::init(0, nullptr);
    initRos(domain);

    for (const auto &script : DOMAIN_SCRIPTS)
        if (QFileInfo::exists(script))
            QProcess::execute("sed", {"-i",
                QString("s/^export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=%1/").arg(domain),
                script});
    for (const auto &unit : SYSTEMD_UNITS)
        QProcess::startDetached("sudo", {"systemctl", "restart", unit});

    status_->setText(QString("已切换至 DOMAIN %1").arg(domain));
}
