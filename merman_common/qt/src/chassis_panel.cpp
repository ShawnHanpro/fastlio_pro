#include "chassis_panel.h"

#include <QCoreApplication>
#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <cmath>
#include <cstdlib>

#include "map_widget.h"

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

// 点位文件按 "- id: N" 行分块；首个块之前的行(如 points: 头)保留在 header 里
struct WpBlock {
    int id;
    QStringList lines;
};

static bool readWaypointBlocks(const QString &path, QStringList &header,
                               QList<WpBlock> &blocks) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return false;
    static const QRegularExpression re("^\\s*-?\\s*id:\\s*(\\d+)");
    const auto lines = QString::fromUtf8(f.readAll()).split('\n');
    bool in_block = false;
    for (const auto &line : lines) {
        const auto m = re.match(line);
        if (m.hasMatch()) {
            blocks.append({m.captured(1).toInt(), {line}});
            in_block = true;
        } else if (in_block) {
            blocks.last().lines.append(line);
        } else {
            header.append(line);
        }
    }
    return true;
}

// 整体重写点位文件：每块首行是 id 行，重写为块结构里的 id；
// 块内其余行原样保留，机器人将来在点里追加的字段不会被改丢
static bool writeWaypointBlocks(const QString &path, const QStringList &header,
                                const QList<WpBlock> &blocks) {
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
        return false;
    static const QRegularExpression re("^(\\s*-?\\s*id:\\s*)\\d+");
    QStringList all = header;
    for (const auto &b : blocks) {
        for (int i = 0; i < b.lines.size(); ++i) {
            if (i == 0) {
                const auto m = re.match(b.lines[i]);
                all.append(m.captured(1) + QString::number(b.id));
            } else {
                all.append(b.lines[i]);
            }
        }
    }
    QString text = all.join('\n');
    if (!text.isEmpty() && !text.endsWith('\n')) text += '\n';
    f.write(text.toUtf8());
    return true;
}

ChassisPanel::ChassisPanel(QWidget *parent) : QWidget(parent) {
    setWindowTitle("Merman 底盘控制");

    // 文字按钮统一规格:最小高度 34、最小宽度按文字 sizeHint 兜底。
    // 左侧加地图后侧栏最大宽度有限,行空间不足时按钮会被压到文字显示不全,
    // 用 sizeHint 做最小宽度保证任何字体/缩放下文字都完整
    auto mkBtn = [](const QString &text) {
        auto *b = new QPushButton(text);
        b->setMinimumHeight(34);
        b->setMinimumWidth(b->sizeHint().width());
        return b;
    };

    auto *mode_combo_ = new QComboBox;
    mode_combo_->addItem("建图");
    mode_combo_->addItem("定位");
    // 机器人开机即定位模式,下拉默认选中定位,防止误选建图覆盖地图
    mode_combo_->setCurrentIndex(1);
    mode_combo_->setMinimumHeight(34);
    auto *apply_mode_btn = mkBtn("确认模式");
    save_btn_ = mkBtn("保存地图");
    save_btn_->setEnabled(false); // 只有处于建图模式才能保存,防止覆盖已有地图
    // 不依赖 /system_mode 确认:机器人默认定位模式,服务确认/自动检测成功后再改
    mode_label_ = new QLabel("当前模式: 定位(默认)");
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
    dom_btn_ = mkBtn("应用");
    auto *dom_row = new QHBoxLayout;
    dom_row->addWidget(new QLabel("DOMAIN_ID"));
    dom_row->addWidget(domain_edit_);
    dom_row->addWidget(dom_btn_);
    dom_row->addStretch();
    dom_row->addWidget(mode_label_);

    vel_timer_ = new QTimer(this);
    vel_timer_->setInterval(100);
    connect(vel_timer_, &QTimer::timeout, this, [this] { publishTwist(v_, w_); });

    // 服务响应轮询定时器:异步请求发出后轮询 future,
    // 响应到达或超时都从这里收尾,GUI 线程全程无阻塞
    mode_watchdog_ = new QTimer(this);
    mode_watchdog_->setInterval(100);
    connect(mode_watchdog_, &QTimer::timeout, this, [this] {
        if (!pending_mode_) { mode_watchdog_->stop(); return; }
        if (mode_future_ && mode_future_->valid() &&
            mode_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            mode_watchdog_->stop();
            const bool localization = pending_mode_localization_;
            const bool ok = mode_future_->get()->success;
            mode_future_.reset();
            pending_mode_ = false;
            if (!ok) { status_->setText("模式切换失败"); return; }
            mode_label_->setText(localization ? "当前模式: 定位" : "当前模式: 建图");
            save_btn_->setEnabled(!localization); // 仅建图模式可保存地图
            status_->setText(localization ? "已切换定位模式" : "已切换建图模式");
        } else if (mode_elapsed_.elapsed() > 5000) {
            mode_watchdog_->stop();
            pending_mode_ = false;
            if (mode_cli_ && mode_future_)
                mode_cli_->remove_pending_request(*mode_future_);
            mode_future_.reset();
            // 组播发现通了但单播路由错向、或服务端没拉起,都是这个表现
            status_->setText("模式切换超时:/system_mode 无响应(服务未拉起或网络不通)");
        }
    });
    save_watchdog_ = new QTimer(this);
    save_watchdog_->setInterval(200);
    connect(save_watchdog_, &QTimer::timeout, this, [this] {
        if (!pending_save_) { save_watchdog_->stop(); return; }
        if (save_future_ && save_future_->valid() &&
            save_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            save_watchdog_->stop();
            // future 只能 get() 一次,第二次会抛 std::future_error 导致进程 abort
            auto res = save_future_->get();
            const bool ok = res->success;
            const QString msg = QString::fromStdString(res->message);
            save_future_.reset();
            pending_save_ = false;
            status_->setText(QString("%1%2").arg(ok ? "保存地图成功" : "保存地图失败")
                                              .arg(msg.isEmpty() ? "" : ": " + msg));
        } else if (save_elapsed_.elapsed() > 10000) {
            save_watchdog_->stop();
            pending_save_ = false;
            if (save_cli_ && save_future_)
                save_cli_->remove_pending_request(*save_future_);
            save_future_.reset();
            status_->setText("保存地图超时:/map_save 无响应(服务未拉起或网络不通)");
        }
    });

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
    auto *lin_btn = mkBtn("应用");
    auto *lin_row = new QHBoxLayout;
    lin_row->addWidget(new QLabel("线速度"));
    lin_row->addWidget(lin_edit_);
    lin_row->addWidget(lin_btn);
    ang_edit_ = new QLineEdit(QString::number(ANG_VEL));
    ang_edit_->setFixedWidth(64);
    auto *ang_btn = mkBtn("应用");
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
    auto *pose_btn = mkBtn("查看当前位姿");
    auto *task_btn = mkBtn("导览点(t)");
    auto *via_btn = mkBtn("途经点(v)");
    auto *clear_btn = mkBtn("清空全部点位");
    auto *wp_row = new QHBoxLayout;
    for (auto *b : {pose_btn, task_btn, via_btn, clear_btn})
        wp_row->addWidget(b);

    // 修改/插入/删除点位：共用同一个 id 输入框
    // 修改=id 点位坐标换成当前位姿；插入=插到 id 位置、后面点位 +1；
    // 删除=删掉 id 点位、后面点位 -1。类型下拉框仅插入时使用。
    wp_id_edit_ = new QLineEdit;
    wp_id_edit_->setValidator(new QIntValidator(1, 99999, this));
    wp_id_edit_->setFixedWidth(64);
    wp_type_combo_ = new QComboBox;
    wp_type_combo_->addItem("导览点");
    wp_type_combo_->addItem("途经点");
    auto *wp_edit_btn = mkBtn("确认修改");
    auto *wp_insert_btn = mkBtn("插入");
    auto *wp_delete_btn = mkBtn("删除");
    auto *wp_edit_row = new QHBoxLayout;
    wp_edit_row->addWidget(new QLabel("输入点位"));
    wp_edit_row->addWidget(wp_id_edit_);
    wp_edit_row->addWidget(wp_type_combo_);
    wp_edit_row->addWidget(wp_edit_btn);
    wp_edit_row->addWidget(wp_insert_btn);
    wp_edit_row->addWidget(wp_delete_btn);
    wp_edit_row->addStretch();

    status_ = new QLabel("就绪");

    // 左侧地图,右侧原有控件列
    map_ = new MapWidget;
    // /map_2d 连续新帧 = FAST-LIO 在建图(它只在建图模式发布)。
    // /system_mode 服务没响应时也能据此把模式显示为建图并放开保存按钮
    connect(map_, &MapWidget::mappingActive, this, [this] {
        if (domain_switching_ || pending_mode_) return;
        mode_label_->setText("当前模式: 建图(自动检测)");
        save_btn_->setEnabled(true);
    });
    auto *side = new QVBoxLayout;
    side->addLayout(dom_row);
    side->addLayout(svc_row);
    side->addLayout(pad_row);
    side->addLayout(wp_row);
    side->addLayout(wp_edit_row);
    side->addWidget(status_);
    side->addStretch();
    auto *side_panel = new QWidget;
    side_panel->setLayout(side);
    // 保留"侧栏固定最大宽度"的设定(地图区吃掉其余宽度);从 540 起
    // 逐步放宽,以"输入点位"编辑行(标签+id 框+类型下拉+三个按钮)完整显示为准
    side_panel->setMaximumWidth(800);
    auto *root = new QHBoxLayout(this);
    root->addWidget(map_, 1);
    root->addWidget(side_panel);
    root->setContentsMargins(6, 6, 6, 6);

    connect(apply_mode_btn, &QPushButton::clicked, this, [this, mode_combo_] {
        setMode(mode_combo_->currentText() == "定位");
    });
    connect(save_btn_, &QPushButton::clicked, this, [this] { saveMap(); });
    connect(task_btn, &QPushButton::clicked, this, [this] { recordWaypoint("task"); });
    connect(via_btn, &QPushButton::clicked, this, [this] { recordWaypoint("via"); });
    connect(pose_btn, &QPushButton::clicked, this, &ChassisPanel::showPose);
    connect(clear_btn, &QPushButton::clicked, this, &ChassisPanel::clearWaypoints);
    connect(wp_edit_btn, &QPushButton::clicked, this, [this] {
        bool ok = false;
        const int id = wp_id_edit_->text().toInt(&ok);
        if (ok && id >= 1) editWaypoint(id);
    });
    connect(wp_id_edit_, &QLineEdit::returnPressed, wp_edit_btn, &QPushButton::click);
    connect(wp_insert_btn, &QPushButton::clicked, this, [this] {
        bool ok = false;
        const int id = wp_id_edit_->text().toInt(&ok);
        if (ok && id >= 1) insertWaypoint(id);
    });
    connect(wp_delete_btn, &QPushButton::clicked, this, [this] {
        bool ok = false;
        const int id = wp_id_edit_->text().toInt(&ok);
        if (ok && id >= 1) deleteWaypoint(id);
    });
    connect(dom_btn_, &QPushButton::clicked, this, [this] {
        bool ok = false;
        int id = domain_edit_->text().toInt(&ok);
        if (ok && id >= 0 && id <= 100) applyDomain(id);
    });
    connect(domain_edit_, &QLineEdit::returnPressed, dom_btn_, &QPushButton::click);
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

ChassisPanel::~ChassisPanel() {
    if (domain_thread_.joinable()) domain_thread_.join(); // 等在途的后台切换收尾
    resetRos();
}

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
    if (!cmd_pub_ || domain_switching_) return; // 切 DOMAIN 期间旧节点已废,不发布
    geometry_msgs::msg::Twist t;
    t.linear.x = v;
    t.angular.z = w;
    cmd_pub_->publish(t);
}

// 模式切换全程不阻塞 GUI:不再 wait_for_service(它会占住 GUI 线程最长 2 秒,
// 且发现到服务也不代表数据能送达),直接异步发请求,由 mode_watchdog_ 轮询收尾。
// 响应丢失时 5 秒明确报超时,而不是永远停在"切换中..."。
void ChassisPanel::setMode(bool localization) {
    if (domain_switching_) {
        status_->setText("正在切换 DOMAIN,请稍候");
        return;
    }
    if (!mode_cli_) return;
    if (pending_mode_) {
        status_->setText("上一个模式切换请求还在等待响应...");
        return;
    }
    pending_mode_ = true;
    pending_mode_localization_ = localization;
    status_->setText(localization ? "切换定位模式..." : "切换建图模式...");
    auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
    req->data = localization;
    mode_future_.emplace(mode_cli_->async_send_request(req)); // 立即返回,响应由 spin 线程回填
    mode_elapsed_.start();
    mode_watchdog_->start();
}

void ChassisPanel::saveMap() {
    if (domain_switching_) {
        status_->setText("正在切换 DOMAIN,请稍候");
        return;
    }
    if (!save_cli_) return;
    if (pending_save_) {
        status_->setText("上一次保存地图还在等待响应...");
        return;
    }
    pending_save_ = true;
    status_->setText("保存地图...");
    save_future_.emplace(save_cli_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>()));
    save_elapsed_.start();
    save_watchdog_->start();
}

void ChassisPanel::recordWaypoint(const QString &type) {
    if (domain_switching_) { status_->setText("正在切换 DOMAIN,请稍候"); return; }
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
    map_->reloadWaypoints(path);
}

void ChassisPanel::editWaypoint(int id) {
    if (domain_switching_) { status_->setText("正在切换 DOMAIN,请稍候"); return; }
    double x, y, theta;
    if (!currentPose(x, y, theta)) {
        status_->setText(QString("未收到位姿数据: %1").arg(POSE_TOPIC));
        return;
    }

    const QString path = waypointsFile();
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        status_->setText(QString("点位文件不存在或无法读取: %1").arg(path));
        return;
    }
    auto lines = QString::fromUtf8(f.readAll()).split('\n');
    f.close();

    // 块以 "- id: N" 开始，到下一个 "- id:" 或文件尾结束。
    // 逐行定点替换目标块内的 x/y/theta，块外与其余字段原样保留，
    // 这样将来机器人在点里追加别的字段也不会被改丢。
    static const QRegularExpression id_re("^\\s*-?\\s*id:\\s*(\\d+)");
    static const QRegularExpression xyz_re("^(\\s*)(x|y|theta|type)\\s*:");

    int block_start = -1;
    int block_end = lines.size();
    for (int i = 0; i < lines.size(); ++i) {
        const auto m = id_re.match(lines[i]);
        if (!m.hasMatch()) continue;
        if (m.captured(1).toInt() == id) {
            block_start = i;
        } else if (block_start >= 0) {
            block_end = i;
            break;
        }
    }
    if (block_start < 0) {
        status_->setText(QString("点位 id=%1 不存在于: %2").arg(id).arg(path));
        return;
    }

    const QString type = wp_type_combo_->currentText() == "途经点" ? "via" : "task";
    const QString values[4] = {type, fmt(x), fmt(y), fmt(theta)};
    bool replaced[4] = {false, false, false, false};
    for (int i = block_start + 1; i < block_end; ++i) {
        const auto m = xyz_re.match(lines[i]);
        if (!m.hasMatch()) continue;
        const QString &key = m.captured(2);
        const int idx = key == "type" ? 0 : key == "x" ? 1 : key == "y" ? 2 : 3;
        if (replaced[idx]) continue; // 块内重复字段，只改第一处
        lines[i] = m.captured(1) + key + ": " + values[idx];
        replaced[idx] = true;
    }

    // 目标块里缺失的字段补在块尾（跳过块尾空行），保证结构完整
    int insert_at = block_end;
    while (insert_at > block_start && lines[insert_at - 1].trimmed().isEmpty())
        --insert_at;
    static const char *keys[4] = {"type", "x", "y", "theta"};
    for (int k = 0; k < 4; ++k) {
        if (replaced[k]) continue;
        lines.insert(insert_at, QString("  %1: %2").arg(keys[k], values[k]));
        ++insert_at;
        ++block_end;
    }

    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        status_->setText(QString("无法写入点位文件: %1").arg(path));
        return;
    }
    f.write(lines.join('\n').toUtf8());
    f.close();

    status_->setText(QString("已修改点位 id=%1 type=%2: x=%3, y=%4, theta=%5 (%6)")
                         .arg(id)
                         .arg(type, fmt(x), fmt(y), fmt(theta), path));
    map_->reloadWaypoints(path);
}

void ChassisPanel::insertWaypoint(int id) {
    if (domain_switching_) { status_->setText("正在切换 DOMAIN,请稍候"); return; }
    double x, y, theta;
    if (!currentPose(x, y, theta)) {
        status_->setText(QString("未收到位姿数据: %1").arg(POSE_TOPIC));
        return;
    }

    const QString path = waypointsFile();
    QStringList header;
    QList<WpBlock> blocks;
    if (!readWaypointBlocks(path, header, blocks)) {
        // 文件不存在时允许插入 id=1 的首个点，与录点创建文件的行为一致
        if (!QFileInfo::exists(path)) {
            header = QStringList{"points:"};
            blocks.clear();
        } else {
            status_->setText(QString("点位文件无法读取: %1").arg(path));
            return;
        }
    }

    int max_id = 0;
    for (const auto &b : blocks) max_id = std::max(max_id, b.id);
    // 合法插入范围 1 ~ 最大id+1，插到末尾等价于追加；超出直接报错避免误插
    if (id > max_id + 1) {
        status_->setText(QString("插入位置超出范围，请输入 1 ~ %1").arg(max_id + 1));
        return;
    }

    // 原 id >= 插入位置的点依次 +1，再定位插入下标保持文件有序
    for (auto &b : blocks)
        if (b.id >= id) ++b.id;
    int insert_idx = blocks.size();
    for (int i = 0; i < blocks.size(); ++i) {
        if (blocks[i].id > id) { insert_idx = i; break; }
    }

    const QString type = wp_type_combo_->currentText() == "途经点" ? "via" : "task";
    WpBlock nb;
    nb.id = id;
    nb.lines = QStringList{
        QString("- id: %1").arg(id),
        QString("  type: %1").arg(type),
        QString("  x: %1").arg(fmt(x)),
        QString("  y: %1").arg(fmt(y)),
        QString("  theta: %1").arg(fmt(theta)),
    };
    blocks.insert(insert_idx, nb);

    if (!writeWaypointBlocks(path, header, blocks)) {
        status_->setText(QString("无法写入点位文件: %1").arg(path));
        return;
    }
    status_->setText(QString("已插入点位 id=%1 type=%2: x=%3, y=%4, theta=%5，"
                             "原 id≥%1 的点位已依次后移 (%6)")
                         .arg(id)
                         .arg(type, fmt(x), fmt(y), fmt(theta), path));
    map_->reloadWaypoints(path);
}

void ChassisPanel::deleteWaypoint(int id) {
    if (domain_switching_) { status_->setText("正在切换 DOMAIN,请稍候"); return; }
    const QString path = waypointsFile();
    QStringList header;
    QList<WpBlock> blocks;
    if (!readWaypointBlocks(path, header, blocks) || blocks.isEmpty()) {
        status_->setText(QString("点位文件不存在或没有点位: %1").arg(path));
        return;
    }

    int idx = -1;
    for (int i = 0; i < blocks.size(); ++i)
        if (blocks[i].id == id) { idx = i; break; }
    if (idx < 0) {
        status_->setText(QString("点位 id=%1 不存在于: %2").arg(id).arg(path));
        return;
    }

    blocks.removeAt(idx);
    // 删除点之后的点依次 -1
    for (auto &b : blocks)
        if (b.id > id) --b.id;

    if (!writeWaypointBlocks(path, header, blocks)) {
        status_->setText(QString("无法写入点位文件: %1").arg(path));
        return;
    }
    status_->setText(QString("已删除点位 id=%1，之后的点位已依次前移 (%2)")
                         .arg(id).arg(path));
    map_->reloadWaypoints(path);
}

void ChassisPanel::showPose() {
    if (domain_switching_) { status_->setText("正在切换 DOMAIN,请稍候"); return; }
    double x, y, theta;
    if (!currentPose(x, y, theta)) {
        status_->setText(QString("未收到位姿数据: %1").arg(POSE_TOPIC));
        return;
    }
    status_->setText(QString("当前位姿: x=%1, y=%2, theta=%3")
                         .arg(fmt(x), fmt(y), fmt(theta)));
}

void ChassisPanel::clearWaypoints() {
    if (domain_switching_) { status_->setText("正在切换 DOMAIN,请稍候"); return; }
    const QString path = waypointsFile();
    if (!QFileInfo::exists(path)) {
        status_->setText(QString("点位文件不存在，无需清除: %1").arg(path));
        return;
    }
    if (QFile::remove(path))
        status_->setText(QString("已清除点位文件: %1").arg(path));
    else
        status_->setText(QString("清除点位文件失败: %1").arg(path));
    map_->reloadWaypoints(path); // 文件已删,地图上的点位同步消失
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

    map_->attachNode(node_);
    map_->reloadWaypoints(waypointsFile());
}

void ChassisPanel::resetRos() {
    map_->detachNode(); // 先摘掉订阅,再停 spin 线程
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

// DOMAIN 切换是纯本地操作(设环境变量+重建本地 ROS 节点),不与机器人通信。
// 节点销毁/重建在坏网络上可能因 DDS 等超时卡好几秒,放后台线程执行,
// GUI 只负责置灰按钮和收尾;期间所有 ROS 操作被 domain_switching_ 挡住。
void ChassisPanel::applyDomain(int domain) {
    if (domain_switching_) return;
    domain_switching_ = true;
    // 旧节点马上销毁,在途服务请求的回调不会再来了,先解除占位
    pending_mode_ = pending_save_ = false;
    mode_watchdog_->stop();
    save_watchdog_->stop();
    if (mode_cli_ && mode_future_)
        mode_cli_->remove_pending_request(*mode_future_);
    if (save_cli_ && save_future_)
        save_cli_->remove_pending_request(*save_future_);
    mode_future_.reset();
    save_future_.reset();
    vel_timer_->stop(); // 切换期间不再遥控
    domain_edit_->setEnabled(false);
    dom_btn_->setEnabled(false);
    status_->setText(QString("正在切换至 DOMAIN %1...").arg(domain));
    setenv("ROS_DOMAIN_ID", std::to_string(domain).c_str(), 1);

    if (domain_thread_.joinable()) domain_thread_.join(); // 上次切换的线程已结束,回收
    domain_thread_ = std::thread([this, domain] {
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
        // GUI 线程只做收尾;面板销毁时该排队调用会被 Qt 自动丢弃,不悬空
        QMetaObject::invokeMethod(this, [this, domain] {
            domain_switching_ = false;
            domain_edit_->setEnabled(true);
            dom_btn_->setEnabled(true);
            status_->setText(QString("已切换至 DOMAIN %1").arg(domain));
        }, Qt::QueuedConnection);
    });
}
