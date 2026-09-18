#include <QApplication>
#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include "chassis_panel.h"

static void onSignal(int) {
    QMetaObject::invokeMethod(qApp, &QCoreApplication::quit, Qt::QueuedConnection);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    ChassisPanel panel;
    panel.resize(1300, 760); // 地图最小 400 + 侧栏最大 800 + 边距,给足两区空间
    panel.show();
    signal(SIGINT, onSignal);
    signal(SIGTERM, onSignal);
    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}
