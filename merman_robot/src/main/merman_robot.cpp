#include <strings.h>

#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

#include "main/boot_init.h"
#include "merman_control_interface/merman_control_interface.h"
#include "merman_hardware_interface/merman_hardware_interface.h"
#include "merman_logger/merman_logger.hpp"

std::string merman_robot_version = "v0.0.00";
std::string merman_control = "v0.0.00";
std::string merman_slam_version = "v0.0.00";
std::string merman_navigation_version = "v0.0.00";
std::string merman_communication_version = "v0.0.00";
std::string merman_logger_verison = "v0.0.0";

static volatile std::sig_atomic_t g_exit_requested = 0;

void SignalHandler(int signum) { g_exit_requested = 1; }

std::string get_robot_version() { return "v26.07.08"; }

void ShowVersionLogo() {
    std::cout << R"(=======================================  )" << std::endl;
    std::cout << R"(==      _         _                  ==  )" << std::endl;
    std::cout << R"(==     /  \      /  \                ==  )" << std::endl;
    std::cout << R"(==    /    \    /    \               ==  )" << std::endl;
    std::cout << R"(==   /  /\  \  /  /\  \              ==  )" << std::endl;
    std::cout << R"(==  /  /  \  \/  /  \  \             ==  )" << std::endl;
    std::cout << R"(== /__/    \ __ /    \ _\erman robot ==  )" << std::endl;
    std::cout << R"(=======================================  )" << std::endl;


    auto merman_control_ptr = MermanControlInterface::GetInstance();

    merman_robot_version = get_robot_version();
    merman_control = MermanControlInterface::get_control_version();
    merman_navigation_version =
        MermanControlInterface::get_navigation_version();
    merman_communication_version =
        merman_control_ptr->get_communication_version();
    merman_logger_verison = MermanLogger::get_logger_version();


    std::cout << "merman_robot_version:         " << merman_robot_version
              << std::endl;
    std::cout << "merman_control:               " << merman_control
              << std::endl;
    std::cout << "merman_communication_version: "
              << merman_communication_version << std::endl;


    std::cout << "merman_logger_verison: "        << merman_logger_verison << std::endl;
}

void StartThread() {
    auto merman_control_ptr = MermanControlInterface::GetInstance();
    auto merman_hardware_ptr = MermanHardwareInterface::GetInstance();

    // merman_hardware_ptr->StartBatteryThread();

    // 激光雷达线程
    // merman_hardware_ptr->StartLidarThread();

    merman_control_ptr->StartCommunicationThread();
}

void StopThread() {
    auto merman_control_ptr = MermanControlInterface::GetInstance();
    auto merman_hardware_ptr = MermanHardwareInterface::GetInstance();

    // merman_hardware_ptr->StopBatteryThread();
    merman_control_ptr->StopCommunicationThread();
}

int main(int argc, char *argv[]) {
    // 解析 --debug 参数并初始化日志管理器
    bool debug_mode = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--debug" && i + 1 < argc) {
            debug_mode = (std::string(argv[i + 1]) == "true");
        }
    }
    MermanLogger::GetInstance()->Initialize("../logs", debug_mode);

    // 注册 Ctrl+C / kill 信号
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    if (argc >= 2) {
        std::string arg_str = argv[1];

        if (0 == strncasecmp(arg_str.c_str(), "-v", 2)) {
            ShowVersionLogo();
        } else {
            std::cout << "Please using \"./merman_robot -v\" for version!"
                      << std::endl;
        }

        return 0;
    }

    ShowVersionLogo();

    BootInit sys_boot;
    sys_boot.SystemInit();

    StartThread();

    while (!g_exit_requested && !sys_boot.get_exit_thread()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << "[merman_robot] exit requested, stopping modules..."
              << std::endl;

    auto merman_control_ptr = MermanControlInterface::GetInstance();

    // 建议你在 MermanControlInterface 里提供这个接口
    // merman_control_ptr->StopAllThreads();

    StopThread();

    std::cout << "[merman_robot] exited." << std::endl;

    return 0;
}