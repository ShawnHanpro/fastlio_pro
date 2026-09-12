#include <thread>
#include <filesystem>
#include <iostream>

#include "lidar/lidar.h"
#include "merman_types.h"
#include "lidar/livox_mid360s.h"

#include "merman_types.h"
#include "battery/battery_driver.h"
#include "merman_hardware_interface/merman_hardware_interface.h"
#include "merman_control_interface/merman_control_interface.h"

MermanHardwareInterface::MermanHardwareInterface()
{
    battery_driver_ = std::make_unique<BatteryDriver>();
}

MermanHardwareInterface::~MermanHardwareInterface() = default;

void MermanHardwareInterface::StartLidarThread() {

    bool      lidar_prase_result = parse_lidar();
    LidarType lidar_type;
    if (get_lidar()->lidar_info_.company == Lidar::LidarInfo::Company::LIVOX) {
        lidar_type = LidarType::LIVOX;
    }
    // MermanControlInterface::GetInstance()->setLidarType(lidar_type);
    
    auto        lidar_ptr = get_lidar();
    std::thread lidar_thread(&Lidar::run, lidar_ptr);
    lidar_thread.detach();
}

void MermanHardwareInterface::StartBatteryThread()
{
    if (battery_driver_ && battery_driver_->isRunning()) {
        std::cout << "Battery thread already running." << std::endl;
        return;
    }

    try {
        std::filesystem::path config_path =
            std::filesystem::path(MERMAN_COMMON_CONFIG_DIR) / "battery.yaml";

        std::cout << "Battery config path: " << config_path << std::endl;

        BatteryConfig cfg = loadBatteryConfigFromYaml(config_path.string());

        battery_driver_ = std::make_unique<BatteryDriver>(cfg);

        battery_driver_->setDataCallback(
            [](const BatteryData& data) {
                // todo: send to communication
                // std::cout << BatteryDriver::toString(data) << std::endl;
                MermanControlInterface::GetInstance()->UpdateBattery(data);
            }
        );

        battery_driver_->setErrorCallback(
            [](const std::string& err) {
                std::cerr << "Battery read/update failed: " << err << std::endl;
            }
        );

        battery_driver_->start();

        std::cout << "Battery thread started." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "StartBatteryThread failed: " << e.what() << std::endl;
        battery_driver_.reset();
    }
}

// void MermanHardwareInterface::StartBatteryThread()
// {
//     try {
//         std::filesystem::path config_path =
//             std::filesystem::path(MERMAN_COMMON_CONFIG_DIR) / "battery.yaml";

//         BatteryConfig cfg = loadBatteryConfigFromYaml(config_path.string());


//         battery_driver_ = std::make_unique<BatteryDriver>(cfg);
//         battery_driver_->open();
//         battery_driver_->testPythonLedFullOn();
//         std::cout << "LED protocol test finished." << std::endl;

//     } catch (const std::exception& e) {
//         std::cerr << "StartBatteryThread failed: " << e.what() << std::endl;
//         battery_driver_.reset();
//     }
// }

void MermanHardwareInterface::StopBatteryThread()
{
    if (!battery_driver_) {
        return;
    }

    try {
        battery_driver_->stop();
        battery_driver_->close();
    } catch (const std::exception& e) {
        std::cerr << "StopBatteryThread failed: " << e.what() << std::endl;
    }

    battery_driver_.reset();

    std::cout << "Battery thread stopped." << std::endl;
}

bool MermanHardwareInterface::parse_lidar() {

    return true;
}

Lidar* MermanHardwareInterface::get_lidar() {
    return LivoxMid360s::GetInstance();
}