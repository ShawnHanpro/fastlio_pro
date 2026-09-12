#pragma once

#include <memory>

class BatteryDriver;
class Lidar;

class MermanHardwareInterface {
private:
    MermanHardwareInterface();

public:
    /**
     * @brief 单例模式
     */
    static MermanHardwareInterface* GetInstance() {
        static MermanHardwareInterface instance;
        return &instance;
    }

    ~MermanHardwareInterface();

    MermanHardwareInterface(const MermanHardwareInterface&) = delete;
    MermanHardwareInterface& operator=(const MermanHardwareInterface&) = delete;

    void StartLidarThread();
    void StartBatteryThread();

    void StopBatteryThread();

private:
    bool parse_lidar();
    Lidar* get_lidar();

private:
    std::unique_ptr<BatteryDriver> battery_driver_;
};