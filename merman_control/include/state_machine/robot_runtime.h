#pragma once

#include <mutex>
#include <memory>
#include <string>

class ChargingManager;
class ErrorManager;
class NavigationManager;
class SlamManager;
class StandbyManager;

struct RobotRuntime
{
    double battery_percent = 100.0;

    bool mapping_running = false;

    bool localization_running = false;

    bool navigation_running = false;

    bool task_running = false;

    bool charging = false;

    bool emergency_stop = false;

    bool navigation_success = false;

    bool navigation_failed = false;

    std::string current_task;

    std::string pending_task;

    mutable std::mutex mutex;

    std::shared_ptr<SlamManager> slam_manager_;

    std::shared_ptr<NavigationManager> navigation_manager_;

    std::shared_ptr<ChargingManager> charging_manager_;

    std::shared_ptr<StandbyManager> standby_manager_;

    std::shared_ptr<ErrorManager> error_manager_;

    // std::shared_ptr<DockingManager> docking_manager;
};

using RobotRuntimePtr = std::shared_ptr<RobotRuntime>;
