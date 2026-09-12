#pragma once

enum class RobotStateName
{
    None = 0,

    Standby,

    Mapping,

    Localization,

    Navigation,

    Task,

    Charging,

    Error
};

inline const char* get_robot_state_name(RobotStateName state)
{
    switch (state) {
        case RobotStateName::None: return "None";
        case RobotStateName::Standby: return "Standby";
        case RobotStateName::Mapping: return "Mapping";
        case RobotStateName::Localization: return "Localization";
        case RobotStateName::Navigation: return "Navigation";
        case RobotStateName::Task: return "Task";
        case RobotStateName::Charging: return "Charging";
        case RobotStateName::Error: return "Error";
        default: return "Unknown";
    }
}
