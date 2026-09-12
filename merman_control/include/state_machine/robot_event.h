#pragma once

#include <string>

enum class RobotEvent
{
    None = 0,

    StartMapping,
    StopMapping,

    StartLocalization,

    StartNavigation,

    StartTask,

    NavigationFinished,

    TaskFinished,

    LowBattery,

    ChargeFinished,

    Error,

    Reset
};

inline std::string get_robot_event_name(RobotEvent event) {
    switch (event) {
        case RobotEvent::None: return "None";
        case RobotEvent::StartMapping: return "StartMapping";
        case RobotEvent::StopMapping: return "StopMapping";
        case RobotEvent::StartLocalization: return "StartLocalization";
        case RobotEvent::StartNavigation: return "StartNavigation";
        case RobotEvent::StartTask: return "StartTask";
        case RobotEvent::NavigationFinished: return "NavigationFinished";
        case RobotEvent::TaskFinished: return "TaskFinished";
        case RobotEvent::LowBattery: return "LowBattery";
        case RobotEvent::ChargeFinished: return "ChargeFinished";
        case RobotEvent::Error: return "Error";
        case RobotEvent::Reset: return "Reset";
        default: return "Unknown";
    }
}