#include "states/localization_state.h"

#include "managers/slam_manager.h"
#include "state_machine/robot_event.h"

LocalizationState::LocalizationState(RobotRuntimePtr runtime)
    : RobotState(runtime)
{
}

const char* LocalizationState::Name() const
{
    return "Localization";
}

RobotStateName LocalizationState::StateID() const
{
    return RobotStateName::Localization;
}

void LocalizationState::OnEntry()
{
    {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        runtime_->mapping_running = false;
        runtime_->localization_running = true;
        runtime_->navigation_running = false;
        runtime_->task_running = false;
        runtime_->charging = false;
    }

    if (runtime_->slam_manager_) {
        runtime_->slam_manager_->SetMode(SlamManager::Mode::Localization);
    }
}

void LocalizationState::OnExit()
{
    {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        runtime_->localization_running = false;
    }

    if (runtime_->slam_manager_) {
        runtime_->slam_manager_->SetMode(SlamManager::Mode::Idle);
    }
}

RobotStateName LocalizationState::Run()
{
    {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        if (runtime_->emergency_stop) {
            return RobotStateName::Error;
        }
    }

    if (runtime_->slam_manager_ && runtime_->slam_manager_->HasError()) {
        return RobotStateName::Error;
    }

    return RobotStateName::None;
}

RobotStateName LocalizationState::HandleEvent(RobotEvent event)
{
    switch (event) {
        case RobotEvent::StartNavigation:
            return RobotStateName::Navigation;

        case RobotEvent::StartTask:
            return RobotStateName::Task;

        case RobotEvent::LowBattery:
            return RobotStateName::Charging;

        case RobotEvent::Error:
            return RobotStateName::Error;

        case RobotEvent::Reset:
            return RobotStateName::Standby;

        default:
            return RobotStateName::None;
    }
}
