#include "states/standby_state.h"
#include "managers/slam_manager.h"
#include "managers/standby_manager.h"
#include "state_machine/robot_event.h"

StandbyState::StandbyState(RobotRuntimePtr runtime) : RobotState(runtime) {}

const char* StandbyState::Name() const { return "Standby"; }

RobotStateName StandbyState::StateID() const { return RobotStateName::Standby; }

void StandbyState::OnEntry() {
    {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        runtime_->mapping_running = false;
        runtime_->localization_running = false;
        runtime_->navigation_running = false;
        runtime_->task_running = false;
        runtime_->charging = false;
        runtime_->navigation_success = false;
        runtime_->navigation_failed = false;
    }

    if (runtime_->slam_manager_) {
        runtime_->slam_manager_->SetMode(SlamManager::Mode::Idle);
    }

    if (runtime_->standby_manager_) {
        runtime_->standby_manager_->Start();
    }
}

void StandbyState::OnExit() {
    if (runtime_->standby_manager_) {
        runtime_->standby_manager_->Stop();
    }
}

RobotStateName StandbyState::HandleEvent(RobotEvent event) {
    switch (event) {
        case RobotEvent::StartMapping:
            return RobotStateName::Mapping;

        case RobotEvent::StartLocalization:
            return RobotStateName::Localization;

        case RobotEvent::StartNavigation:
            return RobotStateName::Navigation;

        case RobotEvent::StartTask:
            return RobotStateName::Task;

        case RobotEvent::LowBattery:
            return RobotStateName::Charging;

        case RobotEvent::Error:
            return RobotStateName::Error;

        default:
            return RobotStateName::None;
    }
}
