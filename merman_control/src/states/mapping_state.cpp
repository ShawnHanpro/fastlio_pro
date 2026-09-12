#include "states/mapping_state.h"
#include "state_machine/robot_event.h"

MappingState::MappingState(RobotRuntimePtr runtime)
    : RobotState(runtime) {}

const char* MappingState::Name() const { return "Mapping"; }

RobotStateName MappingState::StateID() const { return RobotStateName::Mapping; }

void MappingState::OnEntry() {
    {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        runtime_->mapping_running = true;
        runtime_->localization_running = false;
        runtime_->navigation_running = false;
        runtime_->task_running = false;
        runtime_->charging = false;
    }

    std::cout << "MappingState::OnEntry: Start Mapping" << std::endl;

    if (runtime_->slam_manager_) {
        runtime_->slam_manager_->SetMode(
            SlamManager::Mode::Mapping);
    }

    // navigation_manager_->
    //     EnableNavigation();
}

void MappingState::OnExit() {
    {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        runtime_->mapping_running = false;
    }

    if (runtime_->slam_manager_) {
        runtime_->slam_manager_->SetMode(
            SlamManager::Mode::Idle);
    }
}

RobotStateName MappingState::Run() {
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

RobotStateName MappingState::HandleEvent(RobotEvent event) {
    switch (event) {
        case RobotEvent::StopMapping:
            return RobotStateName::Standby;

        case RobotEvent::LowBattery:
            return RobotStateName::Charging;

        case RobotEvent::Error:
            return RobotStateName::Error;

        default:
            return RobotStateName::None;
    }
}
