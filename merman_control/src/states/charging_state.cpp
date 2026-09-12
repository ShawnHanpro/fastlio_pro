#include "states/charging_state.h"
#include "managers/charging_manager.h"
#include "state_machine/robot_event.h"


ChargingState::ChargingState(
    RobotRuntimePtr runtime)
    :
    RobotState(runtime)
{
}

const char* ChargingState::Name() const
{
    return "Charging";
}

RobotStateName ChargingState::StateID() const
{
    return RobotStateName::Charging;
}

void ChargingState::OnEntry()
{
    {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        runtime_->mapping_running = false;
        runtime_->localization_running = false;
        runtime_->navigation_running = false;
        runtime_->task_running = false;
        runtime_->charging = true;
    }

    if (runtime_->charging_manager_) {
        runtime_->charging_manager_->Start();
    }
}

void ChargingState::OnExit()
{
    if (runtime_->charging_manager_) {
        runtime_->charging_manager_->Stop();
    }

    std::lock_guard<std::mutex> lock(runtime_->mutex);
    runtime_->charging = false;
}

RobotStateName ChargingState::Run()
{
    {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        if (runtime_->emergency_stop) {
            return RobotStateName::Error;
        }
    }

    if (!runtime_->charging_manager_) {
        return RobotStateName::Error;
    }

    runtime_->charging_manager_->Update();

    if (runtime_->charging_manager_->HasError()) {
        return RobotStateName::Error;
    }

    if (runtime_->charging_manager_->IsFinished()) {
        return RobotStateName::Standby;
    }

    return RobotStateName::None;
}

RobotStateName
ChargingState::HandleEvent(
    RobotEvent event)
{
    switch(event)
    {
        case RobotEvent::ChargeFinished:
            return RobotStateName::Standby;

        case RobotEvent::Error:
            return RobotStateName::Error;

        case RobotEvent::Reset:
            return RobotStateName::Standby;

        default:
            return RobotStateName::None;
    }
}
