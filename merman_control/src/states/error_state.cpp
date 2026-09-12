#include "states/error_state.h"
#include "managers/charging_manager.h"
#include "managers/error_manager.h"
#include "managers/navigation_manager.h"
#include "managers/slam_manager.h"
#include "state_machine/robot_event.h"

ErrorState::ErrorState(
    RobotRuntimePtr runtime)
    :
    RobotState(runtime)
{
}

const char* ErrorState::Name() const
{
    return "Error";
}

RobotStateName ErrorState::StateID() const
{
    return RobotStateName::Error;
}

void ErrorState::OnEntry()
{
    if (runtime_->navigation_manager_) {
        runtime_->navigation_manager_->Stop();
    }

    if (runtime_->charging_manager_) {
        runtime_->charging_manager_->Stop();
    }

    if (runtime_->slam_manager_) {
        runtime_->slam_manager_->SetMode(SlamManager::Mode::Idle);
    }

    {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        runtime_->emergency_stop = true;
        runtime_->mapping_running = false;
        runtime_->localization_running = false;
        runtime_->navigation_running = false;
        runtime_->task_running = false;
        runtime_->charging = false;
        runtime_->navigation_failed = true;
    }

    if (runtime_->error_manager_) {
        runtime_->error_manager_->Start();
    }
}

void ErrorState::OnExit()
{
    if (runtime_->error_manager_) {
        runtime_->error_manager_->Stop();
    }
}

RobotStateName
ErrorState::HandleEvent(
    RobotEvent event)
{
    if(event == RobotEvent::Reset)
    {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        runtime_->emergency_stop = false;
        runtime_->navigation_failed = false;

        return RobotStateName::Standby;
    }

    return RobotStateName::None;
}
