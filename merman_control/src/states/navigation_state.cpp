#include "states/navigation_state.h"
#include "managers/navigation_manager.h"
#include "managers/slam_manager.h"
#include "state_machine/robot_event.h"

NavigationState::NavigationState(
    RobotRuntimePtr runtime)
    :
    RobotState(runtime)
{
}

const char* NavigationState::Name() const
{
    return "Navigation";
}

RobotStateName
NavigationState::StateID() const
{
    return RobotStateName::Navigation;
}

void NavigationState::OnEntry()
{
    {
        std::lock_guard<std::mutex> lock(
            runtime_->mutex);

        runtime_->mapping_running = false;
        runtime_->localization_running = true;
        runtime_->navigation_running = true;
        runtime_->task_running = false;
        runtime_->charging = false;
        runtime_->navigation_success = false;
        runtime_->navigation_failed = false;
    }

    if (runtime_->slam_manager_) {
        runtime_->slam_manager_->SetMode(SlamManager::Mode::Localization);
    }

    if (runtime_->navigation_manager_) {
        runtime_->navigation_manager_->Start();
    }
}

void NavigationState::OnExit()
{
    if (runtime_->navigation_manager_) {
        runtime_->navigation_manager_->Stop();
    }

    if (runtime_->slam_manager_) {
        runtime_->slam_manager_->SetMode(SlamManager::Mode::Idle);
    }

    std::lock_guard<std::mutex> lock(runtime_->mutex);

    runtime_->navigation_running = false;
    runtime_->localization_running = false;
}

RobotStateName NavigationState::Run()
{
    {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        if (runtime_->emergency_stop) {
            return RobotStateName::Error;
        }
    }

    if (!runtime_->navigation_manager_) {
        return RobotStateName::Error;
    }

    runtime_->navigation_manager_->Update();

    if (runtime_->navigation_manager_->HasError()) {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        runtime_->navigation_failed = true;
        return RobotStateName::Error;
    }

    if (runtime_->navigation_manager_->IsFinished()) {
        std::lock_guard<std::mutex> lock(runtime_->mutex);
        runtime_->navigation_success = true;
        return RobotStateName::Task;
    }

    return RobotStateName::None;
}

RobotStateName
NavigationState::HandleEvent(
    RobotEvent event)
{
    switch(event)
    {
        case RobotEvent::NavigationFinished:
        {
            std::lock_guard<std::mutex> lock(runtime_->mutex);
            runtime_->navigation_success = true;
            return RobotStateName::Task;
        }

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
