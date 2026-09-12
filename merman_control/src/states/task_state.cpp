#include "states/task_state.h"

#include "state_machine/robot_event.h"

TaskState::TaskState(RobotRuntimePtr runtime)
    : RobotState(runtime)
{
}

const char* TaskState::Name() const
{
    return "Task";
}

RobotStateName TaskState::StateID() const
{
    return RobotStateName::Task;
}

void TaskState::OnEntry()
{
    std::lock_guard<std::mutex> lock(runtime_->mutex);

    runtime_->mapping_running = false;
    runtime_->localization_running = false;
    runtime_->navigation_running = false;
    runtime_->task_running = true;
    runtime_->charging = false;

    if (!runtime_->pending_task.empty()) {
        runtime_->current_task = runtime_->pending_task;
        runtime_->pending_task.clear();
    }
}

void TaskState::OnExit()
{
    std::lock_guard<std::mutex> lock(runtime_->mutex);

    runtime_->task_running = false;
}

RobotStateName TaskState::Run()
{
    std::lock_guard<std::mutex> lock(runtime_->mutex);

    if (runtime_->emergency_stop) {
        return RobotStateName::Error;
    }

    return RobotStateName::None;
}

RobotStateName TaskState::HandleEvent(RobotEvent event)
{
    switch (event) {
        case RobotEvent::TaskFinished:
        {
            std::lock_guard<std::mutex> lock(runtime_->mutex);
            runtime_->current_task.clear();
            return RobotStateName::Standby;
        }

        case RobotEvent::StartNavigation:
            return RobotStateName::Navigation;

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
