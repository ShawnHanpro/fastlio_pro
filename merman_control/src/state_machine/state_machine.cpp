#include "state_machine/state_machine.h"

#include "states/standby_state.h"
#include "states/mapping_state.h"
#include "states/localization_state.h"
#include "states/navigation_state.h"
#include "states/task_state.h"
#include "states/charging_state.h"
#include "states/error_state.h"
#include "managers/slam_manager.h"
#include "managers/navigation_manager.h"
#include "managers/charging_manager.h"
#include "managers/standby_manager.h"
#include "managers/error_manager.h"
#include "state_machine/robot_event.h"

StateMachine::StateMachine() { 
    runtime_ = std::make_shared<RobotRuntime>();
    runtime_->slam_manager_ = std::make_shared<SlamManager>(runtime_); 
    runtime_->navigation_manager_ = std::make_shared<NavigationManager>();
    runtime_->charging_manager_ = std::make_shared<ChargingManager>();
    runtime_->standby_manager_ = std::make_shared<StandbyManager>();
    runtime_->error_manager_ = std::make_shared<ErrorManager>();
}

StateMachine::~StateMachine()
{
    Shutdown();
}

void StateMachine::Initialize()
{
    if (initialized_) {
        return;
    }

    std::cout << "init state machine" << std::endl;
    state_store_.Register(
        std::make_shared<StandbyState>(runtime_));

    state_store_.Register(
        std::make_shared<MappingState>(runtime_));

    state_store_.Register(
        std::make_shared<LocalizationState>(runtime_));

    state_store_.Register(
        std::make_shared<NavigationState>(runtime_));

    state_store_.Register(
        std::make_shared<TaskState>(runtime_));

    state_store_.Register(
        std::make_shared<ChargingState>(runtime_));

    state_store_.Register(
        std::make_shared<ErrorState>(runtime_));

    state_store_.ChangeState(
        RobotStateName::Standby);

    runtime_->slam_manager_->Start();

    initialized_ = true;
}

void StateMachine::Shutdown()
{
    if (!initialized_) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(event_mutex_);
        std::queue<QueuedRobotEvent> empty;
        event_queue_.swap(empty);
    }

    state_store_.Clear();

    if (runtime_->navigation_manager_) {
        runtime_->navigation_manager_->Stop();
    }

    if (runtime_->charging_manager_) {
        runtime_->charging_manager_->Stop();
    }

    if (runtime_->standby_manager_) {
        runtime_->standby_manager_->Stop();
    }

    if (runtime_->error_manager_) {
        runtime_->error_manager_->Stop();
    }

    if (runtime_->slam_manager_) {
        runtime_->slam_manager_->Stop();
    }

    initialized_ = false;
}

void StateMachine::PostEvent(RobotEvent event) {
    PostEvent(event, std::string());
}

void StateMachine::PostEvent(RobotEvent event, const std::string& task_name) {
    std::lock_guard<std::mutex> lock(event_mutex_);
    std::cout << "Posting event: " << get_robot_event_name(event) << std::endl;
    event_queue_.push(QueuedRobotEvent{event, task_name});
}

RobotStateName StateMachine::CurrentState() const {
    return state_store_.CurrentState();
}

std::string StateMachine::CurrentStateName() const {
    return get_robot_state_name(CurrentState());
}

void StateMachine::ChangeState(RobotStateName state) {
    state_store_.ChangeState(state);
}

void StateMachine::Run() {
    QueuedRobotEvent queued_event{RobotEvent::None, std::string()};

    {
        std::lock_guard<std::mutex> lock(event_mutex_);
        if (!event_queue_.empty()) {
            queued_event = event_queue_.front();
            std::cout << "Processing event: "
                      << get_robot_event_name(queued_event.event)
                      << std::endl;
            event_queue_.pop();
        }
    }

    RobotState* state = state_store_.Current();

    if (!state) {
        return;
    }

    if (queued_event.event != RobotEvent::None) {
        if (queued_event.event == RobotEvent::StartTask) {
            std::lock_guard<std::mutex> lock(runtime_->mutex);
            runtime_->pending_task = queued_event.task_name;
        }

        RobotStateName next = state->HandleEvent(queued_event.event);

        if (next != RobotStateName::None) {
            ChangeState(next);
            return;
        }

        if (queued_event.event == RobotEvent::StartTask) {
            std::lock_guard<std::mutex> lock(runtime_->mutex);
            runtime_->pending_task.clear();
        }
    }

    RobotStateName next = state->Run();

    if (next != RobotStateName::None) {
        ChangeState(next);
    }
}
