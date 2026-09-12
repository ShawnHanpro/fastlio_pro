#pragma once

#include <mutex>
#include <queue>
#include <string>

#include "state_machine/state_store.h"

class StateMachine
{
public:

    StateMachine();

    ~StateMachine();

    void Initialize();

    void Shutdown();

    void Run();

    void PostEvent(
        RobotEvent event);

    void PostEvent(
        RobotEvent event,
        const std::string& task_name);

    RobotStateName CurrentState() const;

    std::string CurrentStateName() const;

private:

    void ChangeState(
        RobotStateName state);

    struct QueuedRobotEvent
    {
        RobotEvent event;
        std::string task_name;
    };

private:

    RobotRuntimePtr runtime_;

    StateStore state_store_;

    std::queue<QueuedRobotEvent> event_queue_;

    mutable std::mutex event_mutex_;

    bool initialized_ = false;
};
