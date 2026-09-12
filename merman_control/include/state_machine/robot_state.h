#pragma once

#include <memory>

// #include "state_machine/robot_event.h"
#include "state_machine/robot_runtime.h"
#include "state_machine/robot_state_name.h"

enum class RobotEvent;

class RobotState {
public:
    explicit RobotState(RobotRuntimePtr runtime)
        : runtime_(std::move(runtime)) {}

    virtual ~RobotState() = default;

    virtual const char* Name() const = 0;

    virtual RobotStateName StateID() const = 0;

    virtual void OnEntry() {}

    virtual void OnExit() {}

    virtual RobotStateName Run() { return RobotStateName::None; }

    virtual RobotStateName HandleEvent(RobotEvent) {
        return RobotStateName::None;
    }

protected:
    RobotRuntimePtr runtime_;
};
