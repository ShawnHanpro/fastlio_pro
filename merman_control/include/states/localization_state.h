#pragma once

#include "state_machine/robot_state.h"

class LocalizationState : public RobotState
{
public:
    explicit LocalizationState(RobotRuntimePtr runtime);

    const char* Name() const override;

    RobotStateName StateID() const override;

    void OnEntry() override;

    void OnExit() override;

    RobotStateName Run() override;

    RobotStateName HandleEvent(RobotEvent event) override;
};
