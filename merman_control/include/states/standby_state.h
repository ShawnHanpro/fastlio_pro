#pragma once

#include "state_machine/robot_state.h"

class StandbyState : public RobotState {
public:
    explicit StandbyState(RobotRuntimePtr runtime);

    const char* Name() const override;

    RobotStateName StateID() const override;

    void OnEntry() override;

    void OnExit() override;

    RobotStateName HandleEvent(RobotEvent event) override;
};
