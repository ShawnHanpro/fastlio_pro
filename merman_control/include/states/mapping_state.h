#pragma once

#include "state_machine/robot_state.h"
#include "managers/slam_manager.h"

class MappingState : public RobotState {
public:
    explicit MappingState(RobotRuntimePtr runtime);

    const char* Name() const override;

    RobotStateName StateID() const override;

    void OnEntry() override;

    void OnExit() override;

    RobotStateName Run() override;

    RobotStateName HandleEvent(RobotEvent event) override;

private:
    // std::unique_ptr<MappingManager> mapping_manager_;
};
