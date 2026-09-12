#pragma once

#include <memory>
#include <mutex>
#include <unordered_map>

#include "state_machine/robot_state.h"

class StateStore {
public:
    void Register(const std::shared_ptr<RobotState>& state);

    void ChangeState(RobotStateName state);

    void Clear();

    RobotState* Current();

    RobotStateName CurrentState() const;

private:
    std::unordered_map<RobotStateName, std::shared_ptr<RobotState> > states_;

    RobotState* current_ = nullptr;

    mutable std::mutex mutex_;
};
