#include "state_machine/state_store.h"

void StateStore::Register(const std::shared_ptr<RobotState>& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    states_[state->StateID()] = state;
}

void StateStore::ChangeState(RobotStateName state) {
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = states_.find(state);

    if (it == states_.end()) {
        return;
    }

    if (current_ == it->second.get()) {
        return;
    }

    if (current_) {
        current_->OnExit();
    }

    current_ = it->second.get();

    current_->OnEntry();
}

void StateStore::Clear()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (current_) {
        current_->OnExit();
    }

    current_ = nullptr;
}

RobotState* StateStore::Current()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return current_;
}

RobotStateName StateStore::CurrentState() const {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!current_) {
        return RobotStateName::None;
    }

    return current_->StateID();
}
