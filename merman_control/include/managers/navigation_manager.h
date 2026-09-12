#pragma once

#include "managers/manager_base.h"

#include <atomic>

class NavigationManager : public ManagerBase
{
public:

    struct Goal
    {
        double x;
        double y;
        double yaw;
    };

public:

    bool Start() override;

    bool Stop() override;

    void Update() override;

    bool IsRunning() const override;

    bool IsFinished() const override;

    bool HasError() const override;

    std::string Name() const override;

    bool NavigateToGoal(
        const Goal& goal);

    Goal CurrentGoal() const;

private:

    Goal current_goal_;

    std::atomic<bool> running_{false};

    std::atomic<bool> finished_{false};

    std::atomic<bool> error_{false};
};