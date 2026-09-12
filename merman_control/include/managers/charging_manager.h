#pragma once

#include "managers/manager_base.h"

#include <atomic>

class ChargingManager : public ManagerBase
{
public:

    bool Start() override;

    bool Stop() override;

    void Update() override;

    bool IsRunning() const override;

    bool IsFinished() const override;

    bool HasError() const override;

    std::string Name() const override;

private:

    std::atomic<bool> running_{false};

    std::atomic<bool> finished_{false};

    std::atomic<bool> error_{false};
};