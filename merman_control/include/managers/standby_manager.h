#pragma once

#include "managers/manager_base.h"

class StandbyManager : public ManagerBase
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

    bool running_ = false;
};