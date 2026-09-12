#pragma once

#include <iostream>
#include <thread>
#include <atomic>
#include <memory>
#include "state_machine/robot_runtime.h"
#include "managers/manager_base.h"

class SlamManager : public ManagerBase
{
public:

    enum class Mode
    {
        Idle,
        Mapping,
        Localization
    };

public:

    explicit SlamManager(
        RobotRuntimePtr runtime);

    ~SlamManager() override;

    bool Start() override;

    bool Stop() override;

    void Update() override;

    bool IsRunning() const override;

    bool IsFinished() const override;

    bool HasError() const override;

    std::string Name() const override;

    void SetMode(
        Mode mode);

    Mode CurrentMode() const;

private:

    void ThreadLoop();

private:

    RobotRuntimePtr runtime_;

    std::thread thread_;

    std::atomic<bool> running_{false};

    std::atomic<Mode> mode_{
        Mode::Idle
    };
};
