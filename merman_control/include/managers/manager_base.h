#pragma once

#include <thread>
#include <atomic>
#include <memory>
#include <string>
#include "state_machine/robot_runtime.h"

class ManagerBase
{
public:

    virtual ~ManagerBase() = default;

    virtual bool Start() = 0;

    virtual bool Stop() = 0;

    virtual void Update() = 0;

    virtual bool IsRunning() const = 0;

    virtual bool IsFinished() const = 0;

    virtual bool HasError() const = 0;

    virtual std::string Name() const = 0;
};
