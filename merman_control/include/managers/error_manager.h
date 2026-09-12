#pragma once

#include "managers/manager_base.h"

#include <string>

class ErrorManager : public ManagerBase
{
public:

    bool Start() override;

    bool Stop() override;

    void Update() override;

    bool IsRunning() const override;

    bool IsFinished() const override;

    bool HasError() const override;

    std::string Name() const override;

    void SetError(
        const std::string& msg);

    std::string ErrorMessage() const;

private:

    bool running_ = false;

    std::string error_msg_;
};