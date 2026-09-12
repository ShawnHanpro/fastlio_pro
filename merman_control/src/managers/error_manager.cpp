#include "managers/error_manager.h"

bool ErrorManager::Start()
{
    running_ = true;

    return true;
}

bool ErrorManager::Stop()
{
    running_ = false;

    return true;
}

void ErrorManager::Update()
{
}

void ErrorManager::SetError(
    const std::string& msg)
{
    error_msg_ = msg;
}

std::string ErrorManager::ErrorMessage() const
{
    return error_msg_;
}

bool ErrorManager::IsRunning() const
{
    return running_;
}

bool ErrorManager::IsFinished() const
{
    return false;
}

bool ErrorManager::HasError() const
{
    return true;
}

std::string ErrorManager::Name() const
{
    return "ErrorManager";
}