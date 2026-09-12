#include "managers/standby_manager.h"

bool StandbyManager::Start()
{
    running_ = true;

    return true;
}

bool StandbyManager::Stop()
{
    running_ = false;

    return true;
}

void StandbyManager::Update()
{
}

bool StandbyManager::IsRunning() const
{
    return running_;
}

bool StandbyManager::IsFinished() const
{
    return false;
}

bool StandbyManager::HasError() const
{
    return false;
}

std::string StandbyManager::Name() const
{
    return "StandbyManager";
}