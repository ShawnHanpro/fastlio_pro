#include "managers/charging_manager.h"

#include <iostream>

bool ChargingManager::Start()
{
    std::cout
        << "[Charging] Start Docking"
        << std::endl;

    running_ = true;

    finished_ = false;

    return true;
}

bool ChargingManager::Stop()
{
    running_ = false;

    return true;
}

void ChargingManager::Update()
{
    if(!running_)
    {
        return;
    }

    /*
      后续：

      AprilTag

      UWB

      红外

      充电电流
    */
}

bool ChargingManager::IsRunning() const
{
    return running_;
}

bool ChargingManager::IsFinished() const
{
    return finished_;
}

bool ChargingManager::HasError() const
{
    return error_;
}

std::string ChargingManager::Name() const
{
    return "ChargingManager";
}