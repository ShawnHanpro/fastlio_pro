#include "managers/navigation_manager.h"

#include <iostream>

bool NavigationManager::Start()
{
    running_ = true;

    finished_ = false;

    return true;
}

bool NavigationManager::Stop()
{
    running_ = false;

    return true;
}

bool NavigationManager::NavigateToGoal(
    const Goal& goal)
{
    current_goal_ = goal;

    std::cout
        << "[Navigation] Goal:"
        << goal.x
        << ", "
        << goal.y
        << std::endl;

    running_ = true;

    finished_ = false;

    return true;
}

void NavigationManager::Update()
{
    if(!running_)
    {
        return;
    }

    /*
       以后：

       Nav2 Action Feedback

       Goal Status

       Recovery Status
    */
}

NavigationManager::Goal
NavigationManager::CurrentGoal() const
{
    return current_goal_;
}

bool NavigationManager::IsRunning() const
{
    return running_;
}

bool NavigationManager::IsFinished() const
{
    return finished_;
}

bool NavigationManager::HasError() const
{
    return error_;
}

std::string NavigationManager::Name() const
{
    return "NavigationManager";
}