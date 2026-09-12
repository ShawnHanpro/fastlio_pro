#include "managers/slam_manager.h"

#include <chrono>

SlamManager::SlamManager(RobotRuntimePtr runtime) : runtime_(runtime) {}

SlamManager::~SlamManager()
{
    Stop();
}

bool SlamManager::Start() {
    if (running_) {
        return true;
    }

    if (thread_.joinable()) {
        thread_.join();
    }

    std::cout << "start slam manager" << std::endl;
    running_ = true;

    thread_ = std::thread(&SlamManager::ThreadLoop, this);

    return true;
}

bool SlamManager::Stop() {
    running_ = false;

    if (thread_.joinable()) {
        thread_.join();
    }

    mode_ = Mode::Idle;

    return true;
}

void SlamManager::SetMode(Mode mode) { mode_ = mode; }

SlamManager::Mode SlamManager::CurrentMode() const { return mode_; }

void SlamManager::ThreadLoop() {
    Mode last_mode = Mode::Idle;

    while (running_) {
        const Mode current_mode = mode_;

        if (current_mode != last_mode) {
            switch (current_mode) {
                case Mode::Mapping:
                    std::cout << "slam manager mode: mapping" << std::endl;
                    break;

                case Mode::Localization:
                    std::cout << "slam manager mode: localization" << std::endl;
                    break;

                default:
                    std::cout << "slam manager mode: idle" << std::endl;
                    break;
            }

            last_mode = current_mode;
        }

        switch (current_mode) {
            case Mode::Mapping:
                /*
                RTABMap Mapping
                */

                break;

            case Mode::Localization:

                /*
                RTABMap Localization
                */

                break;

            default:

                break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void SlamManager::Update() {}

bool SlamManager::IsRunning() const { return running_; }

bool SlamManager::IsFinished() const { return false; }

bool SlamManager::HasError() const { return false; }

std::string SlamManager::Name() const { return "SlamManager"; }
