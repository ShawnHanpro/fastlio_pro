#include "manager/manager.h"
#include "channels/ros2_channel.h"

Manager::Manager(MermanCommunicationInterface& communication)
    : communication_(communication) {

    }

Manager::~Manager()
{
    stop();
}

bool Manager::init(const CommunicationConfig& config)
{
    config_ = config;
    channels_.clear();

    if (config_.enable_ros2_topic) {
        auto channel = std::make_unique<Ros2Channel>(communication_);
        if (!channel->init(config_)) {
            return false;
        }
        channels_.push_back(std::move(channel));
    }

    // 未来扩展：
    //
    // if (config_.enable_mqtt) {
    //     auto channel = std::make_unique<Mqttchannel>();
    //     channel->init(config_);
    //     channels_.push_back(std::move(channel));
    // }
    //
    // if (config_.enable_tcp) {
    //     auto channel = std::make_unique<Tcpchannel>();
    //     channel->init(config_);
    //     channels_.push_back(std::move(channel));
    // }
    //
    // if (config_.enable_sbus) {
    //     auto channel = std::make_unique<Sbuschannel>();
    //     channel->init(config_);
    //     channels_.push_back(std::move(channel));
    // }

    return true;
}

bool Manager::start()
{
    for (auto& channel : channels_) {
        if (!channel->start()) {
            return false;
        }
    }

    return true;
}

void Manager::stop()
{
    for (auto& channel : channels_) {
        if (channel) {
            channel->stop();
        }
    }
}

void Manager::UpdateBattery(const BatteryData &battery) {
    battery_ = battery;
}

bool Manager::publishRobotStatus(const RobotStatus& status)
{
    bool ok = true;

    for (auto& channel : channels_) {
        ok = channel->PublishRobotStatus(status) && ok;
    }

    return ok;
}

bool Manager::publishJson(const std::string& topic_name, const std::string& json)
{
    bool ok = true;

    for (auto& channel : channels_) {
        ok = channel->publishJson(topic_name, json) && ok;
    }

    return ok;
}

void Manager::setCommandCallback(CommandCallback callback)
{
    for (auto& channel : channels_) {
        channel->setCommandCallback(callback);
    }
}

bool Manager::set_nav2_waypoints(std::vector<NavigationWaypoint>& waypoints){
    bool ok = true;

    for (auto& channel : channels_) {
        ok = channel->set_waypoints(waypoints);
    }

    return ok;
}

bool Manager::PublishCmdVel(double linear_x, double angular_z) {
    bool ok = true;

    for (auto& channel : channels_) {
        ok = channel->PublishCmdVel(linear_x, angular_z);
    }

    return ok;
}

bool Manager::get_pose(Pose2D& pose) {
    bool ok = true;

    for (auto& channel : channels_) {
        ok = channel->get_pose(pose);
    }

    return ok;
}

bool Manager::PoseArrived(int32_t pose_id, bool play_audio) {
    bool ok = true;

    for (auto& channel : channels_) {
        ok = channel->PoseArrived(pose_id, play_audio);
    }

    return ok;
}

bool Manager::CancelWaypointNavigation() {
    bool ok = true;

    for (auto& channel : channels_) {
        ok = channel->CancelWaypointNavigation();
    }

    return ok;
}

