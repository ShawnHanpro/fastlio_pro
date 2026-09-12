/*
 * @Author: Shengyu Han hanshy2@shanghai-electric.com
 * @Date: 2026-07-14 19:06:11
 * @LastEditors: Shengyu Han hanshy2@shanghai-electric.com
 * @LastEditTime: 2026-07-17 13:50:00
 * @FilePath: /slam_nav/merman_communication/include/manager/manager.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <memory>
#include <vector>
#include "merman_types.h"
#include "base/channel_base.h"
#include "merman_communication_interface/merman_communication_interface.h"

class Manager
{
public:
    explicit Manager(MermanCommunicationInterface& communication);
    ~Manager();

    bool init(const CommunicationConfig& config);
    bool start();
    void stop();

    void UpdateBattery(const BatteryData &battery);

    bool publishRobotStatus(const RobotStatus& status);
    bool publishJson(const std::string& topic_name, const std::string& json);

    void setCommandCallback(CommandCallback callback);

    bool set_nav2_waypoints(std::vector<NavigationWaypoint>& waypoints);

    bool PublishCmdVel(double linear_x, double angular_z);

    bool get_pose(Pose2D& pose);
    bool PoseArrived(int32_t pose_id, bool play_audio);

    bool CancelWaypointNavigation();

private:
    MermanCommunicationInterface& communication_;
    CommunicationConfig config_;
    std::vector<std::unique_ptr<ChannelBase>> channels_;

    BatteryData battery_;
};