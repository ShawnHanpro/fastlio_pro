/*
 * @Author: Shengyu Han hanshy2@shanghai-electric.com
 * @Date: 2026-07-14 19:06:11
 * @LastEditors: Shengyu Han hanshy2@shanghai-electric.com
 * @LastEditTime: 2026-07-17 14:45:37
 * @FilePath: /slam_nav/merman_communication/include/base/channel_base.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <string>
#include "merman_types.h"


class ChannelBase
{
public:
    virtual ~ChannelBase() = default;

    virtual bool init(const CommunicationConfig& config) = 0;
    virtual bool start() = 0;
    virtual void stop() = 0;

    virtual bool PublishRobotStatus(const RobotStatus& status) = 0;

    // 通用 JSON 发布接口，后续扩展很方便
    virtual bool publishJson(const std::string& topic_name, const std::string& json) = 0;

    virtual void setCommandCallback(CommandCallback callback) = 0;

    virtual bool set_waypoints(
        std::vector<NavigationWaypoint>& waypoints) = 0;

    virtual bool PublishCmdVel(double linear_x, double angular_z) = 0;

    virtual bool get_pose(Pose2D& pose) = 0;
    virtual bool PoseArrived(int32_t pose_id, bool play_audio) = 0;
    virtual bool CancelWaypointNavigation() = 0;
};
