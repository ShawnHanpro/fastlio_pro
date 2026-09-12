#include "config_managers/communication_config.h"

#include <iostream>
#include <yaml-cpp/yaml.h>

bool LoadCommunicationConfig(
    const std::string& yaml_path,
    CommunicationConfig& config)
{
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);

        if (!root["communication"]) {
            std::cerr
                << "communication config not found in yaml: "
                << yaml_path
                << std::endl;
            return false;
        }

        const YAML::Node communication = root["communication"];

        config.robot_sn =
            communication["robot_sn"].as<std::string>(
                config.robot_sn);

        // enable 配置
        if (communication["enable"]) {
            const YAML::Node enable = communication["enable"];

            config.enable_ros2_topic =
                enable["ros2_topic"].as<bool>(
                    config.enable_ros2_topic);

            config.enable_mqtt =
                enable["mqtt"].as<bool>(
                    config.enable_mqtt);

            config.enable_tcp =
                enable["tcp"].as<bool>(
                    config.enable_tcp);

            config.enable_sbus =
                enable["sbus"].as<bool>(
                    config.enable_sbus);
        }

        // ROS2 配置
        if (communication["ros2"]) {
            const YAML::Node ros2 = communication["ros2"];

            config.ros_node_name =
                ros2["node_name"].as<std::string>(
                    config.ros_node_name);

            config.robot_status_topic =
                ros2["robot_status_topic"].as<std::string>(
                    config.robot_status_topic);

            config.pub_cmd_vel_topic =
                ros2["pub_cmd_vel_topic"].as<std::string>(
                    config.pub_cmd_vel_topic);

            config.pub_audio_play_topic =
                ros2["pub_audio_play_topic"].as<std::string>(
                    config.pub_audio_play_topic);

            config.local_cmd_topic =
                ros2["local_cmd_topic"].as<std::string>(
                    config.local_cmd_topic);

            config.cloud_cmd_topic =
                ros2["cloud_cmd_topic"].as<std::string>(
                    config.cloud_cmd_topic);

            config.sub_pose_topic =
                ros2["sub_pose_topic"].as<std::string>(
                    config.sub_pose_topic);

            config.sub_cmd_vel_topic =
                ros2["sub_cmd_vel_topic"].as<std::string>(
                    config.sub_cmd_vel_topic);

            config.sub_audio_done_topic =
                ros2["sub_audio_done_topic"].as<std::string>(
                    config.sub_audio_done_topic);
                    
            config.sub_pause_task_topic =
                ros2["sub_pause_task_topic"].as<std::string>(
                    config.sub_pause_task_topic);

            config.start_task_service =
                ros2["start_task_service"].as<std::string>(
                    config.start_task_service);

            config.stop_task_service =
                ros2["stop_task_service"].as<std::string>(
                    config.stop_task_service);

            config.pose_arrived_service =
                ros2["pose_arrived_service"].as<std::string>(
                    config.pose_arrived_service);

            config.follow_waypoints_action =
                ros2["follow_waypoints_action"].as<std::string>(
                    config.follow_waypoints_action);

            config.navigation_frame =
                ros2["navigation_frame"].as<std::string>(
                    config.navigation_frame);

            config.follow_waypoints_wait_timeout_sec =
                ros2["follow_waypoints_wait_timeout_sec"].as<int>(
                    config.follow_waypoints_wait_timeout_sec);

            config.status_upload_period_ms =
                ros2["status_upload_period_ms"].as<int>(
                    config.status_upload_period_ms);

            config.pose_log_throttle_ms =
                ros2["pose_log_throttle_ms"].as<int>(
                    config.pose_log_throttle_ms);

            // QoS 配置
            if (ros2["qos"]) {
                const YAML::Node qos = ros2["qos"];

                config.qos_depth =
                    qos["depth"].as<int>(
                        config.qos_depth);

                config.qos_reliability =
                    qos["reliability"].as<std::string>(
                        config.qos_reliability);
            }
        }

        return true;
    } catch (const YAML::BadFile& e) {
        std::cerr
            << "failed to open communication config: "
            << yaml_path
            << ", error: "
            << e.what()
            << std::endl;

        return false;
    } catch (const YAML::Exception& e) {
        std::cerr
            << "failed to parse communication config: "
            << yaml_path
            << ", error: "
            << e.what()
            << std::endl;

        return false;
    } catch (const std::exception& e) {
        std::cerr
            << "unexpected error while loading communication config: "
            << e.what()
            << std::endl;

        return false;
    }
}
