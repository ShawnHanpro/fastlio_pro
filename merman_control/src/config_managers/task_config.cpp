/*
 * @Author: toony haoyangzhou1225@163.com
 * @Date: 2026-07-20 16:07:29
 * @LastEditors: toony haoyangzhou1225@163.com
 * @LastEditTime: 2026-07-20 17:45:04
 * @FilePath: /slam_nav/merman_control/src/config_managers/task_config.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "config_managers/task_config.h"

#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>

bool LoadTaskConfig(
    const std::string& yaml_path,
    TaskConfig& config)
{
    try {
        YAML::Node root = YAML::LoadFile(yaml_path);

        if (!root["task"]) {
            std::cerr
                << "task config not found in yaml: "
                << yaml_path
                << std::endl;
            return false;
        }

        const YAML::Node task = root["task"];

        // 路点配置
        if (task["waypoint"]) {
            const YAML::Node waypoint = task["waypoint"];

            config.waypoints_file =
                waypoint["points_file"].as<std::string>(
                    config.waypoints_file);
        }

        // 旋转配置
        if (task["rotation"]) {
            const YAML::Node rotation = task["rotation"];

            config.rotation_angle_tolerance_deg =
                rotation["angle_tolerance_deg"].as<double>(
                    config.rotation_angle_tolerance_deg);

            config.rotation_kp =
                rotation["kp"].as<double>(
                    config.rotation_kp);

            config.rotation_max_angular_speed_rad_s =
                rotation["max_angular_speed_rad_s"].as<double>(
                    config.rotation_max_angular_speed_rad_s);

            config.rotation_min_angular_speed_rad_s =
                rotation["min_angular_speed_rad_s"].as<double>(
                    config.rotation_min_angular_speed_rad_s);

            config.rotation_control_period_ms =
                rotation["control_period_ms"].as<int>(
                    config.rotation_control_period_ms);

            config.rotation_stable_count_required =
                rotation["stable_count_required"].as<int>(
                    config.rotation_stable_count_required);

            config.rotation_max_pose_failure_count =
                rotation["max_pose_failure_count"].as<int>(
                    config.rotation_max_pose_failure_count);

            config.rotation_timeout_sec =
                rotation["timeout_sec"].as<int>(
                    config.rotation_timeout_sec);
        }

        return true;
    } catch (const YAML::BadFile& e) {
        std::cerr
            << "failed to open task config: "
            << yaml_path
            << ", error: "
            << e.what()
            << std::endl;

        return false;
    } catch (const YAML::Exception& e) {
        std::cerr
            << "failed to parse task config: "
            << yaml_path
            << ", error: "
            << e.what()
            << std::endl;

        return false;
    } catch (const std::exception& e) {
        std::cerr
            << "unexpected error while loading task config: "
            << e.what()
            << std::endl;

        return false;
    }
}
