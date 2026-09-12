#pragma once

#include <string>
#include "merman_types.h"

/**
 * @brief 从 YAML 文件加载通信配置
 *
 * @param yaml_path YAML 文件路径
 * @param config 输出配置
 * @return true 加载成功
 * @return false 加载失败
 */
bool LoadCommunicationConfig(
    const std::string& yaml_path,
    CommunicationConfig& config);