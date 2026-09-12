/*
 * @Author: Shengyu Han hanshy2@shanghai-electric.com
 * @Date: 2026-07-17 10:59:33
 * @LastEditors: Shengyu Han hanshy2@shanghai-electric.com
 * @LastEditTime: 2026-07-17 13:14:07
 * @FilePath: /slam_nav/merman_slam/src/merman_slam_interface/merman_slam_interface.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <string>
#include "merman_slam_interface/merman_slam_interface.h"
#include "merman_logger/merman_logger.hpp"

MermanSlamInterface::MermanSlamInterface() {
    new_laser_scan_ = false;
    feed_laser_scan_ = false;
}

MermanSlamInterface::~MermanSlamInterface() = default;

std::string MermanSlamInterface::get_slam_version(){
    ML_INFO("--- SLAM Version queried: v0.1 ---");
    
    // 测试浮点数各种小数位数格式化 (直接在格式化字符串中指定)
    double pi_val = 3.14159265;
    ML_INFO("Formatting Float (2 decimals): {:.2f}", pi_val);
    ML_INFO("Formatting Float (3 decimals): {:.3f}", pi_val);
    ML_INFO("Formatting Float (4 decimals): {:.4f}", pi_val);
    
    // 测试布尔值的打印 (打印 true/false 形式)
    bool is_active = true;
    bool is_failed = false;
    ML_INFO("Formatting Boolean: active={}, failed={}", is_active, is_failed);
    
    // 测试 Stream 流式打印的布尔值 (应当输出 true/false 形式)
    ML_INFO_STREAM << "Formatting Boolean in Stream: active=" << is_active << ", failed=" << is_failed;
    
    return "v0.1";
}

void MermanSlamInterface::Exit() {
    exit_ = true;
}

void MermanSlamInterface::UpdateLaserScan(const LaserScan &laser_scan) {
    std::unique_lock<std::mutex> laser_scan_lock(laser_scan_mutex_);
    laser_scan_          = laser_scan;
    new_laser_scan_ = true;
    if (feed_laser_scan_) {
        laser_scan_queue_.push(laser_scan);
    }
    laser_scan_cv_.notify_all();
}

void MermanSlamInterface::FeedLaserScan() {
    
    auto start = std::chrono::steady_clock::now();
    
    int64_t    time_pre = 0;
    while (!exit_) {
        {
            std::unique_lock<std::mutex> laser_scan_lock(laser_scan_mutex_);
            laser_scan_cv_.wait(laser_scan_lock, [&]() { return feed_laser_scan_ && new_laser_scan_; });
            end_laser_scan_     = false;
            new_laser_scan_ = false;
        }

        auto update_scan_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        if (update_scan_time > 200) {

        }

        while (!laser_scan_queue_.empty()) {
            LaserScan laser_scan;
            laser_scan = laser_scan_queue_.front(); 
            laser_scan_queue_.pop(); 

            // todo: 处理第一帧时间戳
            int64_t time_now = laser_scan.start_time;
            int64_t duration = time_now - time_pre;
            time_pre         = time_now;

            if (duration <= 0) {
                
                continue;
            }

            int feed_laser_scan_result = -1;
            // feed_laser_scan_result = rtabmap_->FeedLidar(laser_scan);

        }

        end_laser_scan_ = true;
    }
}

void MermanSlamInterface::get_map() {

}

void MermanSlamInterface::get_pose() {

}