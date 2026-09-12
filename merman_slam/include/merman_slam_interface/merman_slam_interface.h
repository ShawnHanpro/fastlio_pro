#pragma once

#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>

#include "merman_types.h"

class MermanSlamInterface {
private:
    MermanSlamInterface();
    ~MermanSlamInterface();

public:
    static MermanSlamInterface *GetInstance() {
        static MermanSlamInterface instance;
        return &instance;
    }

    static std::string get_slam_version();

    void Exit();
    void UpdateLaserScan(const LaserScan &laser_scan);
    
    void get_map();
    void get_pose();
    
private:
    void FeedLaserScan();
    
    bool exit_ = false;
    bool new_laser_scan_;
    bool end_laser_scan_ = true;
    bool feed_laser_scan_ = false;

    // std::shared_ptr<Rtabmap> rtabmap_;

    std::mutex              laser_scan_mutex_;
    std::condition_variable laser_scan_cv_;

    std::queue<LaserScan> laser_scan_queue_;

    LaserScan laser_scan_;
};
