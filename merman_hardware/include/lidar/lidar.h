#pragma once

class Lidar {
public:
    Lidar()  = default;
    ~Lidar() = default;

    virtual void run() = 0;

    bool if_stop() { return stop_; }

    void set_stop(bool stop) { stop_ = stop; }

    bool if_lidar_error() { return lidar_error_; }

    void reset() {}

    bool        stop_                        = true;
    bool        lidar_error_                 = false;
    bool is_data_ = true;

    // 雷达信息(公司和型号)
    struct LidarInfo {
        enum Company {
            LIVOX=0
        };
        enum Type {

            MID360s = 0x01
        };

        LidarInfo() : company(Company::LIVOX), type(Type::MID360s) {}

        Company company;
        Type    type;
    };

    LidarInfo lidar_info_;  // 雷达信息结构体
};