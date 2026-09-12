#include "lidar/livox_mid360s.h"
#include "merman_control_interface/merman_control_interface.h"

LivoxMid360s::LivoxMid360s(){}

void LivoxMid360s::run() {

    while (is_data_) {
        SendData();
    }

}

void LivoxMid360s::SendData() {

    //包装正常的激光数据
    LaserScan laser_scan;
    MermanControlInterface::GetInstance()->UpdateLaserScan(laser_scan);
}