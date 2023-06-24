//
// Created by skywoodsz on 2023/6/24.
//

#ifndef SRC_HWLOOP_H
#define SRC_HWLOOP_H

#include <chrono>
#include <thread>

#include <ros/ros.h>

#include "unitree_hw/UnitreeHw.h"

class HwLoop{
using Clock = std::chrono::high_resolution_clock;
using Duration = std::chrono::duration<double>;

public:
    HwLoop(ros::NodeHandle& nh, std::shared_ptr<UnitreeHw> hardware_interface);
    ~HwLoop();

    void update();

private:
    // ROS Param
    ros::NodeHandle nh_;

    // Timer Param
    double cycleTimeErrorThreshold_{}, loopHz_{};
    std::thread loopThread_;
    std::atomic_bool loopRunning_{};
    ros::Duration elapsedTime_;
    Clock::time_point lastTime_;

    std::shared_ptr<UnitreeHw> hardwareInterface_;

};


#endif //SRC_HWLOOP_H
