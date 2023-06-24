//
// Created by skywoodsz on 2023/6/24.
//

#ifndef SRC_UNITREEHW_H
#define SRC_UNITREEHW_H

#include <memory>
#include <string>
#include <vector>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>

// ROS
#include <ros/ros.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <tf2_ros/transform_broadcaster.h>

// ROS control
#include <hardware_interface/robot_hw.h>

// Unitree
#include "unitree_legged_sdk/unitree_legged_sdk.h"

// Msgs
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <cheetah_msgs/LegContact.h>
#include <cheetah_msgs/LegsState.h>
#include <cheetah_msgs/MotorState.h>

using namespace UNITREE_LEGGED_SDK;

// high cmd
constexpr uint16_t TARGET_PORT = 8082;
constexpr uint16_t LOCAL_PORT = 8090;
constexpr char TARGET_IP[] = "192.168.123.220";   // target IP address


//// low cmd
//constexpr uint16_t TARGET_PORT = 8007;
//constexpr uint16_t LOCAL_PORT = 8082;
//constexpr char TARGET_IP[] = "192.168.123.10";   // target IP address

struct UnitreeOdomData
{
    Eigen::Vector3d pos;
    Eigen::Quaterniond ori;
    Eigen::Vector3d linear_vel;
    Eigen::Vector3d angular_vel;
};

struct UnitreeImuData
{
    double ori[4];
    double ori_cov[9];
    double angular_vel[3];
    double angular_vel_cov[9];
    double linear_acc[3];
    double linear_acc_cov[9];
};

struct UnitreeFootState
{
    Eigen::Vector3d foot_vel_;
    Eigen::Vector3d bfoot_pos_;
    Eigen::Vector3d bfoot_vel_;
    Eigen::Vector3d foot_force_;
};

struct UnitreeMotorData
{
    double pos_, vel_, tau_, acc_, temperature_;      // state
    double pos_des_, vel_des_, kp_, kd_, ff_;  // command
};


class UnitreeHw : public hardware_interface::RobotHW{
public:
    UnitreeHw() = default;

    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

    void read(const ros::Time& time, const ros::Duration& period) override;

    void write(const ros::Time& time, const ros::Duration& period) override;

    bool stop();

private:
    void publishState(const ros::Time& time);
    void setupCommand();
    void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg);

    // UDP
    std::shared_ptr<UNITREE_LEGGED_SDK::UDP> udp_;
    UNITREE_LEGGED_SDK::HighState high_state_{};
    UNITREE_LEGGED_SDK::HighCmd high_cmd_{};

    // State
    UnitreeOdomData odom_data_{};
    UnitreeImuData imu_data_{};
    bool contact_state_[4]{};
    UnitreeFootState bleg_data_[4]{};
    UnitreeMotorData joint_data_[12]{};

    // Param
    int contact_threshold_{};
    bool control_flag_{};

    // Pub
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::Imu>> imu_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<cheetah_msgs::LegContact>> leg_contact_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<cheetah_msgs::LegsState>> leg_state_pub_;
    std::shared_ptr<realtime_tools::RealtimePublisher<cheetah_msgs::MotorState>> motor_pub_;

    // Sub
    ros::Subscriber cmd_sub_;
    realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_buffer_;
};



#endif //SRC_UNITREEHW_H
