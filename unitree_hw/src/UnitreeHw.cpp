//
// Created by skywoodsz on 2023/6/24.
//

#include "unitree_hw/UnitreeHw.h"

bool UnitreeHw::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {

    // UDP init
    ROS_INFO("\033[1;32m----> UDP Init!\033[0m");
    udp_ = std::make_shared<UNITREE_LEGGED_SDK::UDP>(UNITREE_LEGGED_SDK::HIGHLEVEL, LOCAL_PORT,
                                                     TARGET_IP, TARGET_PORT);
    udp_->InitCmdData(high_cmd_);

    // Param
    root_nh.getParam("contact_threshold", contact_threshold_);
    control_flag_ = true;

    // Publisher
    odom_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::Odometry>>(robot_hw_nh, "/dog/unitree_odom", 100);

    imu_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::Imu>>(robot_hw_nh, "/dog/imu_data", 100);

    leg_contact_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<cheetah_msgs::LegContact>>(robot_hw_nh, "/dog/leg_contact", 100);

    leg_state_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<cheetah_msgs::LegsState>>(robot_hw_nh, "/dog/leg_state", 100);

    motor_pub_ =
        std::make_shared<realtime_tools::RealtimePublisher<cheetah_msgs::MotorState>>(robot_hw_nh, "/dog/motor_data", 100);

    // Subscriber
    cmd_sub_ = robot_hw_nh.subscribe<geometry_msgs::Twist>("/dog/cmd",1,&UnitreeHw::cmdCallback,this);

    return true;
}

bool UnitreeHw::stop() {
    std::cout<<"\033[1;32m----> STOP Init!\033[0m"<<std::endl;

    // stop cmd
    control_flag_ = false;
    high_cmd_.mode = 1;
    high_cmd_.gaitType = 0;

    size_t count = 0;
    while (count<10)
    {
        count++;
        ros::WallRate(10).sleep();
    }

    std::cout<<"\033[1;32m----> STOP!\033[0m"<<std::endl;

    return true;
}


void UnitreeHw::read(const ros::Time &time, const ros::Duration &period) {
    // UDP
    udp_->Recv();
    udp_->GetRecv(high_state_);

    // IMU
    imu_data_.ori[1] = high_state_.imu.quaternion[2];
    imu_data_.ori[2] = high_state_.imu.quaternion[3];
    imu_data_.ori[3] = high_state_.imu.quaternion[0];
    imu_data_.ori[0] = high_state_.imu.quaternion[1];
    imu_data_.angular_vel[0] = high_state_.imu.gyroscope[0];
    imu_data_.angular_vel[1] = high_state_.imu.gyroscope[1];
    imu_data_.angular_vel[2] = high_state_.imu.gyroscope[2];
    imu_data_.linear_acc[0] = high_state_.imu.accelerometer[0];
    imu_data_.linear_acc[1] = high_state_.imu.accelerometer[1];
    imu_data_.linear_acc[2] = high_state_.imu.accelerometer[2];

    // odom
    odom_data_.pos[0] = high_state_.position[0];
    odom_data_.pos[1] = high_state_.position[1];
    odom_data_.pos[2] = high_state_.position[2];
    odom_data_.linear_vel[0] = high_state_.velocity[0];
    odom_data_.linear_vel[1] = high_state_.velocity[1];
    odom_data_.linear_vel[2] = high_state_.velocity[2];
    odom_data_.ori.w() = high_state_.imu.quaternion[0];
    odom_data_.ori.x() = high_state_.imu.quaternion[1];
    odom_data_.ori.y() = high_state_.imu.quaternion[2];
    odom_data_.ori.z() = high_state_.imu.quaternion[3];
    odom_data_.angular_vel[0] = high_state_.imu.gyroscope[0];
    odom_data_.angular_vel[1] = high_state_.imu.gyroscope[1];
    odom_data_.angular_vel[2] = high_state_.yawSpeed;

    // leg state
    // FL
    bleg_data_[0].bfoot_pos_[0] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::FL_].x;
    bleg_data_[0].bfoot_pos_[1] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::FL_].y;
    bleg_data_[0].bfoot_pos_[2] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::FL_].z;
    bleg_data_[0].bfoot_vel_[0] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::FL_].x;
    bleg_data_[0].bfoot_vel_[1] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::FL_].y;
    bleg_data_[0].bfoot_vel_[2] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::FL_].z;
    // FR
    bleg_data_[1].bfoot_pos_[0] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::FR_].x;
    bleg_data_[1].bfoot_pos_[1] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::FR_].y;
    bleg_data_[1].bfoot_pos_[2] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::FR_].z;
    bleg_data_[1].bfoot_vel_[0] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::FR_].x;
    bleg_data_[1].bfoot_vel_[1] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::FR_].y;
    bleg_data_[1].bfoot_vel_[2] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::FR_].z;
    // RL
    bleg_data_[2].bfoot_pos_[0] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::RL_].x;
    bleg_data_[2].bfoot_pos_[1] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::RL_].y;
    bleg_data_[2].bfoot_pos_[2] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::RL_].z;
    bleg_data_[2].bfoot_vel_[0] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::RL_].x;
    bleg_data_[2].bfoot_vel_[1] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::RL_].y;
    bleg_data_[2].bfoot_vel_[2] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::RL_].z;
    // RR
    bleg_data_[3].bfoot_pos_[0] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::RR_].x;
    bleg_data_[3].bfoot_pos_[1] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::RR_].y;
    bleg_data_[3].bfoot_pos_[2] = high_state_.footPosition2Body[UNITREE_LEGGED_SDK::RR_].z;
    bleg_data_[3].bfoot_vel_[0] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::RR_].x;
    bleg_data_[3].bfoot_vel_[1] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::RR_].y;
    bleg_data_[3].bfoot_vel_[2] = high_state_.footSpeed2Body[UNITREE_LEGGED_SDK::RR_].z;

    // leg contact
    bleg_data_[0].foot_force_[2] = high_state_.footForce[UNITREE_LEGGED_SDK::FL_];
    bleg_data_[1].foot_force_[2] = high_state_.footForce[UNITREE_LEGGED_SDK::FR_];
    bleg_data_[2].foot_force_[2] = high_state_.footForce[UNITREE_LEGGED_SDK::RL_];
    bleg_data_[3].foot_force_[2] = high_state_.footForce[UNITREE_LEGGED_SDK::RR_];

    contact_state_[0] = (high_state_.footForce[UNITREE_LEGGED_SDK::FL_] > contact_threshold_);
    contact_state_[1] = (high_state_.footForce[UNITREE_LEGGED_SDK::FR_] > contact_threshold_);
    contact_state_[2] = (high_state_.footForce[UNITREE_LEGGED_SDK::RL_] > contact_threshold_);
    contact_state_[3] = (high_state_.footForce[UNITREE_LEGGED_SDK::RR_] > contact_threshold_);

    // motor
    for (int i = 0; i < 12; ++i)
    {
        joint_data_[i].pos_ = high_state_.motorState[i].q;
        joint_data_[i].vel_ = high_state_.motorState[i].dq;
        joint_data_[i].acc_ = high_state_.motorState[i].ddq;
        joint_data_[i].tau_ = high_state_.motorState[i].tauEst;
        joint_data_[i].temperature_ = high_state_.motorState[i].temperature;
    }

    // pub msgs
    publishState(time);
}

void UnitreeHw::write(const ros::Time &time, const ros::Duration &period) {
    // TODO: rate
    if(control_flag_)
        setupCommand();
    udp_->SetSend(high_cmd_);
    udp_->Send();
}

void UnitreeHw::setupCommand() {
    double linear_x, linear_y;
    double angular_yaw;
    size_t mode;

    geometry_msgs::Twist cmd_msg_= *cmd_buffer_.readFromRT();
    linear_x = cmd_msg_.linear.x;
    linear_y = cmd_msg_.linear.y;
    angular_yaw = cmd_msg_.angular.z;

    if(linear_x == 0.0 && linear_y == 0.0 && angular_yaw ==0.0)
        mode = 1;
    else
        mode = 2;

    switch (mode) {
        case 0: // wait thye cmd
            high_cmd_.mode = 0;
            break;
        case 1: // force stand
            high_cmd_.mode = 1;
            high_cmd_.gaitType = 0;
            break;
        case 2: // force walk
            high_cmd_.mode = 2;
            high_cmd_.gaitType = 1;
            high_cmd_.velocity[0] = linear_x;
            high_cmd_.velocity[1] = linear_y;
            high_cmd_.yawSpeed    = angular_yaw;
            break;
        case 6:
            high_cmd_.mode = 6;
        default:
            high_cmd_.mode = 1;
            high_cmd_.gaitType = 0;
            ROS_ERROR_STREAM("Invalid mode ");
            break;
    }
}

void UnitreeHw::cmdCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    cmd_buffer_.writeFromNonRT(*msg);
}

void UnitreeHw::publishState(const ros::Time &time) {

    if (imu_pub_->trylock()) {
        imu_pub_->msg_.header.stamp = time;
        imu_pub_->msg_.header.frame_id = "imu_link";

        imu_pub_->msg_.orientation.x = imu_data_.ori[0];
        imu_pub_->msg_.orientation.y = imu_data_.ori[1];
        imu_pub_->msg_.orientation.z = imu_data_.ori[2];
        imu_pub_->msg_.orientation.w = imu_data_.ori[3];

        imu_pub_->msg_.linear_acceleration.x = imu_data_.linear_acc[0];
        imu_pub_->msg_.linear_acceleration.y = imu_data_.linear_acc[1];
        imu_pub_->msg_.linear_acceleration.z = imu_data_.linear_acc[2];

        imu_pub_->msg_.angular_velocity.x = imu_data_.angular_vel[0];
        imu_pub_->msg_.angular_velocity.y = imu_data_.angular_vel[1];
        imu_pub_->msg_.angular_velocity.z = imu_data_.angular_vel[2];

        imu_pub_->unlockAndPublish();
    }

    if(leg_contact_pub_->trylock())
    {
        for (int i = 0; i < 4; ++i) {
            leg_contact_pub_->msg_.contact_state[i] = contact_state_[i];
        }
        leg_contact_pub_->unlockAndPublish();
    }

    if (odom_pub_->trylock()) {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.orientation.x = odom_data_.ori.x();
        odom_pub_->msg_.pose.pose.orientation.y = odom_data_.ori.y();
        odom_pub_->msg_.pose.pose.orientation.z = odom_data_.ori.z();
        odom_pub_->msg_.pose.pose.orientation.w = odom_data_.ori.w();

        odom_pub_->msg_.pose.pose.position.x = odom_data_.pos[0];
        odom_pub_->msg_.pose.pose.position.y = odom_data_.pos[1];
        odom_pub_->msg_.pose.pose.position.z = odom_data_.pos[2];

        odom_pub_->msg_.twist.twist.angular.x = odom_data_.linear_vel[0];
        odom_pub_->msg_.twist.twist.angular.y = odom_data_.linear_vel[1];
        odom_pub_->msg_.twist.twist.angular.z = odom_data_.linear_vel[2];

        odom_pub_->msg_.twist.twist.linear.x = odom_data_.angular_vel[0];
        odom_pub_->msg_.twist.twist.linear.y = odom_data_.angular_vel[1];
        odom_pub_->msg_.twist.twist.linear.z = odom_data_.angular_vel[2];

        odom_pub_->unlockAndPublish();
    }

    if(leg_state_pub_->trylock())
    {
        leg_state_pub_->msg_.header.stamp = time;
        for (size_t leg = 0; leg < 4; leg++)
        {
            leg_state_pub_->msg_.bfoot_pos[leg].x = bleg_data_[leg].bfoot_pos_[0];
            leg_state_pub_->msg_.bfoot_pos[leg].y = bleg_data_[leg].bfoot_pos_[1];
            leg_state_pub_->msg_.bfoot_pos[leg].z = bleg_data_[leg].bfoot_pos_[2];

            leg_state_pub_->msg_.bfoot_vel[leg].x = bleg_data_[leg].bfoot_vel_[0];
            leg_state_pub_->msg_.bfoot_vel[leg].y = bleg_data_[leg].bfoot_vel_[1];
            leg_state_pub_->msg_.bfoot_vel[leg].z = bleg_data_[leg].bfoot_vel_[2];

            leg_state_pub_->msg_.foot_vel[leg].x = bleg_data_[leg].foot_vel_[0];
            leg_state_pub_->msg_.foot_vel[leg].y = bleg_data_[leg].foot_vel_[1];
            leg_state_pub_->msg_.foot_vel[leg].z = bleg_data_[leg].foot_vel_[2];

            leg_state_pub_->msg_.foot_force[leg].x = bleg_data_[leg].foot_force_[0];
            leg_state_pub_->msg_.foot_force[leg].y = bleg_data_[leg].foot_force_[1];
            leg_state_pub_->msg_.foot_force[leg].z = bleg_data_[leg].foot_force_[2];

            if(contact_state_[leg]){
                leg_state_pub_->msg_.foot_contact[leg].z = 100;
            }
            else{
                leg_state_pub_->msg_.foot_contact[leg].z = 0;
            }
        }
        leg_state_pub_->unlockAndPublish();
    }

    if(motor_pub_->trylock())
    {
        cheetah_msgs::MotorState motor_state;
        motor_state.header.stamp = time;
        for (int i = 0; i < 12; ++i)
        {
            motor_state.q[i] = joint_data_[i].pos_;
            motor_state.dq[i] = joint_data_[i].vel_;
            motor_state.ddq[i] = joint_data_[i].acc_;
            motor_state.tau[i] = joint_data_[i].tau_;
            motor_state.temperature[i] = joint_data_[i].temperature_;
        }
        motor_pub_->msg_ = motor_state;
        motor_pub_->unlockAndPublish();
    }

}


