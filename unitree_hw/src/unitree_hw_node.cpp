//
// Created by skywoodsz on 2023/6/24.
//

#include "unitree_hw/UnitreeHw.h"
#include "unitree_hw/HwLoop.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "qm_hw");
    ros::NodeHandle nh;
    ros::NodeHandle robotHwNh("~");

    ros::AsyncSpinner spinner(3);
    spinner.start();

    ROS_INFO("\033[1;32m Main Loop Start! \033[0m");
    try{
        std::shared_ptr<UnitreeHw> unitreeHw = std::make_shared<UnitreeHw>();

        unitreeHw->init(nh, robotHwNh);

        HwLoop controlLoop(nh, unitreeHw);

        ros::waitForShutdown();
    } catch (const ros::Exception& e) {
        ROS_FATAL_STREAM("Error in the hardware interface:\n"
                                 << "\t" << e.what());
        return 1;
    }

    return 0;
}
