/****************************************************************************\
 * @file    StateEstimationLogic.cpp
 * @author  CQS
 * @date    2021/04/04
 * @brief   
 ******************************************************************************
 * @attent  
 ****************************************************************************/
#include "StateEstimationLogic.h"
#include <ros/ros.h>

/* User Config */
#define LOOPRATE    50
/* User Config */


int main(int argc, char **argv){
    ros::init(argc, argv, "state_estimation");    // 创建Node
    ros::NodeHandle nh;                     // 和topic service param等交互的公共接口，是操作节点的凭据
    ros::Rate loop_rate(LOOPRATE);

    while (ros::ok())
    {
        loop_rate.sleep();
    }
}




