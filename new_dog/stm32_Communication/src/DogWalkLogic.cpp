/****************************************************************************\
 * @file    DogWalkLogic.cpp
 * @author  CQS
 * @date    2021/04/17
 * @brief   
 ******************************************************************************
 * @attent  四足机器人位置环行走逻辑
 *          每条腿腾空1
 ****************************************************************************/
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>

#define SWINGLEG_TIME       1000                // 每条腿的腾空时间
#define FORCELEG_TIME       SWINGLEG_TIME * 3   // 每条腿的触地时间
#define UPPERLEG_LENGTH     215                 // 大腿长度[mm]
#define LOWERLEG_LENGTH     210                 // 小腿长度[mm]

int main(int argc, char **argv){
    ros::init(argc, argv, "DogWalkLogic");    // 创建Node
    ros::NodeHandle nh;                     // 和topic service param等交互的公共接口，是操作节点的凭据

    // ros::Publisher UpStreamPub = nh.advertise<std_msgs::Float32MultiArray>("/upstream", UPQUEUESIZE);   // 发布12个电机的数据

    ros::Rate loop_rate(1000.0);
    while(ros::ok()){

        loop_rate.sleep();
    }
}

/** @brief 触地腿逻辑 */
void TouchLegLogic(){

}

/** @brief 腿逆运动学计算 */


