#include<iostream>
using namespace std;
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/Imu.h>
#include  "../include/pid.h" 
#include "base_controller.h"
#include <unistd.h>
int main(int argc, char** argv)
{
    // base_controller _base_controller = base_controller(argc,argv);
 
    base_controller _base_controller;
    ros::NodeHandle *pnh;
    ros::init(argc, argv, "control");
    _base_controller.pnh = new ros::NodeHandle();
    _base_controller.controller_init();
    ros::Rate loop_rate(1000.0);
    cout<<_base_controller.begin_time<<endl;
    int i = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        _base_controller.main_loop();
        if(i%1000 == 0)
        {
            cout<<"mode"<<_base_controller.control_mode[0]<<endl;
            cout<<"value"<<_base_controller.control_value[0]<<endl;
            i ++;
        } 
        loop_rate.sleep();
    }
    
    return 0;
}
