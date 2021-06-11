#include<iostream>
using namespace std;
#include <stdio.h>
#include <ros/ros.h>
#include"leg_controller.h"

leg_controller Legcontroller;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leg_controller");
    Legcontroller.pnh =  new ros::NodeHandle();
    Legcontroller.legcontroller_init();
    ros::Rate loop_rate(1000.0);

    while (ros::ok())
    {
        double start_time = getCurrentTime();
        ros::spinOnce();
        if(Legcontroller.time_index%1000 == 1)
        {
            cout<<"spin time is :"<<getCurrentTime() - start_time<<endl;
        }
        Legcontroller.main();
        if(Legcontroller.time_index%1000 == 0)
        {
            Legcontroller.visual();
        }
        if(Legcontroller.time_index%1000 == 1)
        {
            double loop_time = getCurrentTime() - start_time;
            cout<<"loop_time is :"<<loop_time<<endl;
        }
        Legcontroller.time_index++ ;
        loop_rate.sleep();
    }
    cout<<"delete"<<endl;
    delete Legcontroller.pnh;
}