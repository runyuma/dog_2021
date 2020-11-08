#include<iostream>
using namespace std;
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/Imu.h>
#include  "../include/pid.h" 
#include "sensors_information.h"
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
    while (ros::ok())
    {
        _base_controller.main_loop();
        cout<<_base_controller.loop_time<<endl;
        ros::spinOnce();
        loop_rate.sleep();
    }
    // ros::init(argc, argv, "control");
    // ros::NodeHandle nh;
    
    // sensors_information _sensors_information;
    // ros::Subscriber _sub = nh.subscribe(_sensors_information.joint_topic, 10,&sensors_information::jointStatesCallback, &_sensors_information);
//     ros::Subscriber _imusub = nh.subscribe(_sensors_information.imu_topic,10,&sensors_information::imuCallback, &_sensors_information);
//     list<string> jointtopiclist{"/Pos_controller1/command", "/Pos_controller2/command", "/Pos_controller3/command", "/Pos_controller4/command", "/Pos_controller5/command", "/Pos_controller6/command", "/Pos_controller7/command", "/Pos_controller8/command", "/Pos_controller9/command", "/Pos_controller10/command", "/Pos_controller11/command", "/Pos_controller12/command" };
//     ros::Publisher jointsEffortPublishers[12];
//     list<string>::iterator it1 = jointtopiclist.begin();
//     std_msgs::Float64 msg[12];
//     for( int i=0;i<12;i++ )
//     {
//          jointsEffortPublishers[i] = nh.advertise<std_msgs::Float64>(*it1,10);
//         ++it1;
//     }
//     float p1 = 15;
//     float p2 = 20;
//     float p3 = 10;
//     float d1 = 2;
//     float d2 = 2;
//     float d3 = 0.1;

//     float p_list[12] = {p1,p2, p3, p1,p2, p3, p1,p2, p3, p1,p2, p3,};
//     float d_list[12] = {d1,d2,d3,d1,d2,d3,d1,d2,d3,d1,d2,d3,};
//     pid pid_list[12];
//     for( int i=0;i<12;i++ )
//     {
//         pid_list[i] = pid(p_list[i],d_list[i],15);
//     }

//     double loop_time = 0.0001;
//     ros::Rate loop_rate(1000.0);
//     double begin_time, current_ros_time,ros_loop_time;
//     int initial = 0;
//     double start, finish,last,duration; 
//     start = getCurrentTime() ;
//     last =   getCurrentTime() ;
//     while (ros::ok() and initial <= 1)
// //  first call time is zero
//     {
//         ros::spinOnce();
//         begin_time = ros::Time::now().toSec();
//         current_ros_time = begin_time;
//         loop_rate.sleep();
//         initial ++ ;
//     }


//     while (ros::ok())
//     {
        
//         double loop_start = getCurrentTime() ;
//         ros::spinOnce();
//         for( int i=0;i<12;i++ )
//         {
//             pid_list[i].cur_update(_sensors_information.joint_pos[i], _sensors_information.joint_vel[i]);
//             pid_list[i].calculate(0, 0);
//         }
//         msg[0].data = pid_list[0].output;
//         msg[1].data = pid_list[1].output;
//         msg[2].data = pid_list[2].output;
//         jointsEffortPublishers[0].publish(msg[0]);
//         jointsEffortPublishers[1].publish(msg[1]);
//         jointsEffortPublishers[2].publish(msg[2]);
//         cout <<"pos is :"<<pid_list[2].cur_pos<<"vel is :"<< pid_list[2].cur_vel<<endl ; 
//         cout <<"outcom is :"<<pid_list[2].output <<endl ; 
//         double last_time = current_ros_time;
//         current_ros_time =  ros::Time::now().toSec()-begin_time;
//         ros_loop_time = current_ros_time - last_time;
//         finish = getCurrentTime() ;
//         duration = (finish - last) ;
//         last = finish;
//          double loop_current_time  =  getCurrentTime() - loop_start;
//         //  cout <<"looptime is :"<<duration <<endl ; 
//          if(loop_current_time <= loop_time )
//          {
//              int utime = (int)(1000000*(loop_time - loop_current_time));
//              usleep(utime);
//          }

//     }
    
    return 0;
}
