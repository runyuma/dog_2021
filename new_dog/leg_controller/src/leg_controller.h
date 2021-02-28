#ifndef LEG_CONTROLLER
#define LEG_CONTROLLER
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
using namespace std;
#include<sys/select.h>
#include"leg_dynamic.h"
// int __NaN=0xFFC00000;
// const float NaN=*((float *)&__NaN);

class leg_controller
{
private:
    
public:
    Eigen::Vector3f foot_points[4];
    Eigen::Vector3f foot_vel[4];
    float target_mode[12];
    float target_value[12];
    std_msgs::Float32MultiArray footpoint_pubmsg;
    std_msgs::Float32MultiArray footvel_pubmsg;
    std_msgs::Float32MultiArray jointtarget_pubmsg; 

    float target_force[12];
    float target_swing[24];
    float joint_pos[12];
    float joint_vel[12];
    int leg_status[4] = {-1,-1,-1,-1};
    int leg_getvalue[4] = {0,0,0,0};

    int use_sim;
     std::vector<float> damping_compensation; 
    // third joint torque compensation
    
    ros::NodeHandle *pnh;
    ros::Subscriber sensor_subscriber;
    ros::Subscriber force_subscriber;
    ros::Subscriber swingleg_subscriber;
    ros::Subscriber leg_status_subscriber;

    ros::Publisher footpoint_publisher;
    ros::Publisher footvel_publisher;
    ros::Publisher jointtarget_publisher;
    leg_params Leg_parameter;
    int time_index = 0;
    leg_controller(/* args */);
    ~leg_controller();
    void legcontroller_init();
    void sensor_callback(const  std_msgs::Float32MultiArray::ConstPtr& msg);
    void groundforce_callback(const  std_msgs::Float32MultiArray::ConstPtr& msg);
    void swingleg_callback(const  std_msgs::Float32MultiArray::ConstPtr& msg);
    void leg_status_callback(const  std_msgs::Int32MultiArray::ConstPtr& msg);
    void visual();
    void main();
    void jointtarget_publish();
    void footpoint_publish();
};

double getCurrentTime();




#endif // LEG_CONTROLLER
