#ifndef _BASE_CONTROLLER_H_ 
#define _BASE_CONTROLLER_H_
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include<sensor_msgs/JointState.h>
#include<sensor_msgs/Imu.h>
#include  "../include/pid.h" 
#include "time.h"
#include "sensors_information.h"
#include<list>
#include<string>
#include <sys/time.h>  
#include<sys/select.h>
class base_controller
{
private:
    list<string> jointtopiclist{"/Pos_controller1/command", "/Pos_controller2/command", "/Pos_controller3/command", "/Pos_controller4/command", "/Pos_controller5/command", "/Pos_controller6/command", "/Pos_controller7/command", "/Pos_controller8/command", "/Pos_controller9/command", "/Pos_controller10/command", "/Pos_controller11/command", "/Pos_controller12/command" };
    const string upstreamtopic = "/upstream";
    float p[3] = {40,40,50};
    float d[3] = {0.5,0.5,0.3};
    void control_pos(int _motor_id, double _pos);
    void control_torque(int _motor_id, double _tor);
    int time_index = 0;
public:
    ros::NodeHandle *pnh;
    ros::Subscriber _sub;
    ros::Subscriber _imusub;
    ros::Subscriber _downstreamsub;
    ros::Publisher jointsEffortPublishers[12];
    ros::Publisher upstreamPublisher;
    int control_mode[12]; // mode to control moter -1:uninitial 0:pos 1:vel 2:torque 3:enable 4:disable
    double control_value[12];
    std_msgs::Float64 motorcontrol_msg[12];
    std_msgs::Float32MultiArray sensors_msg;
    pid pid_list[12];
    sensors_information _sensors_information;
    double begin_time;
    double current_time;
    double loop_time;
    base_controller();
    ~base_controller();
    void set_subscriber();
    void set_publisher();
    void set_pidcontroller();
    void controller_init();
    void main_loop();
    void set_pos(int _motor_id, double _pos);
    void set_torque(int _motor_id, double _tor);
    void pub_motortorque();
    void upstreamPublish();
};
double getCurrentTime();

#endif 
