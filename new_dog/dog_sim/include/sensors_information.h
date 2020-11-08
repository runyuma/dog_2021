 #ifndef _SENSOR_INFORMATION_H_ 
#define _SENSOR_INFORMATION_H_ 
#include <ros/ros.h>
#include <string>
using namespace std;
#include <std_msgs/Float32.h>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/JointState.h>
class sensors_information
{
private:
    int joints_mapping[12] =  {3, 4, 5, 9, 10, 11, 0, 1, 2,6, 7, 8};
    
public:
    const string joint_topic = "/joint_states"; 
    // not add &
    const string imu_topic = "/imu";
    ros::Subscriber imu_subscriber, jointstates_subscriber;
    double joint_pos[12], joint_vel[12], quaternion[4], angular_vel[3],acceleration[3];
    sensors_information();
    ~sensors_information();
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
};

#endif 