 #ifndef _SENSOR_INFORMATION_H_ 
#define _SENSOR_INFORMATION_H_ 
#include <ros/ros.h>
#include <string>
using namespace std;
#include <std_msgs/Float32.h>
#include<sensor_msgs/Imu.h>
#include<sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
class sensors_information
{
private:
    int joints_mapping[12] =  {3, 4, 5, 9, 10, 11, 0, 1, 2,6, 7, 8};
    
public:
    const string joint_topic = "/joint_states"; 
    // not add &
    const string imu_topic = "/imu";
    const string downstream_topic = "/downstream";
    // ros::Subscriber imu_subscriber, jointstates_subscriber;
    double joint_pos[12], joint_vel[12], quaternion[4], angular_vel[3],acceleration[3],target_value[12];//target_value upperconcole
    int target_mode[12] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1 };
    double joint_pos_PC[12], joint_vel_PC[12];
    sensors_information();
    ~sensors_information();
    void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void downstreamCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
};

#endif 