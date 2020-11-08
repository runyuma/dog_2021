#include "sensors_information.h"
sensors_information::sensors_information(){}
sensors_information::~sensors_information(){}

void sensors_information::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for( int i=0;i<12;i++ )
    {
        joint_pos[i] = msg->position[joints_mapping[i]];
        joint_vel[i] = msg->velocity[joints_mapping[i]];
        // cout <<joint_pos[i] <<endl ;
    }
}

void sensors_information::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    quaternion[0], quaternion[1],quaternion[2],quaternion[3] = msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
    angular_vel[0],angular_vel[1],angular_vel[2]  =  msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    acceleration[0],acceleration[1],acceleration[2] = msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    // cout <<quaternion[0] <<endl ;
   
}
