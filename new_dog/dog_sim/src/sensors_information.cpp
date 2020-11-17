#include "sensors_information.h"
sensors_information::sensors_information(){}
sensors_information::~sensors_information(){}

void sensors_information::jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for( int i=0;i<12;i++ )
    {
        if(i%3==0)
        {
            joint_pos[i] = msg->position[joints_mapping[i]];
        }
        else if (i%3==1)
        {
            joint_pos[i] = msg->position[joints_mapping[i]];
        }
        else if (i%3==2)
        {
            joint_pos[i] = msg->position[joints_mapping[i]];
        }       
        joint_vel[i] = msg->velocity[joints_mapping[i]];
    }
}

void sensors_information::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    quaternion[0], quaternion[1],quaternion[2],quaternion[3] = msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;
    angular_vel[0],angular_vel[1],angular_vel[2]  =  msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    acceleration[0],acceleration[1],acceleration[2] = msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
   
}
void sensors_information::downstreamCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for( int i=0;i<12;i++ )
    {
        double _mode = msg->data[i];
        target_mode[i] = (int)_mode;
        target_value[i] = msg->data[i+12];
    }
     
}
