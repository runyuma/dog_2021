#include "base_controller.h"

void base_controller::main_loop()
{
    double _last_current_time = current_time;
    pub_motortorque();
    current_time = ros::Time::now().toSec() - begin_time;
    loop_time = current_time - _last_current_time;
}

void base_controller::controller_init()
{
    set_subscriber();
    set_publisher();
    set_pidcontroller();
    int initial = 0;
    ros::Rate loop_rate(1000.0);
     while (ros::ok() and initial <= 1)
     {
        ros::spinOnce();
        if(initial == 1)
        {
            begin_time = ros::Time::now().toSec();
            current_time = 0.;
            for( int i=0;i<12;i++ )
            {
                set_pos(i, 0.);
            }
        }
        loop_rate.sleep();
        initial++;
     }
}


base_controller::base_controller(){}
void base_controller::set_subscriber()
{
    _sub = pnh->subscribe(_sensors_information.joint_topic, 10,&sensors_information::jointStatesCallback, &_sensors_information);
    _imusub = pnh->subscribe(_sensors_information.imu_topic,10,&sensors_information::imuCallback, &_sensors_information);
}
void base_controller::set_publisher()
{
     list<string>::iterator it1 = jointtopiclist.begin();
     for( int i=0;i<12;i++ )
    {
         jointsEffortPublishers[i] = pnh->advertise<std_msgs::Float64>(*it1,10);
        ++it1;
    }
}
void base_controller::set_pidcontroller()
{
    float p_list[12] = {p[0],p[1], p[2], p[0],p[1], p[2],  p[0],p[1], p[2],  p[0],p[1], p[2],};
    float d_list[12] = {d[0],d[1], d[2],d[0],d[1], d[2],d[0],d[1], d[2],d[0],d[1], d[2]};
    for( int i=0;i<12;i++ )
    {
        pid_list[i] = pid(p_list[i],d_list[i],15);
    }

}


void base_controller::control_pos(int _motor_id, double _pos)
{
    pid_list[_motor_id].cur_update(_sensors_information.joint_pos[_motor_id], _sensors_information.joint_vel[_motor_id]);
    pid_list[_motor_id].calculate(0, 0);
    motorcontrol_msg[_motor_id].data = pid_list[_motor_id].output;
    jointsEffortPublishers[_motor_id].publish(motorcontrol_msg[_motor_id]);
}
void base_controller::control_torque(int _motor_id, double _tor)
{
    motorcontrol_msg[_motor_id].data = _tor;
    jointsEffortPublishers[_motor_id].publish(motorcontrol_msg[_motor_id]);
}
void base_controller::set_pos(int _motor_id, double _pos)
{
    control_mode[_motor_id] = 0;
    control_value[_motor_id] = _pos; 
}
void base_controller::set_torque(int _motor_id, double _tor)
{
    control_mode[_motor_id] = 1;
    control_value[_motor_id] = _tor; 
}

void base_controller::pub_motortorque()
{
    for( int i=0;i<12;i++ )
    {
        if(control_mode[i] == 0)
        {
            control_pos(i, control_value[i]);
        }
        else
        {
            control_torque(i, control_value[i]);
        }
        
    }
}
base_controller::~base_controller(){}
    
double getCurrentTime()  
{  
   struct timeval tv;  
   gettimeofday(&tv,NULL);  
   return (double)(tv.tv_sec * 1000000 + tv.tv_usec)/1000000;  
} 
