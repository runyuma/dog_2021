#include "base_controller.h"

void base_controller::main_loop()
{
    double _last_current_time = current_time;
    pub_motortorque();
    upstreamPublish();
    current_time = ros::Time::now().toSec() - begin_time;
    loop_time = current_time - _last_current_time;

}

void base_controller::controller_init()
{
    set_subscriber();
    set_publisher();
    set_pidcontroller();
    int initial = 0;
    for( int i=0;i<12;i++ )
    {
        control_mode[i] == -1;// control calue not initialed
        motorcontrol_msg[i].data = 0;
    }
    ros::Rate loop_rate(1000.0);
     while (ros::ok() and initial <= 1)
     {
        ros::spinOnce();
        if(initial == 1)
        {
            begin_time = ros::Time::now().toSec();
            current_time = 0.;
        }
        loop_rate.sleep();
        initial++;
     }
}

void base_controller::pub_motortorque()
//upperconcole data to downconcole 
//publish 
{
    for( int i=0;i<12;i++ )
    {
        control_mode[i] = _sensors_information.target_mode[i];
        if(control_mode[i] == -1)
        {
            control_value[i] = 0.;
            control_torque(i, control_value[i]);
             //do nothing
        }
        else if (control_mode[i] == 0)
        {
            if(i%3==0)
            {
                control_value[i] = _sensors_information.target_value[i] ;
            }
            else if (i%3==1)
            {
                control_value[i] = _sensors_information.target_value[i] - 0.78;
            }
            else if (i%3 == 2)
            {
                control_value[i] = _sensors_information.target_value[i] + 1.57;
            }
            control_pos(i, control_value[i]);
        }
        else if (control_mode[i] == 1)
        {
           control_value[i] = _sensors_information.target_value[i] ;
        }
        else if (control_mode[i] == 2)
        {
            control_value[i] = _sensors_information.target_value[i] ;
            control_torque(i, control_value[i]);
        }
        else if (control_mode[i] == 3)
        {
            /* code */
        }
        else if (control_mode[i] == 4)
        {
            /* code */
        }    
        
    }
}

void base_controller::upstreamPublish()
{
    sensors_msg.data.clear();
    float _msg[24];
    for( int i=0;i<12;i++ )
    {
         if(i%3==0)
         {
           _msg[i] = _sensors_information.joint_pos[i];
            _msg[i+12] = _sensors_information.joint_vel[i];
         }
         else if (i%3==1)
         {
            _msg[i] = _sensors_information.joint_pos[i] + 0.78;
            _msg[i+12] = _sensors_information.joint_vel[i];
         }
         else if (i%3 == 2)
         {
            _msg[i] = _sensors_information.joint_pos[i] - 1.57;
            _msg[i+12] = _sensors_information.joint_vel[i];
         }
    }
    for( int i=0;i<24;i++ )
    {
        sensors_msg.data.push_back(_msg[i]);
    }
    upstreamPublisher.publish(sensors_msg);
    time_index ++;
}

void base_controller::control_pos(int _motor_id, double _pos)
{
    pid_list[_motor_id].cur_update(_sensors_information.joint_pos[_motor_id], _sensors_information.joint_vel[_motor_id]);
    pid_list[_motor_id].calculate(_pos, 0);
    motorcontrol_msg[_motor_id].data = pid_list[_motor_id].output;
    jointsEffortPublishers[_motor_id].publish(motorcontrol_msg[_motor_id]);
    if(time_index%1000 ==0)
    {
        cout<<"id: "<< _motor_id <<"tor   "<<motorcontrol_msg[_motor_id].data<<"pos   "<<_sensors_information.joint_pos[_motor_id]<<"target   "<<_pos<<endl;
    }
    
}
void base_controller::control_torque(int _motor_id, double _tor)
{
    motorcontrol_msg[_motor_id].data = _tor;
    jointsEffortPublishers[_motor_id].publish(motorcontrol_msg[_motor_id]);
    if(time_index%1000 ==0)
    {
    cout<<"id: "<< _motor_id<<"tor"<<motorcontrol_msg[_motor_id].data<<endl;
    }
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

base_controller::base_controller(){}
base_controller::~base_controller(){}
void base_controller::set_subscriber()
{
    _sub = pnh->subscribe(_sensors_information.joint_topic, 10,&sensors_information::jointStatesCallback, &_sensors_information);
    _imusub = pnh->subscribe(_sensors_information.imu_topic,10,&sensors_information::imuCallback, &_sensors_information);
    _downstreamsub = pnh->subscribe(_sensors_information.downstream_topic,10,&sensors_information::downstreamCallback, &_sensors_information);
}
void base_controller::set_publisher()
{
     list<string>::iterator it1 = jointtopiclist.begin();
     for( int i=0;i<12;i++ )
    {
         jointsEffortPublishers[i] = pnh->advertise<std_msgs::Float64>(*it1,10);
        ++it1;
    }
    upstreamPublisher = pnh->advertise<std_msgs::Float32MultiArray>(upstreamtopic,10);
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
double getCurrentTime()  
{  
   struct timeval tv;  
   gettimeofday(&tv,NULL);  
   return (double)(tv.tv_sec * 1000000 + tv.tv_usec)/1000000;  
} 
