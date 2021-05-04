#include"leg_controller.h"
#define SIGN(x) (x>0 ? 1 : -1)
void leg_controller::sensor_callback(const  std_msgs::Float32MultiArray::ConstPtr& msg)
{
    for( int i=0;i<4;i++ )
    {
        if(leg_getvalue[i])
        {
            for( int j=0;j<3;j++ )
            {
                joint_pos[3*i+j] = msg->data[3*i+j];
                joint_vel[3*i+j] = msg->data[3*i+j+12];
            }
        // cout<<i <<"get_value"<<endl;
        }
        else
        {
            int judge = 1;
            for( int j=0;j<3;j++ )
            {
                if (isnan(msg->data[3*i+j]) or isnan(msg->data[3*i+j+12]))
                {
                    judge = 0;
                }

            }
            if (judge)
            {
                leg_getvalue[i] = 1;
                cout<<i<<"changed"<<endl;
            }
            else
            {
                //  cout<<i<<"nan"<<endl;
            }
        }
    }
    
}
void leg_controller::groundforce_callback(const  std_msgs::Float32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() == 12)
    {
    for( int i=0;i<12;i++ )
    {
        target_force[i] = msg->data[i];
    }
    }
}
void leg_controller::swingleg_callback(const  std_msgs::Float32MultiArray::ConstPtr& msg)
{
    
    if (msg->data.size() == 24)
    {
        for( int i=0;i<24;i++ )
        {
        target_swing[i] = msg->data[i];
        }
        // cout<<"  "<<target_swing[0]<<endl;
    }
 }
void leg_controller::leg_status_callback(const  std_msgs::Int32MultiArray::ConstPtr& msg)
{
    if (msg->data.size() == 4)
    {
        for( int i=0;i<4;i++ )
        {
            leg_status[i] = msg->data[i];
            // cout<< leg_status[i];
        }
    }
    
}
leg_controller::leg_controller(/* args */)
{
}
leg_controller::~leg_controller()
{
}
void leg_controller::legcontroller_init()
{
sensor_subscriber =  pnh->subscribe("/upstream", 10, &leg_controller::sensor_callback , this);
force_subscriber = pnh->subscribe("/ground_force", 10, &leg_controller::groundforce_callback ,this);
swingleg_subscriber = pnh->subscribe("/swing_leg", 10,&leg_controller:: swingleg_callback ,this);
leg_status_subscriber = pnh->subscribe("/leg_status", 10, &leg_controller::leg_status_callback, this );

footpoint_publisher =  pnh->advertise<std_msgs::Float32MultiArray>("/foot_points",10);
footvel_publisher = pnh->advertise<std_msgs::Float32MultiArray>("/foot_vel",10);
jointtarget_publisher  = pnh->advertise<std_msgs::Float32MultiArray>("/downstream",10);


for( int i=0;i<12;i++ )
{
    footpoint_pubmsg.data.push_back(0);
    footvel_pubmsg.data.push_back(0);
    jointtarget_pubmsg.data.push_back(0);
    jointtarget_pubmsg.data.push_back(0);
}
float mass_list[3] ;
float lenth_list[3];
float discomlist[3] ;

pnh-> getParam("use_sim",use_sim);

pnh-> getParam("damping_compensation",damping_compensation);
pnh-> getParam("hip_mass",mass_list[0]);
pnh-> getParam("upper_link_mass",mass_list[1]);
pnh-> getParam("lower_link_mass",mass_list[2]);
pnh-> getParam("hip_lenth",lenth_list[0]);
pnh-> getParam("upper_link_lenth",lenth_list[1]);
pnh-> getParam("lower_link_lenth",lenth_list[2]);
pnh-> getParam("coma_lenth",discomlist[0]);
pnh-> getParam("comb_lenth",discomlist[1]);
pnh-> getParam("comc_lenth",discomlist[2]);



Leg_parameter = leg_params(mass_list,lenth_list,discomlist);
}

void leg_controller::main()
{
    for( int i=0;i<4;i++ )
    {
        int sidesign;
        if (i%2 == 0)
        {sidesign = 1;}
        else
        {sidesign = -1;}

        float _joint_pos[3],_joint_vel[3];
        for( int j=0;j<3;j++ )
        {
            _joint_pos[j] = joint_pos[j + 3*i];
             _joint_vel[j] = joint_vel[j + 3*i];
        }
        if(leg_getvalue[i] == 1 )
        {
            Eigen::Vector3f _foot_point = get_footpoints(sidesign, _joint_pos, Leg_parameter.lenth_list);
            Eigen::Matrix3f jacobian = get_jacobian(sidesign,_joint_pos,Leg_parameter.lenth_list);
            Eigen::Vector3f _foot_vel = get_footpointvel(jacobian,_joint_vel);
            foot_points[i] = _foot_point;
            foot_vel[i] = _foot_vel;
            if(time_index%1000 == 0)
            {
            cout<< "This is jacobian"<<i<<"\n"<<jacobian<<endl;
            cout<< "This is foot_point"<<i<<"\n"<<_foot_point<<endl;
            cout<<"This is foot_vel"<<i<<"\n"<<_foot_vel<<endl;
            }

            if (leg_status[i] == -1)
            {
                for( int j=0;j<3;j++ )
                {  
                    target_mode[3*i+j] = -1;
                }
            }

            else if(leg_status[i] == 0)
            {
                Eigen::Vector3f _Force,_joint_torque;
                _Force<<target_force[3*i+0],target_force[3*i+1],target_force[3*i+2];
                _joint_torque = jacobian.transpose() * _Force;
                for( int j=0;j<3;j++ )
                {  
                    target_mode[3*i+j] = 2;
                    target_value[3*i+j] = _joint_torque.data()[j];
                }
            }

            else if(leg_status[i] == 1)
            {
                double start_time = getCurrentTime() ;
                Eigen::Matrix3f KP,KD;
                KP = Eigen::Matrix3f::Zero();
                KD = Eigen::Matrix3f::Zero();
                std::vector<float> _kp, _kd;
                pnh-> getParam("swingleg_P",_kp);
                pnh-> getParam("swingleg_D",_kd);
                KP(0,0) = _kp[0];
                KP(1,1) = _kp[1];
                KP(2,2) = _kp[2];
                KD(0,0) = _kd[0];
                KD(1,1) = _kd[1];
                KD(2,2) = _kd[2];
            
                Eigen::Vector3f _target_pos, _target_vel ,_joint_torque;
                _target_pos<<target_swing[3*i], target_swing[3*i+1], target_swing[3*i+2];
                _target_vel<<target_swing[3*i + 12], target_swing[3*i+1 + 12], target_swing[3*i+2 + 12];
                _joint_torque = get_tauff(sidesign, _joint_pos, _joint_vel,Leg_parameter);
                Eigen::Vector3f taubf= jacobian * get_feedbackward(KP ,KD , _foot_point, _foot_vel, _target_pos,_target_vel);
               if(time_index %1000 == 0)
               {
                    cout<<"tauff"<<i<<_joint_torque<<endl;
                    cout<<"taubf"<<i<<"     "<<taubf<<KP<<KD<<_foot_point<<_foot_vel<<endl;
               }
               _joint_torque += taubf;
               if(!use_sim)
               {
                   for( int j=0;j<3;j++ )
                   {
                        _joint_torque[j] = _joint_torque[j] + SIGN(_joint_torque[j]) * damping_compensation[j];
                   }
                   
               }
                // cout<<getCurrentTime() - start_time<<endl;
                for( int j=0;j<3;j++ )
                {  
                    target_mode[3*i+j] = 2;
                    target_value[3*i+j] = _joint_torque.data()[j];
                }
            }
            else if(leg_status[i] == 5)
            {
                Eigen::Vector3f _Force,_joint_torque;
                _Force<<target_force[3*i+0],target_force[3*i+1],target_force[3*i+2];
                _joint_torque =  _Force;
                for( int j=0;j<3;j++ )
                {  
                    target_mode[3*i+j] = 0;
                    target_value[3*i+j] = _joint_torque.data()[j];
                }
            }     

            else if(leg_status[i] == 6)
            {
                Eigen::Vector3f _Force,_joint_torque;
                _Force<<target_force[3*i+0],target_force[3*i+1],target_force[3*i+2];
                _joint_torque =  _Force;
                for( int j=0;j<3;j++ )
                {  
                    target_mode[3*i+j] = 2;
                    target_value[3*i+j] = _joint_torque.data()[j];
                }
            }     
        }
    }
    double start_time = getCurrentTime() ;
    jointtarget_publish();
    footpoint_publish();
    if(time_index%1000 == 1)
    {
            cout<<"pub_time"<<getCurrentTime() - start_time<<endl;
    }

}
void leg_controller::jointtarget_publish()
{
    for( int i=0;i<12;i++ )
    {
        jointtarget_pubmsg.data[i] = target_mode[i];
        jointtarget_pubmsg.data[i+12] = target_value[i];
    }
    jointtarget_publisher.publish(jointtarget_pubmsg);
}
void leg_controller::footpoint_publish()
{
    for( int i=0;i<12;i++ )
    {
        footpoint_pubmsg.data[i] =  foot_points[i/3].data()[i%3];
        footvel_pubmsg.data[i] =  foot_vel[i/3].data()[i%3];
    }
    footpoint_publisher.publish(footpoint_pubmsg);
    footvel_publisher.publish(footvel_pubmsg);
}

void leg_controller::visual()
{
    cout<<"joint_pos"<<endl;
    for( int i=0;i<12;i++ )
    {
        cout<< joint_pos[i]<<",";
    }
    cout<<endl;
    cout<<"joint_vel"<<endl;
    for( int i=0;i<12;i++ )
    {
        cout<< joint_vel[i]<<",";
    }
    cout<<endl;
    cout<<"target_force"<<endl;
    for( int i=0;i<12;i++ )
    {
        cout<< target_force[i]<<",";
    }
    cout<<endl;
    cout<<"target_swing"<<endl;
    for( int i=0;i<24;i++ )
    {
        cout<< target_swing[i]<<",";
    }
    cout<<endl;
    cout<<"leg_status"<<endl;
    for( int i=0;i<4;i++ )
    {
        cout<< leg_status[i]<<",";
    }
    cout<<endl;
    cout<<"target_mode"<<endl;
    for( int i=0;i<12;i++ )
    {
        cout<< target_mode[i]<<",";
    }
    cout<<endl;
    cout<<"target_value"<<endl;
    for( int i=0;i<12;i++ )
    {
        cout<< target_value[i]<<",";
    }
    cout<<endl;
}

double getCurrentTime()  
{  
   struct timeval tv;  
   gettimeofday(&tv,NULL);  
   return (double)(tv.tv_sec * 1000000 + tv.tv_usec)/1000000;  
} 