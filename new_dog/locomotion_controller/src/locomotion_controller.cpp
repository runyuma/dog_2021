#include "locomotion_controller.h"
locomotion_controller::locomotion_controller(){}
void locomotion_controller::init()
{
  footpoint_subscriber = pnh->subscribe("/foot_points",10,&locomotion_controller::footpoint_callback,this);
  footvel_subscriber = pnh->subscribe("/foot_vel",10,&locomotion_controller::footvel_callback,this);
  state_estimation_subscriber = pnh->subscribe("/state",10,&locomotion_controller::state_estimation_callback,this);

  force_publisher = pnh->advertise<std_msgs::Float32MultiArray>("/ground_force",10);
  swingleg_publisher = pnh->advertise<std_msgs::Float32MultiArray>("/swing_leg",10);
  leg_status_publisher = pnh->advertise<std_msgs::Int32MultiArray>("/leg_status",10);

  for( int i=0;i<4;i++ )
  {
    status_msg.data.push_back(0);
  }
  pnh->getParam("body_lenth",body_lenth);
  pnh->getParam("body_width",body_width);
  std::vector<int> leg_init(4,1);
  pnh->setParam("leg_enable",leg_init);
}
locomotion_controller::~locomotion_controller(){}




//***************************************************************************************/callback function/***************************************************************************************//
void locomotion_controller::footpoint_callback(const  std_msgs::Float32MultiArray::ConstPtr& msg)
{
  if(_Dog->is_moving)
  {
    for( int i=0;i<4;i++ )
    {
      int Xside_sign = pow(-1,i);
      int Yside_sign = pow(-1,1+i/2);
      _Dog->footpoint(0,i) = msg->data[i*3 + 0] + Xside_sign * body_width ;
      _Dog->footpoint(1,i) = msg->data[i*3 + 1] + Yside_sign * body_lenth;
      _Dog->footpoint(2,i) = msg->data[i*3 + 2];
    }
  }
}
void locomotion_controller::footvel_callback(const  std_msgs::Float32MultiArray::ConstPtr& msg){
  if(_Dog->is_moving)
  {
    for( int i=0;i<4;i++ )
    {
      _Dog->footvel(0,i) = msg->data[i*3 + 0];
      _Dog->footvel(1,i) = msg->data[i*3 + 1];
      _Dog->footvel(2,i) = msg->data[i*3 + 2];
    }
  }
}
void locomotion_controller::state_estimation_callback(const  std_msgs::Float32MultiArray::ConstPtr& msg){
  if(_Dog->is_moving)
  {
    _Dog->rpy<<msg->data[0],msg->data[1],msg->data[2];
    _Dog->body_pos<<msg->data[3],msg->data[4],msg->data[5];
    _Dog->omega<<msg->data[6],msg->data[7],msg->data[8];
    _Dog->body_vel<<msg->data[9],msg->data[10],msg->data[11];
  }
}
//***************************************************************************************/action/***************************************************************************************//
void locomotion_controller::moving_init()
{
  _Dog = new dog_controller();
  pnh->getParam("state_estimation_mode",_Dog->state_estimation_mode);
  pnh->getParam("body_lenth",_Dog->body_lenth);
  pnh->getParam("body_width",_Dog->body_width);
  pnh->setParam("current_gait",0);
  last_rostime =  ros::Time::now();
  _Dog->last_rostime = ros::Time::now();//double ros::Time::now().toSec()
  time_index = 0;//TODO: to bechange because action only manipulatedog
  _Dog->is_moving = 1;
}

void locomotion_controller::moving_func()
{
  ros_time = ros::Time::now();
  loop_time = ros_time.toSec() - last_rostime.toSec();
  last_rostime = ros_time;
  std::cout<<"loop_time"<<loop_time<<std::endl;
}
