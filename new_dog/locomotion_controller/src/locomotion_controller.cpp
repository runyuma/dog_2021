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

  pnh->getParam("body_lenth",_Dog->body_lenth);
  pnh->getParam("body_width",_Dog->body_width);
  pnh->getParam("hip_lenth",_Dog->hip_lenth);

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
      _Dog->footpoint(0,i) = msg->data[i*3 + 0] + Xside_sign * _Dog->body_width ;
      _Dog->footpoint(1,i) = msg->data[i*3 + 1] + Yside_sign * _Dog->body_lenth;
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


//***************************************************************************************/publish/***************************************************************************************//
void locomotion_controller::status_publish()
{
  std_msgs::Int32MultiArray status_msg;
  for (int i = 0;i<4;i++) {
    if(_Dog->schedualgroundLeg[i] == 1)
    {
      status_msg.data[i] = 0;
    }
    else {
      status_msg.data[i]= 1;
    }
  }
  leg_status_publisher.publish(status_msg);
}

void locomotion_controller::set_schedulegroundleg()
{
  if(_Dog->set_schedule)
  {
    std::vector<int> _scheduleleg;
    for (int i = 0;i<4;i++) {
      _scheduleleg[i] = _Dog->schedualgroundLeg[i];
    }
    pnh->setParam("schedule_groundleg", _scheduleleg);
    _Dog->set_schedule = 0;
  }
}

void locomotion_controller::force_publish()
{
  std_msgs::Float32MultiArray groundforce_msg;
  groundforce_msg.data = {0,0,0,0,0,0,0,0,0,0,0,0,};
  for (int i = 0;i<4;i++) {
    if(_Dog->schedualgroundLeg[i] == 1)
    {
      groundforce_msg.data[3*i] = _Dog->force_list(0,i);
      groundforce_msg.data[3*i + 1] = _Dog->force_list(1,i);
      groundforce_msg.data[3*i + 2] = _Dog->force_list(2,i);
    }
    else {
      groundforce_msg.data[3*i] = 0;
      groundforce_msg.data[3*i + 1] = 0;
      groundforce_msg.data[3*i + 2] = 0;
    }
  }
  force_publisher.publish(groundforce_msg);
}

void locomotion_controller::swing_publoish()
{
  std_msgs::Float32MultiArray swingleg_msg;
  swingleg_msg.data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};
  for (int i = 0;i<4;i++) {
    int Xside_sign = pow(-1,i);
    int Yside_sign = pow(-1,1+i/2);
    if(_Dog->schedualgroundLeg[i] == 0)
    {
      swingleg_msg.data[3*i] = _Dog->target_swingpos(0,i) - Xside_sign * _Dog->body_width;
      swingleg_msg.data[3*i + 1] = _Dog->target_swingpos(1,i) - Yside_sign * _Dog->body_lenth;
      swingleg_msg.data[3*i + 2] = _Dog->target_swingpos(2,i);
      swingleg_msg.data[3*i + 12] = _Dog->target_swingvel(0,i);
      swingleg_msg.data[3*i + 13] = _Dog->target_swingvel(1,i);
      swingleg_msg.data[3*i + 14] = _Dog->target_swingvel(2,i);
    }
    else {
      swingleg_msg.data[3*i] = 0;
      swingleg_msg.data[3*i + 1] = 0;
      swingleg_msg.data[3*i + 2] = 0;
      swingleg_msg.data[3*i + 12] = 0;
      swingleg_msg.data[3*i + 13] = 0;
      swingleg_msg.data[3*i + 14] = 0;
    }
  }
  swingleg_publisher.publish(swingleg_msg);
}
//***************************************************************************************/visualize/***************************************************************************************//
void locomotion_controller::visual()
{
  if(time_index%10 == 0)
  {
    std::cout<<"foot_point: "<<_Dog->footpoint<<std::endl;
    std::cout<<"foot_vel: "<<_Dog->footvel<<std::endl;
    std::cout<<"rpy: "<<_Dog->rpy<<std::endl;
    std::cout<<"xyz: "<<_Dog->body_pos<<std::endl;
    std::cout<<"omega: "<<_Dog->omega<<std::endl;
    std::cout<<"vel: "<<_Dog->body_vel<<std::endl;
    std::cout<<"schedualgroundLeg: "<<std::endl;
    for (int i = 0;i<4;i++) {std::cout<<_Dog->schedualgroundLeg[i]<<" ";}
    std::cout<<std::endl;
    std::cout<<"phase: "<<std::endl;
    for (int i = 0;i<4;i++) {std::cout<<_Dog->_statemachine.phase[i]<<" ";}
    std::cout<<std::endl;
    std::cout<<"target_state: "<<_Dog->target_state<<std::endl;
    std::cout<<"gait_time: "<<_Dog->_statemachine._gait.Gait_currentTime<<std::endl;
    std::cout<<"loop_time: "<<_Dog->loop_time<<std::endl;
    std::cout<<"target_force/Torque: "<<_Dog->target_force<<_Dog->target_torque<<std::endl;
    std::cout<<"force_list: "<<_Dog->force_list<<std::endl;
    std::cout<<"swing_pos: "<<_Dog->target_swingpos<<std::endl;
    std::cout<<"swing_vel: "<<_Dog->target_swingvel<<std::endl;
  }
}



//***************************************************************************************/action/***************************************************************************************//
void locomotion_controller::moving_init()
{
  _Dog = new dog_controller();
  pnh->getParam("state_estimation_mode",_Dog->state_estimation_mode);
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
  _Dog->ros_time = ros_time;
  float _vel,_omega;
  pnh->getParam("command_vel",_vel);
  pnh->getParam("command_omega",_omega);
  _Dog->command_vel<<0,_vel,0;
  _Dog->command_omega<<0,0,_omega;
  pnh->getParam("current_gait",_Dog->gait_num);
  _Dog->statemachine_update();
  _Dog->get_TFmat();
  if(time_index%8 == 0){
    _Dog->Force_calculation();
    force_publish();
  }
  _Dog->swingleg_calculation();
  swing_publoish();
  set_schedulegroundleg();
  status_publish();
  time_index += 1;
  visual();

}


