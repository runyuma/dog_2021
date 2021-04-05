#include "locomotion_controller.h"
#define ABS(x) (x>0 ? x : -x)


locomotion_controller::locomotion_controller(){}
void locomotion_controller::init()
{
  _Dog = new dog_controller();
  hz100 = new ros::Rate(100);
  hz1000 = new ros::Rate(1000);
  footpoint_subscriber = pnh->subscribe("/foot_points",10,&locomotion_controller::footpoint_callback,this);
  footvel_subscriber = pnh->subscribe("/foot_vel",10,&locomotion_controller::footvel_callback,this);
  state_estimation_subscriber = pnh->subscribe("/state",10,&locomotion_controller::state_estimation_callback,this);

  force_publisher = pnh->advertise<std_msgs::Float32MultiArray>("/ground_force",10);
  swingleg_publisher = pnh->advertise<std_msgs::Float32MultiArray>("/swing_leg",10);
  leg_status_publisher = pnh->advertise<std_msgs::Int32MultiArray>("/leg_status",10);

  pnh->getParam("body_lenth",_Dog->body_lenth);
  pnh->getParam("body_width",_Dog->body_width);
  pnh->getParam("hip_lenth",_Dog->hip_lenth);
  pnh->getParam("walking_height",_Dog->walking_height);
  pnh->getParam("walking_height",_Dog->walking_height);
  pnh->getParam("use_sim",_Dog->use_sim);
  pnh->getParam("total_weight",_Dog->body_mass);


  std::vector<int> leg_init(4,1);
  pnh->setParam("leg_enable",leg_init);

  std::vector<float> stand_force_p,stand_force_D,stand_troque_p,stand_troque_D,trot_force_p,trot_force_D,trot_troque_p,trot_troque_D;
  pnh->getParam("stand_force_p",stand_force_p);
  pnh->getParam("stand_force_D",stand_force_D);
  pnh->getParam("stand_troque_p",stand_troque_p);
  pnh->getParam("stand_troque_D",stand_troque_D);

  pnh->getParam("trot_force_p",trot_force_p);
  pnh->getParam("trot_force_D",trot_force_D);
  pnh->getParam("trot_troque_p",trot_troque_p);
  pnh->getParam("trot_troque_D",trot_troque_D);

  _Dog->stand_force_p(0,0) = stand_force_p[0];
  _Dog->stand_force_p(1,1) = stand_force_p[1];
  _Dog->stand_force_p(2,2) = stand_force_p[2];
  _Dog->stand_force_D(0,0) = stand_force_D[0];
  _Dog->stand_force_D(1,1) = stand_force_D[1];
  _Dog->stand_force_D(2,2) = stand_force_D[2];
  _Dog->stand_troque_p(0,0) = stand_troque_p[0];
  _Dog->stand_troque_p(1,1) = stand_troque_p[1];
  _Dog->stand_troque_p(2,2) = stand_troque_p[2];
  _Dog->stand_troque_D(0,0) = stand_troque_D[0];
  _Dog->stand_troque_D(1,1) = stand_troque_D[1];
  _Dog->stand_troque_D(2,2) = stand_troque_D[2];

  _Dog->trot_force_p(0,0) = trot_force_p[0];
  _Dog->trot_force_p(1,1) = trot_force_p[1];
  _Dog->trot_force_p(2,2) = trot_force_p[2];
  _Dog->trot_force_D(0,0) = trot_force_D[0];
  _Dog->trot_force_D(1,1) = trot_force_D[1];
  _Dog->trot_force_D(2,2) = trot_force_D[2];
  _Dog->trot_troque_p(0,0) = trot_troque_p[0];
  _Dog->trot_troque_p(1,1) = trot_troque_p[1];
  _Dog->trot_troque_p(2,2) = trot_troque_p[2];
  _Dog->trot_troque_D(0,0) = trot_troque_D[0];
  _Dog->trot_troque_D(1,1) = trot_troque_D[1];
  _Dog->trot_troque_D(2,2) = trot_troque_D[2];

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
    if(ABS(_Dog->rpy(1))>=0.2)
    {
      pnh ->setParam("fallen_error",1);
    }
    _Dog->body_pos<<msg->data[3],msg->data[4],msg->data[5];
    _Dog->omega<<msg->data[6],msg->data[7],msg->data[8];
    _Dog->body_vel<<msg->data[9],msg->data[10],msg->data[11];
  }
}


//***************************************************************************************/publish/***************************************************************************************//
void locomotion_controller::status_publish()
{
  std_msgs::Int32MultiArray status_msg;
  status_msg.data = {1,1,1,1};
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
    std::vector<int> _scheduleleg(4);
    for (int i = 0;i<4;i++) {
      _scheduleleg[i] = _Dog->schedualgroundLeg[i];
    }
    pnh->setParam("schedule_groundleg", _scheduleleg);
    _Dog->set_schedule = 0;
  }
}

void locomotion_controller::set_error()
{
  if(_Dog->osqp_unsolved_error)
  {
    pnh->setParam("osqp_unsolve_error", 1);
    _Dog->osqp_unsolved_error = 0;
  }
  if(_Dog->fallen_error)
  {
    pnh->setParam("fallen_error", 1);
    _Dog->fallen_error = 0;
  }
}

void locomotion_controller::force_publish()
{
  std_msgs::Float32MultiArray groundforce_msg;
  groundforce_msg.data = {0,0,0,0,0,0,0,0,0,0,0,0,};
  for (int i = 0;i<4;i++) {
    if(_Dog->schedualgroundLeg[i] == 1)
    {
      groundforce_msg.data[3*i] = -_Dog->force_list(0,i);
      groundforce_msg.data[3*i + 1] = -_Dog->force_list(1,i);
      groundforce_msg.data[3*i + 2] = -_Dog->force_list(2,i);
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
  if(time_index%100 == 0)
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
    std::cout<<"target_force/Torque: "<<_Dog->target_force<<std::endl<<_Dog->target_torque<<std::endl;
    std::cout<<"force_list: "<<_Dog->force_list<<std::endl;
    std::cout<<"swing_pos: "<<_Dog->target_swingpos<<std::endl;
    std::cout<<"swing_vel: "<<_Dog->target_swingvel<<std::endl;
  }
}



//***************************************************************************************/action/***************************************************************************************//
void locomotion_controller::moving_init()
{
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
//  std::cout<<"loop_time"<<loop_time<<std::endl;
  _Dog->ros_time = ros_time;
  float _vel,_omega;
  pnh->getParam("command_vel",_vel);
  pnh->getParam("command_omega",_omega);
  pnh->getParam("contact_state",_Dog->contact_state);
  _Dog->command_vel<<0,_vel,0;
  _Dog->command_omega<<0,0,_omega;

  pnh->getParam("current_gait",_Dog->gait_num);
  int changed = _Dog->statemachine_update();
  _Dog->get_TFmat();
  if(time_index%1 == 0 or changed){
    _Dog->Force_calculation();
    force_publish();
  }
  _Dog->swingleg_calculation();

  std::string str_moving= "moving";
  pnh->setParam("dog_action",str_moving);
  swing_publoish();
  set_schedulegroundleg();
  set_error();
  status_publish();
  time_index += 1;
  visual();
  hz1000->sleep();
}

bool locomotion_controller::shrink(int start_index,Eigen::Vector3f rpy)
{
  std::vector<float> joint_pos_target = {0,1.2,-2.4};
  Eigen::Matrix<float,3,4> target_footpoint;
  target_footpoint<< 0,0,0,0,
                    0,0,0,0,
                    -0.1,-0.1,-0.1,-0.1;
  std::vector<int> _scheduleleg = {0,0,0,0};
  for (int i = 0;i<4;i++) {
    _scheduleleg[i] = _Dog->schedualgroundLeg[i];
  }
  pnh->setParam("schedule_groundleg", _scheduleleg);
  if(time_index - start_index <= 20 )
  {
    std_msgs::Int32MultiArray status_msg;
    status_msg.data = {1,1,1,1};
    leg_status_publisher.publish(status_msg);
    std_msgs::Float32MultiArray swingleg_msg;
    swingleg_msg.data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};
    for (int i = 0;i<4;i++) {

      swingleg_msg.data[3*i] = target_footpoint(0,i);
      swingleg_msg.data[3*i + 1] = target_footpoint(1,i);
      swingleg_msg.data[3*i + 2] = target_footpoint(2,i);


      swingleg_msg.data[3*i + 12] = 0;
      swingleg_msg.data[3*i + 13] = 0;
      swingleg_msg.data[3*i + 14] = 0;
      }
    swingleg_publisher.publish(swingleg_msg);
    time_index += 1;
    hz100->sleep();
    return 0;
   }
  else if(time_index - start_index < 200) {
    std_msgs::Int32MultiArray status_msg;
    status_msg.data = {5,5,5,5};
    leg_status_publisher.publish(status_msg);
    std_msgs::Float32MultiArray groundforce_msg;
    groundforce_msg.data = {0,0,0,0,0,0,0,0,0,0,0,0,};
    for (int i = 0;i<4;i++) {
      groundforce_msg.data[3*i] = joint_pos_target[0];
      groundforce_msg.data[3*i+1] = joint_pos_target[1];
      groundforce_msg.data[3*i+2] = joint_pos_target[2];
    }
    force_publisher.publish(groundforce_msg);
    time_index += 1;
    hz100->sleep();
    return 0;
  }
  else if(time_index - start_index == 300)
  {
    time_index += 1;
    hz100->sleep();
    if(ABS(rpy(0)<0.1) and ABS(rpy(1))<0.1)
    {
     return 1;
    }


  }
}

bool locomotion_controller::error_handle()
{
  int osqp_error=0;
  int fallen_error = 0;
  pnh->getParam("osqp_unsolve_error",osqp_error);
  pnh->getParam("fallen_error",fallen_error);
  if(osqp_error or fallen_error)
  {
    pnh->setParam("start_move", 0);
    return 1;
  }
  else {
    return 0;
  }
  //TODO different ways
}
void locomotion_controller::moving_reset()
{
  _Dog->dog_reset();
  pnh->setParam("move_reset", 1);//TODO: not reset
  pnh->setParam("osqp_unsolve_error",0);
  pnh->setParam("fallen_error",0);
}
void locomotion_controller::idle()
{
  hz100->sleep();
  last_rostime =  ros::Time::now();
  _Dog->last_rostime = ros::Time::now();
}
//***************************************************************************************//***************************************************************************************//
void locomotion_controller::cout_matrix(std::string strings, Eigen::MatrixXf matrix)
{
  std::cout<<strings<<": "<<std::endl;
  int rows = matrix.rows();
  int cols = matrix.cols();
  for (int row = 0;row < rows;row ++) {
    for (int col = 0;col < cols;col++) {
      std::cout<<matrix(col,row)<<" ";
    }
    std::cout<<std::endl;
  }
}
