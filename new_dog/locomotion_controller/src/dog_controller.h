#ifndef DOG_CONTROLLER_H
#define DOG_CONTROLLER_H
#include<Eigen/Dense>
#include<iostream>
#include <ros/ros.h>
#include"qp_solver.h"
#include"gait_schedular.h"
#include"math.h"
class dog_controller
{
private:
  Eigen::Vector3f g;
public:
  float body_mass = 10 + (0.3+0.6+0.5)* 4 ;
  float body_width;
  float body_lenth;

  int is_moving = 0;

  int state_estimation_mode;

  state_machine _state_machine;
  qp_solver _qp_solver;

  Eigen::Vector3f rpy = Eigen::Vector3f::Zero() ;
  Eigen::Vector3f omega = Eigen::Vector3f::Zero();
  Eigen::Vector3f body_pos = Eigen::Vector3f::Zero();
  Eigen::Vector3f body_vel = Eigen::Vector3f::Zero();
  Eigen::Vector3f command_vel = Eigen::Vector3f::Zero();
  Eigen::Vector3f command_omega = Eigen::Vector3f::Zero();

  Eigen::Matrix<float,3,4> footpoint = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix<float,3,4> footvel = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix<float,3,4> target_swingpos = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix<float,3,4> target_swingvel = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix<float,3,4> init_SWINGfootpoint = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix3f TF_mat = Eigen::Matrix<float,3,3>::Identity();
  int schedualgroundLeg[4] = {1,1,1,1};
  Eigen::Matrix<float,3,4> target_state = Eigen::Matrix<float,3,4>::Zero();
  std::vector<Eigen::Matrix<float,3,4>> targetstates;
  Eigen::Vector3f target_force,target_torque;

  ros::Time ros_time,last_rostime;
  double loop_time;
  int last_gait = 0;
  int gait_num = 0;
  dog_controller();
  void get_TFmat();
  void getTargetstate(float t,int n, Eigen::Matrix<float,3,4> last_targetstate = Eigen::Matrix<float,3,4>::Zero());
  void getTarget_Force();
  void statemachine_update();

};

#endif // DOG_CONTROLLER_H
