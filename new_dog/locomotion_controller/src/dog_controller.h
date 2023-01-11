#ifndef DOG_CONTROLLER_H
#define DOG_CONTROLLER_H
#include<Eigen/Dense>
#include<iostream>
#include <ros/ros.h>
#include"qp_solver.h"
#include"gait_schedular.h"
#include"math.h"
#include<map>
class dog_controller
{
private:
  Eigen::Vector3f g;
  float start_phasetime;
  int state_index;
public:
  float body_mass;
  float body_width;
  float body_lenth;
  float hip_lenth;

  int is_moving = 0;
  int set_schedule = 0;
  int state_estimation_mode;
  std::vector<int> contact_state = {1,1,1,1};
  int USE_RAIBERT_HEURISTIC;
  int use_sim = 0;

  state_machine _statemachine;
  qp_solver _qp_solver;

  Eigen::Vector3f rpy = Eigen::Vector3f::Zero() ;
  Eigen::Vector3f omega = Eigen::Vector3f::Zero();
  Eigen::Vector3f body_pos = Eigen::Vector3f::Zero();
  Eigen::Vector3f body_vel = Eigen::Vector3f::Zero();
  Eigen::Vector3f command_vel = Eigen::Vector3f::Zero();
  Eigen::Vector3f command_omega = Eigen::Vector3f::Zero();
  float command_pitch = 0.0f; // target pitch angle
  float walking_height;
  float foot_height = 0;//the hight of  robot according to the height of foot
  Eigen::Matrix<float,3,4>ground_point = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix<float,3,4>target_groundleg = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix3f target_mat = Eigen::Matrix<float,3,3>::Identity();

  Eigen::Matrix<float,3,4> footpoint = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix<float,3,4> footvel = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix<float,3,4> target_swingpos = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix<float,3,4> target_swingvel = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix<float,3,4> init_SWINGfootpoint = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix3f TF_mat = Eigen::Matrix<float,3,3>::Identity();
  Eigen::Matrix3f posture_mat = Eigen::Matrix<float,3,3>::Identity();
  Eigen::Matrix<float,3,4> force_list = Eigen::Matrix<float,3,4>::Zero();
  int schedualgroundLeg[4] = {1,1,1,1};
  Eigen::Matrix<float,3,4> target_state = Eigen::Matrix<float,3,4>::Zero();
  Eigen::Matrix<float,3,4> last_targetstate = Eigen::Matrix<float,3,4>::Zero();
  std::vector<Eigen::Matrix<float,3,4>> targetstates;
  Eigen::Vector3f target_force,target_torque;

  Eigen::Matrix3f stand_force_p = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f stand_force_D = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f stand_troque_p = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f stand_troque_D = Eigen::Matrix3f::Zero();

  Eigen::Matrix3f trot_force_p = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f trot_force_D = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f trot_troque_p = Eigen::Matrix3f::Zero();
  Eigen::Matrix3f trot_troque_D = Eigen::Matrix3f::Zero();

  int osqp_unsolved_error = 0;
  int fallen_error = 0;/*TODO:// set error */

  ros::Time ros_time,last_rostime;
  double loop_time;
  int last_gait = 0;
  int gait_num = 0;
  dog_controller();
  void get_TFmat();
  void getTargetstate(float t,int n, Eigen::Matrix<float,3,4> last_targetstate = Eigen::Matrix<float,3,4>::Zero());
  void getTarget_Force();
  int statemachine_update();
  void Force_calculation();
  void swingleg_calculation();
  void targetfootpoint_calculation();
  void get_groundpoint();
  void dog_reset();

};
Eigen::Matrix3f get_tfmat(Eigen::Vector3f & _rpy);
#endif // DOG_CONTROLLER_H
