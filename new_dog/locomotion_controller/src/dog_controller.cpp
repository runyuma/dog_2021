#include "dog_controller.h"
#define DEFAULT_HEIGHT 0.32f
#define ABS(x) (x>0 ? x : -x)
#define SIGN(x) (x>0 ? 1 : -1)
#define MIN(a,b) (a>b ? b : a)
#define PI 3.14159f
#define USE_RAIBERT_HEURISTIC 0
dog_controller::dog_controller()
{
  g<<0,0,-9.81;
}

void dog_controller::get_TFmat()
{
  Eigen::Matrix3f matr_y,matr_x,matr_z;

  matr_y<<cos(rpy(1)),0,sin(rpy(1)),
          0,1,0,
          -sin(rpy(1)),0,cos(rpy(1));
  matr_x<<1,0,0,
          0,cos(rpy(0)),-sin(rpy(0)),
          0,sin(rpy(0)),cos(rpy(0));
  matr_z<<cos(rpy(2)),-sin(rpy(2)),0,
          sin(rpy(2)),cos(rpy(2)),0,
          0,0,1;
  TF_mat = matr_z*matr_y*matr_x;

}

void dog_controller::getTargetstate(float t, int n, Eigen::Matrix<float, 3, 4> last_targetstate)
{
  std::vector<Eigen::Matrix<float,3,4>> states(n);
  if(_state_machine._gait.name == "STANDING")
  {
    if(DEFAULT_HEIGHT - body_pos(2)>= 0.05)
    {
      for (int i;i<n;i++) {
        Eigen::Matrix<float,3,4> _state;
        Eigen::Vector3f target_rpy =  Eigen::Vector3f::Zero();
        target_rpy(2) = rpy(2);
        Eigen::Vector3f target_omega =  Eigen::Vector3f::Zero();
        Eigen::Vector3f target_pos = body_pos;
        Eigen::Vector3f target_vel = Eigen::Vector3f::Zero();
        if(i<= n/2){
          target_vel(2) = 4*(DEFAULT_HEIGHT - body_pos(2)) * i /(n*n*t);
          target_pos(2) = 2*(DEFAULT_HEIGHT - body_pos(2)) *i* i /(n*n) + body_pos(2);
        }
        else {
          target_vel(2) = 4*(DEFAULT_HEIGHT - body_pos(2)) * (n - i) /(n*n*t);
          target_pos(2) =  - 2*(DEFAULT_HEIGHT - body_pos(2)) *(n-i)* (n-i) /(n*n) + DEFAULT_HEIGHT;
        }
        _state.block(0,0,3,1) = target_rpy;
        _state.block(0,1,3,1) = target_pos;
        _state.block(0,2,3,1) = target_omega;
        _state.block(0,3,3,1) = target_vel;
        states[i] = _state;
      }
      targetstates = states;
    }
    else {
      for (int i;i<n;i++)
      {
        Eigen::Matrix<float,3,4> _state;
        Eigen::Vector3f target_rpy =  Eigen::Vector3f::Zero();
        Eigen::Vector3f target_omega =  Eigen::Vector3f::Zero();
        Eigen::Vector3f target_vel = Eigen::Vector3f::Zero();
        Eigen::Vector3f target_pos;
        if(last_targetstate == Eigen::Matrix<float,3,4>::Zero())
        {
          target_rpy(2) = rpy(2);
          target_pos = body_pos;
          target_pos(2) = DEFAULT_HEIGHT;
        }
        else {
          target_rpy(2) = last_targetstate(2,0);
          target_pos = last_targetstate.block(0,1,3,1);
          target_pos(2) = DEFAULT_HEIGHT;
        }
        _state.block(0,0,3,1) = target_rpy;
        _state.block(0,1,3,1) = target_pos;
        _state.block(0,2,3,1) = target_omega;
        _state.block(0,3,3,1) = target_vel;
        states[i] = _state;
      }
     targetstates = states;

    }
  }
  else if(_state_machine._gait.name == "TROTING_WALKING" or _state_machine._gait.name == "TROTING_RUNING" or _state_machine._gait.name == "SLOW_WALKING")
  {
    Eigen::Vector3f current_vel = TF_mat.inverse() * body_vel;
    float vel_diff = command_vel(1) - current_vel(1);
    Eigen::Vector3f _rpy = last_targetstate.block(0,0,3,1);
    Eigen::Matrix3f matr_y,matr_x,matr_z;
    matr_y<<cos(_rpy(1)),0,sin(_rpy(1)),
            0,1,0,
            -sin(_rpy(1)),0,cos(_rpy(1));
    matr_x<<1,0,0,
            0,cos(_rpy(0)),-sin(_rpy(0)),
            0,sin(_rpy(0)),cos(_rpy(0));
    matr_z<<cos(_rpy(2)),-sin(_rpy(2)),0,
            sin(_rpy(2)),cos(_rpy(2)),0,
            0,0,1;
    Eigen::Matrix3f target_TFmat = matr_z * matr_y * matr_x;
    if(ABS(vel_diff)>0.3)
    {
      float ay = SIGN(vel_diff) * MIN(ABS(vel_diff)/(n*t),0.8);
      for (int i;i<n;i++)
      {
        Eigen::Matrix<float,3,4> _state;
        Eigen::Vector3f target_rpy =  last_targetstate.block(0,0,3,1) + t * i * command_omega;
        Eigen::Vector3f target_omega =  command_omega;
        Eigen::Vector3f target_vel;
        Eigen::Vector3f target_pos;
        float vy = (TF_mat.inverse() * last_targetstate.block(0,2,3,1))(1) + ay * i *t;
        float dy = (TF_mat.inverse() * last_targetstate.block(0,2,3,1))(1) * i * t + ay * i * i * t*t/2;
        Eigen::Vector3f dy_Vec,vy_Vec;
        dy_Vec<<0,dy,0;
        vy_Vec<<0,vy,0;
        target_pos = last_targetstate.block(0,1,3,1) * target_TFmat * dy_Vec;
        if(ABS(target_rpy(2))>PI)
        {
          target_rpy(2) += -SIGN(target_rpy(2)) * 2 * PI;
        }
        target_vel = target_TFmat * vy_Vec;
        _state.block(0,0,3,1) = target_rpy;
        _state.block(0,1,3,1) = target_pos;
        _state.block(0,2,3,1) = target_omega;
        _state.block(0,3,3,1) = target_vel;
        states[i] = _state;
      }
      targetstates = states;
    }
    else{
      for (int i;i<n;i++)
      {
        Eigen::Matrix<float,3,4> _state;
        float dy =  t * i * command_vel(1);
        Eigen::Vector3f dy_Vec;
        dy_Vec<<0,dy,0;
        Eigen::Vector3f target_rpy =  last_targetstate.block(0,0,3,1) + t * i * command_omega;
        Eigen::Vector3f target_omega =  command_omega;
        Eigen::Vector3f target_vel = TF_mat * command_vel;
        Eigen::Vector3f target_pos = last_targetstate.block(0,1,3,1) + target_TFmat * dy_Vec;
        if(ABS(target_rpy(2))>PI)
        {
          target_rpy(2) += -SIGN(target_rpy(2)) * 2 * PI;
        }
        _state.block(0,0,3,1) = target_rpy;
        _state.block(0,1,3,1) = target_pos;
        _state.block(0,2,3,1) = target_omega;
        _state.block(0,3,3,1) = target_vel;
        states[i] = _state;
      }
    }
  }
}

void dog_controller::getTarget_Force()
{
  Eigen::Vector3f target_rpy = target_state.block(0,0,3,1);
  Eigen::Vector3f target_pos = target_state.block(0,1,3,1);
  Eigen::Vector3f target_omega = target_state.block(0,2,3,1);
  Eigen::Vector3f target_vel = target_state.block(0,3,3,1);

  Eigen::Matrix3f Force_KP,Force_KD,Torque_KP,Torque_KD;
  Eigen::Vector3f Force_limit,Torque_limit;
  Force_limit<<50,50,200;
  Torque_limit<<20,30,30;
  if(_state_machine._gait.name == "STANDING")
  {
    Force_KP<<2500,0,0,0,800,0,0,0,800;
    Force_KD<<600,0,0,0,350,0,0,0,150;
    Torque_KP<<200,0,0,0,300,0,0,0,300;
    Torque_KD<<30,0,0,0,12,0,0,0,50;
  }
  else if (_state_machine._gait.name == "TROTING_WALKING" or _state_machine._gait.name == "TROTING_RUNING") {
    Force_KP<<400,0,0,0,450,0,0,0,600;
    Force_KD<<250,0,0,0,100,0,0,0,120;
    Torque_KP<<400,0,0,0,600,0,0,0,500;
    Torque_KD<<50,0,0,0,50,0,0,0,50;
  }
  else if (_state_machine._gait.name == "SLOW_WALKING") {
    Force_KP<<500,0,0,0,800,0,0,0,800;
    Force_KD<<300,0,0,0,350,0,0,0,150;
    Torque_KP<<400,0,0,0,600,0,0,0,600;
    Torque_KD<<50,0,0,0,65,0,0,0,50;
  }

  target_force = Force_KP * (target_pos - body_pos) + Force_KD * (target_vel - body_vel) - body_mass * g;
  target_force = TF_mat.inverse() * target_force;
  if(ABS(rpy(2))>= PI/2 and ABS(target_rpy(2))>= PI and rpy(2) * target_rpy(2) < 0)
  {
    target_rpy(2) += SIGN(target_rpy(2)) * 2 * PI ;//TODO:somthing wrong
  }
  target_torque = Torque_KP * (target_rpy - rpy) + Torque_KD * (target_omega - omega);
  for (int i = 0;i<3;i++) {
    if(ABS(target_force(i))>= Force_limit(i)){target_force(i) = SIGN(target_force(i)) * Force_limit(i);}
      if(ABS(target_torque(i))>= Torque_limit(i)){target_torque(i) = SIGN(target_torque(i)) * Torque_limit(i);}
  }



}

void dog_controller::statemachine_update()
{

}


