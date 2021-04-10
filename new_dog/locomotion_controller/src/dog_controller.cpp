#include "dog_controller.h"
#define DEFAULT_HEIGHT 0.32f
#define ABS(x) (x>0 ? x : -x)
#define SIGN(x) (x>0 ? 1 : -1)
#define MIN(a,b) (a>b ? b : a)
#define PI 3.14159f


std::map<int,std::string> gait_map= {
  { 0,"STANDING" },
  { 1,"TROTING_RUNING",}};
void dog_controller::dog_reset()
{
  USE_RAIBERT_HEURISTIC = 0;
  is_moving = 0;
  set_schedule = 0;
  contact_state = {1,1,1,1};
  command_vel = Eigen::Vector3f::Zero();
  command_omega = Eigen::Vector3f::Zero();
  target_swingpos = Eigen::Matrix<float,3,4>::Zero();
  target_swingvel = Eigen::Matrix<float,3,4>::Zero();
  init_SWINGfootpoint = Eigen::Matrix<float,3,4>::Zero();
  TF_mat = Eigen::Matrix<float,3,3>::Identity();
  force_list = Eigen::Matrix<float,3,4>::Zero();
  last_gait = 0;
  gait_num = 0;

  int osqp_unsolved_error = 0;
  int fallen_error = 0;
  //error reset

  _statemachine._gait=gait_schedular(gait_map[0]);
  _statemachine.phase = {1,1,1,1};
  //statemachine&phase reset
  is_moving = 1;
}


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
  if(_statemachine._gait.name == "STANDING")
  {
    if(DEFAULT_HEIGHT - body_pos(2)>= 0.05)
    {
      for (int i=0;i<n;i++) {
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
      for (int i=0;i<n;i++)
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
  else if(_statemachine._gait.name == "TROTING_WALKING" or _statemachine._gait.name == "TROTING_RUNING" or _statemachine._gait.name == "SLOW_WALKING")
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
      for (int i=0;i<n;i++)
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
        target_pos = last_targetstate.block(0,1,3,1) + target_TFmat * dy_Vec;
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
      for (int i=0;i<n;i++)
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
      targetstates = states;
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
  if(_statemachine._gait.name == "STANDING")
  {
//    Force_KP<<2500,0,0,0,800,0,0,0,800;
//    Force_KD<<600,0,0,0,350,0,0,0,150;
//    Torque_KP<<200,0,0,0,300,0,0,0,300;
//    Torque_KD<<30,0,0,0,12,0,0,0,50;
      Force_KP = stand_force_p;
      Force_KD = stand_force_D;
      Torque_KP = stand_troque_p;
      Torque_KD = stand_troque_D;
  }
  else if (_statemachine._gait.name == "TROTING_WALKING" or _statemachine._gait.name == "TROTING_RUNING") {
//    Force_KP<<400,0,0,0,450,0,0,0,600;
//    Force_KD<<250,0,0,0,100,0,0,0,120;
//    Torque_KP<<400,0,0,0,600,0,0,0,500;
//    Torque_KD<<50,0,0,0,50,0,0,0,50;
    Force_KP = trot_force_p;
    Force_KD = trot_force_D;
    Torque_KP = trot_troque_p;
    Torque_KD = trot_troque_D;
    if( USE_RAIBERT_HEURISTIC)
    {
          Force_KP<<50,0,0,0,50,0,0,0,600;
          Force_KD<<250,0,0,0,100,0,0,0,120;
          Torque_KP<<400,0,0,0,600,0,0,0,500;
          Torque_KD<<50,0,0,0,50,0,0,0,50;
    }
  }
  else if (_statemachine._gait.name == "SLOW_WALKING") {
    Force_KP<<500,0,0,0,800,0,0,0,800;
    Force_KD<<300,0,0,0,350,0,0,0,150;
    Torque_KP<<400,0,0,0,600,0,0,0,600;
    Torque_KD<<50,0,0,0,65,0,0,0,50;
  }
  int USE_BODYHEIGHT = 0;
  Eigen::Matrix3f _Force_KP = Force_KP;
  Eigen::Matrix3f _Force_KD = Force_KD;
  if(state_estimation_mode == 1)
  {
    _Force_KP(2,2) = 0;
    _Force_KD(2,2) =  _Force_KD(2,2);
    float foot_height = 0;
    int num = 0;
    for (int i = 0;i<4;i++) {
      if (schedualgroundLeg[i] == 1)
      {
        foot_height += footpoint(2,i);
        num += 1;
      }
    }
    foot_height = - foot_height/num;
//    std::cout<<foot_height<<std::endl;
    float z_force = Force_KP(2,2)*(walking_height - foot_height);

    target_force = _Force_KP * TF_mat.inverse() * (target_pos - body_pos) + _Force_KD * TF_mat.inverse() * (target_vel - body_vel) - TF_mat.inverse() *body_mass * g;
    target_force(2,0) = target_force(2,0) + z_force;
    cout<<"height"<<foot_height<<"  zforce  "<<z_force<<"  "<<Force_KD * TF_mat.inverse() * (target_vel - body_vel)<<std::endl<<"  "<<TF_mat.inverse() *body_mass * g<<std::endl;
  }
  else {
    target_force = _Force_KP * TF_mat.inverse() * (target_pos - body_pos) + Force_KD * TF_mat.inverse() * (target_vel - body_vel) - TF_mat.inverse() *body_mass * g;
  }
  if(ABS(rpy(2))>= PI/2 and ABS(target_rpy(2))>= PI and rpy(2) * target_rpy(2) < 0)
  {
    target_rpy(2) += SIGN(target_rpy(2)) * 2 * PI ;//TODO:somthing wrong
  }
  target_torque = Torque_KP * (target_rpy - rpy) + Torque_KD * (target_omega - omega);
//  cout<<"pos error : "<<std::endl<<target_pos - body_pos<<std::endl;
//  cout<<"rpy error : "<<std::endl<<target_rpy - rpy<<std::endl;

  for (int i = 0;i<3;i++) {
    if(ABS(target_force(i))>= Force_limit(i)){target_force(i) = SIGN(target_force(i)) * Force_limit(i);}
    if(ABS(target_torque(i))>= Torque_limit(i)){target_torque(i) = SIGN(target_torque(i)) * Torque_limit(i);}
  }


}

int dog_controller::statemachine_update()
{
  int _gait_index = _statemachine._gait.Gait_index;
  float _gait_time = _statemachine._gait.Gait_time[_gait_index];


  if(_statemachine._gait.Gait_currentTime == 0)
  {
    start_phasetime = ros_time.toSec();
    _statemachine._gait.get_schedualgroundLeg();
    memcpy(schedualgroundLeg,_statemachine._gait.schedualgroundLeg,4*sizeof (int));
    _statemachine.phase = _statemachine._gait.Gait_phase[_gait_index];

    getTargetstate(0.01,int(_gait_time/0.01),last_targetstate);
    int last_index = _statemachine._gait.get_lastindex();
    for (int i=0;i<4;i++) {
      if(_statemachine._gait.Gait_phase[_gait_index][i]!= 1 and _statemachine._gait.Gait_phase[last_index][i] == 1)
      {
        init_SWINGfootpoint.block(0,i,3,1) = footpoint.block(0,i,3,1);
      }
    }
  }
  loop_time = (ros_time-last_rostime).toSec();
  _statemachine._gait.gaittime_update((ros_time-last_rostime).toSec());

  state_index = int(_statemachine._gait.Gait_currentTime/0.01);
  if(state_index>= _gait_time/0.01)
  {
    state_index = int(_gait_time/0.01) - 1;
  }

  int next_index = _statemachine._gait.get_nextindex();
  for (int i = 0;i<4;i++) {
    if(schedualgroundLeg[i] != 1)
    {
      float phase_diff = _statemachine._gait.Gait_phase[next_index][i] - _statemachine._gait.Gait_phase[_gait_index][i];
      _statemachine.phase[i] += (ros_time - last_rostime).toSec() * phase_diff/_gait_time;
//      std::cout<<"phase: "<<std::endl;
//      for (int i = 0;i<4;i++) {std::cout<<_statemachine.phase[i]<<" ";}
//      std::cout<<std::endl;
    }
  }
  int changed = 0;
  if(_statemachine._gait.Gait_currentTime >= _gait_time)
  {
    int _permit = 1;
//    for(int i = 0;i<4;i++) {
//      if(_statemachine._gait.Gait_phase[next_index][i] == 1 and _statemachine._gait.Gait_phase[_gait_index][i] != 1){_permit = 0;}
//    }
    if(_permit or _statemachine._gait.Gait_currentTime >= 1.2 * _gait_time)
    {
      changed = 1;
      _statemachine._gait.Gait_index = _statemachine._gait.get_nextindex();
      _statemachine.phase = _statemachine._gait.Gait_phase[_statemachine._gait.Gait_index];
      _statemachine._gait.Gait_currentTime = 0;
      if(_statemachine._gait.name == "STANDING")
      {
        last_targetstate = targetstates[state_index];
//        last_targetstate(0,2) = rpy(2);
//        last_targetstate.block(0,1,3,1) = body_pos;
//        last_targetstate(2,2) = omega(2);
//        last_targetstate.block(0,3,3,1) = body_vel;

      }
      else if(_statemachine._gait.name == "TROTING_WALKING" or _statemachine._gait.name == "TROTING_RUNING"){
        if(not USE_RAIBERT_HEURISTIC)
        {
          last_targetstate = targetstates[state_index];
        }
        else {
          last_targetstate.block(0,0,3,1) = rpy;
          last_targetstate.block(0,1,3,1) = body_pos;
          last_targetstate.block(0,2,3,1) = omega;
          last_targetstate.block(0,3,3,1) = body_vel;
        }
      }

      if(gait_num != last_gait)
      {
        _statemachine._gait=gait_schedular(gait_map[gait_num]);
      }
      last_gait = gait_num;
      _statemachine._gait.get_schedualgroundLeg();
      memcpy(schedualgroundLeg,_statemachine._gait.schedualgroundLeg,4*sizeof (int));
    }
  }
  set_schedule = 1;
  last_rostime = ros_time;
  return changed;
}

void dog_controller::Force_calculation()
{
  target_state = targetstates[state_index];
  getTarget_Force();
  Eigen::VectorXf ForceTorque = Eigen::VectorXf::Zero(6);
  ForceTorque.block(0,0,3,1) = target_force;
  ForceTorque.block(3,0,3,1) = target_torque;

  try {
    int a=_qp_solver.solveQP(footpoint,schedualgroundLeg,ForceTorque);
    if(a == 1){throw 1;}
  } catch (int) {
    osqp_unsolved_error = 1;
  }

  force_list = _qp_solver.foot_force;
  cout<<"error"<<_qp_solver.Error<<endl;
  for (int i = 0;i<4;i++) {
    if(_statemachine.phase[i]>= 0.95 and _statemachine._gait.Gait_phase[_statemachine._gait.Gait_index][i] != 1)
    {
      schedualgroundLeg[i] = 1;
      Eigen::Vector3f _force;
      _force<<0,0,10;
      force_list.block(0,i,3,1) =  _force;
    }
  }


}



void dog_controller::swingleg_calculation()
{
  target_state = targetstates[state_index];
  Eigen::Vector3f _target_vel = target_state.block(0,3,3,1);
  _target_vel = TF_mat.inverse()*_target_vel;
  std::vector<float> current_phase = _statemachine._gait.Gait_phase[_statemachine._gait.Gait_index];
  std::vector<float> next_phase = _statemachine._gait.Gait_phase[_statemachine._gait.get_nextindex()];
  float time = _statemachine._gait.Gait_time[_statemachine._gait.Gait_index];
  for (int i = 0;i<4;i++) {
    if(schedualgroundLeg[i] == 0)
    {
     int Xsidesign = pow(-1,i);
     int Ysidesign = pow(-1,1+i/2);
     float phase1 = current_phase[i];
     float phase2 = next_phase[i];
     float swing_time = time/(phase2 - phase1);
     Eigen::Vector3f final_point;
     final_point<<Xsidesign*(body_width + hip_lenth),Ysidesign*body_lenth + _statemachine._gait.Gait_pacePropotion*swing_time*_target_vel(1),-DEFAULT_HEIGHT;
     if( USE_RAIBERT_HEURISTIC)
     {
//       TODO
     }
     Eigen::Vector3f init_pos = init_SWINGfootpoint.block(0,i,3,1);
     _statemachine.gait_swingLeg(_statemachine.phase[i],swing_time,init_pos,final_point);
     if(use_sim)
     {
       _statemachine.target_pos(0) = final_point(0);
//       _statemachine.target_pos(1) = final_point(1);// magic change
     }

     target_swingpos.block(0,i,3,1) = _statemachine.target_pos;
     target_swingvel.block(0,i,3,1) = _statemachine.target_vel;
    }
  }
}




