#include "gait_schedular.h"

gait_schedular::gait_schedular(){}
gait_schedular::gait_schedular(string _gait)
{
/*  if(_gait=="TROTING_WALKING")
  {
    name = _gait;
    Gait_index = 0;
    Gait_phase = {{1, 0, 0, 1},{1,1,1,1},{0, 1, 1, 0},{1,1,1,1}};
    Gait_time = {0.2, 0.6, 0.2, 0.6};
    Gait_status = {"pace", "land", "pace", "land"};
    Gait_pacePropotion = 0.5;
  }*/

  ///jkc
  if(_gait=="TROTING_WALKING")
  {
    name = _gait;
    Gait_index = 0;
    Gait_phase = {{1, 0, 0, 1},{1,1,1,1},{0, 1, 1, 0},{1,1,1,1}};
    Gait_time = {0.15, 0.075, 0.15, 0.075};
    Gait_status = {"pace", "land", "pace", "land"};
    Gait_pacePropotion = 1.0;
  }

  else if (_gait == "TROTING_RUNING") {
    name = _gait;
    Gait_index = 0;
    Gait_phase = {{1, 0, 0, 1},{0, 1, 1, 0}};
    Gait_time = {0.2, 0.2};
    Gait_status = {"pace",  "pace"};
    Gait_pacePropotion = 0.5;
  }
  else if(_gait == "SLOW_WALKING"){
    name = _gait;
    Gait_index = 0;
    Gait_phase = {{1, 1, 0, 1},{1,1,1,1},{1,0,1,1},{1,1,1,1},{1,1,1,0},{1,1,1,1},{0, 1, 1, 1},{1,1,1,1}};
    Gait_time = {0.2,0.03, 0.2,0.03, 0.2,0.03,0.2,0.03};
    Gait_status = {"pace", "land", "pace", "land", "pace", "land", "pace", "land"};
    Gait_pacePropotion = 2;
 }
  else if (_gait == "STANDING") {
    name = _gait;
    Gait_index = 0;
    Gait_phase = {{1, 1, 1, 1}};
    Gait_time = {2};
    Gait_status = {"pace"};
    Gait_pacePropotion = 0.;

  }

}

int gait_schedular::get_nextindex()
{
 int lenth = Gait_time.size();
 if(Gait_index == lenth-1){return 0;}
 else {return Gait_index+1;}
}

int gait_schedular::get_lastindex()
{
  int lenth = Gait_time.size();
  if(Gait_index == 0){return lenth-1;}
  else {return Gait_index-1;}
}

void gait_schedular::gaittime_update(float T)
{
  Gait_currentTime += T;
}

void gait_schedular::get_schedualgroundLeg()
{
  for (int i=0;i<4;i++) {
    if(Gait_phase[Gait_index][i] ==1)
    {
      schedualgroundLeg[i] = 1;
    }
    else {
      schedualgroundLeg[i] = 0;
    }
  }
}

void gait_schedular::get_nextschedualgroundLeg()
{
  std::vector<int> groundLeg = {0,0,0,0};
  int next_index = get_nextindex();
  for (int i=0;i<4;i++) {
    if(Gait_phase[next_index][i] ==1)
    {
      nextschedualgroundLeg[i] = 1;
    }
    else {
      nextschedualgroundLeg[i] = 0;
    }
  }
}

void gait_schedular::test()
{
cout<<get_lastindex()<<endl;
cout<<get_nextindex()<<endl;
get_schedualgroundLeg();
get_nextschedualgroundLeg();
for (int i=0;i<4;i++) {cout<<schedualgroundLeg[i]<<endl;}
for (int i=0;i<4;i++) {cout<<nextschedualgroundLeg[i]<<endl;}
}

state_machine::state_machine(){}

void state_machine::gait_swingLeg(float _phase, float Tf, Eigen::Vector3f &ini_pos, Eigen::Vector3f &fin_pos)
{
  target_pos<<ini_pos(0) + (fin_pos(0) - ini_pos(0))*(3 * float(pow(_phase,2)) - 2*float(pow(_phase,3))),ini_pos(1) + (fin_pos(1) - ini_pos(1))*(3 * float(pow(_phase,2)) - 2*float(pow(_phase,3))),0;
  target_vel<<(fin_pos(0) - ini_pos(0))*(6 * _phase - 6*float(pow(_phase,2)))/Tf, (fin_pos(1) - ini_pos(1))*(6 * _phase - 6*float(pow(_phase,2)))/Tf,0;
  target_acc<<(fin_pos(0) - ini_pos(0))*(6  - 12*_phase)/float(pow(Tf,2)),(fin_pos(1) - ini_pos(1))*(6  - 12*_phase)/float(pow(Tf,2)),0;
  if (_phase<=0.5)
  {
    target_pos(2) = ini_pos(2) + height * (3 * float(pow(2*_phase,2)) - 2*float(pow(2*_phase,3)));
    target_vel(2) = height * (6 * (2*_phase) - 6*float(pow(2*_phase,2)))/(Tf/2);
    target_acc(2) = height * (6 - 6*(2*_phase))/float(pow(Tf/2,2));
  }
  else {
    target_pos(2) = fin_pos(2) + height - height * (3 * float(pow(2*_phase -1,2)) - 2*float(pow(2*_phase -1,3))) - 0.02;
    target_vel(2) = - height * (6 * (2*_phase -1) - 6*float(pow(2*_phase - 1,2)))/(Tf/2) - 0.05;
    target_acc(2) = - height * (6 - 6*(2*_phase -1 ))/float(pow(Tf/2,2));
  }
}

void state_machine::test()
{
  Eigen::Vector3f ini_pos,fin_pos;
  ini_pos<<0.1,0.255,-0.3;
  fin_pos<<0.1,0.305,-0.3;
  gait_swingLeg(0.5,0.4,ini_pos,fin_pos);
  std::cout<<"target_pos"<<target_pos<<std::endl;
  std::cout<<"target_vel"<<target_vel<<std::endl;
}
