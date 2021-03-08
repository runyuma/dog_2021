#ifndef GAIT_SCHEDULAR_H
#define GAIT_SCHEDULAR_H
using namespace std;
#include <string>
#include <list>
#include <vector>
#include<iostream>
#include<Eigen/Dense>
#include<math.h>

class gait_schedular
{
public:
  std::string name;
  std::vector<std::vector<float>> Gait_phase;
  std::vector<float> Gait_time;
  std::vector<std::string> Gait_status;
  int schedualgroundLeg[4];
  int nextschedualgroundLeg[4];
  float Gait_currentTime = 0;
  float Gait_pacePropotion = 0.5;
  int Gait_index = 0;

  gait_schedular();
  gait_schedular(std::string _gait);
  int get_nextindex();
  int get_lastindex();
  void gaittime_update(float T);
  void get_schedualgroundLeg();
  void get_nextschedualgroundLeg();
  void test();
};
class state_machine
{
public:
  std::vector<float> phase = {1,1,1,1};
  float time_step = 0.001;
  gait_schedular _gait = gait_schedular();
  float height = 0.15;
  Eigen::Vector3f target_pos,target_vel,target_acc;
  state_machine();
  void gait_swingLeg(float _phase, float Tf, Eigen::Vector3f &ini_pos, Eigen::Vector3f &fin_pos);
  void test();

};

#endif // GAIT_SCHEDULAR_H
