#include <ros/ros.h>
#include "locomotion_controller.h"
#define TEST_QP 0
using namespace std;
locomotion_controller _locomotion_controller;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "locomotion");
  _locomotion_controller.pnh = new ros::NodeHandle();
  _locomotion_controller.init();
  ros::Rate loop_rate(1000.0);
  if(TEST_QP)
  {
    qp_solver _qp_solver = qp_solver();
    _qp_solver.test();
   gait_schedular _gait = gait_schedular("TROTING_WALKING");
   _gait.test();
   state_machine _state_mechane;
   _state_mechane.test();
  }

  while (ros::ok())
  {
    ros::spinOnce();
    if(_locomotion_controller.time_index == 0)
    {
      _locomotion_controller.moving_init();
    }
    _locomotion_controller.moving_func();
    loop_rate.sleep();
  }
}
