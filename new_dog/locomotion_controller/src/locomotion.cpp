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
  if(TEST_QP)
  {
    qp_solver _qp_solver = qp_solver();
    _qp_solver.test();
   gait_schedular _gait = gait_schedular("TROTING_WALKING");
   _gait.test();
   state_machine _state_mechane;
   _state_mechane.test();
  }
  int start_move = 0;
  while (ros::ok())
  {
    ros::spinOnce();
//    _locomotion_controller.pnh->setParam("locomotion_runing",1);
    static bool error_inited = 0;
    int start_timeindex;
    if(_locomotion_controller.time_index == 0)
    {
      _locomotion_controller.moving_init();
    }
//    _locomotion_controller.moving_func();
    // std::cout<<"startmove"<<start_move <<std::endl;
    if(not _locomotion_controller.error_handle())
    {

      _locomotion_controller.pnh->getParam("start_move", start_move);
      if(start_move)
      {
        _locomotion_controller.moving_func();
      }
      else {
        _locomotion_controller.pnh->getParam("start_move", start_move);
        _locomotion_controller.idle();
      }
    }
    else {
      if (not error_inited)
      {
        start_timeindex = _locomotion_controller.time_index;
        error_inited = 1;
        start_move = 0;
      }
      bool recover = _locomotion_controller.shrink(start_timeindex,_locomotion_controller._Dog->rpy);
      // std::cout<<"recover"<<recover <<std::endl;
      if(recover)
      {
         _locomotion_controller.moving_reset();
        // std::cout<<"********************recover*****************"<<std::endl;
      }
    }

  }
}
