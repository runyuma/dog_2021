#ifndef LOCOMOTION_CONTROLLER_H
#define LOCOMOTION_CONTROLLER_H
#include<ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include"dog_controller.h"
#include<math.h>
using namespace std;

class locomotion_controller
{
private:
public:
  ros::NodeHandle *pnh;
  ros::Subscriber footpoint_subscriber;
  ros::Subscriber footvel_subscriber;
  ros::Subscriber state_estimation_subscriber;

  ros::Time ros_time,last_rostime;
  double loop_time;

  int footpoint_gotvalue;
  int footvel_gotvalue;
  int state_gotvalue;

  ros::Publisher force_publisher;
  ros::Publisher swingleg_publisher;
  ros::Publisher leg_status_publisher;


  int time_index = 0;

  dog_controller *_Dog;

  locomotion_controller();
  ~locomotion_controller();
  void init();
  void footpoint_callback(const  std_msgs::Float32MultiArray::ConstPtr& msg);
  void footvel_callback(const  std_msgs::Float32MultiArray::ConstPtr& msg);
  void state_estimation_callback(const  std_msgs::Float32MultiArray::ConstPtr& msg);

  void status_publish();
  void set_schedulegroundleg();
  void force_publish();
  void swing_publoish();

  void moving_init();
  void moving_func();

  void visual();
};
#endif // LOCOMOTION_CONTROLLER_H
