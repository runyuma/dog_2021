#ifndef LEG_DYNAMIC_H_INCLUDED
#define LEG_DYNAMIC_H_INCLUDED

#include <iostream>
#include <Eigen/Dense>
#include <math.h>
using namespace std;


class leg_params
{
    public:
    float mass_list[3];
    float lenth_list[3];
    float discomlist[3];
/*!
 * 数组指针
 * 长度都是三
 */
    leg_params();
    leg_params(float *_mass_list, float *_lenth_list , float *_discomlist);
    ~leg_params();
};
Eigen::Vector3f get_footpoints(int sidesign,float *theta_list, float *lenth_list);
Eigen::Matrix3f get_jacobian(int sidesign,float *theta_list, float *lenth_list);
Eigen::Vector3f get_footpointvel(Eigen::Matrix3f _jacobian, float * jointvel_list);
Eigen::Vector3f get_tauff(int sidesign, float * theta_list, float * joint_vel, leg_params _legparam);
Eigen::Vector3f get_feedbackward(Eigen::Matrix3f kp,Eigen::Matrix3f kd,Eigen::Vector3f current_pos,Eigen::Vector3f current_vel,Eigen::Vector3f target_pos,Eigen::Vector3f target_vel);
#endif // LEG_DYNAMIC_H_INCLUDED
