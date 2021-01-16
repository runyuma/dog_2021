#include "leg_dynamic.h"

leg_params::leg_params(){}
leg_params::~leg_params(){}
leg_params::leg_params(float *_mass_list, float *_lenth_list , float *_discomlist)
{
    for( int i=0;i<3;i++ )
    {
        mass_list[i] = _mass_list[i];
        lenth_list[i] = _lenth_list[i];
        discomlist[i] = _discomlist[i];
    }

}
Eigen::Vector3f get_footpoints(int sidesign,float *theta_list, float *lenth_list)
{
    Eigen::Matrix3f Tofootpoint;
    float s0 = sin(theta_list[0]);
    float c0 = cos(theta_list[0]);
    float s1 = sin(theta_list[1]);
    float c1 = cos(theta_list[1]);
    float s12 = sin(theta_list[1]+theta_list[2]);
    float c12 = cos(theta_list[1]+theta_list[2]);
   
    Tofootpoint(0,0) = sidesign *c0;
    Tofootpoint(0,1) = sidesign * s0 * c1;
    Tofootpoint(0,2) = sidesign * s0 * c12;
    Tofootpoint(1,0) = 0;
    Tofootpoint(1,1) = s1;
    Tofootpoint(1,2) = s12;
    Tofootpoint(2,0) = s0;
    Tofootpoint(2,1) =  - c0 * c1;
    Tofootpoint(2,2) = - c0 * c12;
    Eigen::Vector3f lenthVec;
    lenthVec<< lenth_list[0], lenth_list[1], lenth_list[2];
    Eigen::Vector3f foot_point;
    foot_point = Tofootpoint * lenthVec;

    return foot_point;
}

Eigen::Matrix3f get_jacobian(int sidesign,float *theta_list, float *lenth_list)
{
    Eigen::Matrix3f jacobian;

    float l1 = lenth_list[0];
    float l2 = lenth_list[1];
    float l3 = lenth_list[2];

    float s0 = sin(theta_list[0]);
    float c0 = cos(theta_list[0]);
    float s1 = sin(theta_list[1]);
    float c1 = cos(theta_list[1]);
    float s12 = sin(theta_list[1]+theta_list[2]);
    float c12 = cos(theta_list[1]+theta_list[2]);

    jacobian(0,0) =  sidesign* (-s0 * l1 + c0 * (c1 * l2 + c12 * l3));
    jacobian(0,1) =  sidesign * s0 * (s1 * l2 + s12 * l3);
    jacobian(0,2) =  sidesign * s0 * (s12 * l3);
    jacobian(1,0) = 0;
    jacobian(1,1) = c1 * l2 + c12 * l3;
    jacobian(1,2) = c12 * l3;
    jacobian(2,0) = c0 * l1 - s0 * (c1 * l2 + c12 * l3);
    jacobian(2,1) = c0 * (s1 * l2 + s12 * l3);
    jacobian(2,2) =  c0 * (s12 * l3);

    return jacobian;
}
Eigen::Vector3f get_footpointvel(Eigen::Matrix3f _jacobian, float * jointvel_list)
{
    Eigen::Vector3f joint_array;
    joint_array<<jointvel_list[0],jointvel_list[1],jointvel_list[2];
    Eigen::Vector3f foot_pointvel = _jacobian * joint_array;
    return foot_pointvel;
}

Eigen::Vector3f get_tauff(int sidesign, float * theta_list, float * joint_vel, leg_params _legparam)
{
    float h = - _legparam.mass_list[2] * _legparam.discomlist[2] * _legparam.lenth_list[1] * sin(theta_list[2]);
    Eigen::Matrix3f C_mat;
    C_mat(0,0) = 0;
    C_mat(0,1) = 0;
    C_mat(0,2) = 0;
    C_mat(1,0) = h * joint_vel[2];
    C_mat(1,1) = h * (joint_vel[1] + joint_vel[2]);
    C_mat(1,2) = 0;
    C_mat(2,0) = 0;
    C_mat(2,1) = - h * joint_vel[1];
    C_mat(2,2) = 0;
    Eigen::Vector3f joint_array;
    joint_array<<joint_vel[0],joint_vel[1],joint_vel[2];
    Eigen::Vector3f gravity;
    gravity<< 0,0,-9.81;

    float lenthlist_l2[3] = {_legparam.lenth_list[0],_legparam.discomlist[1],0};
    Eigen::Matrix3f ja_l2 =get_jacobian(sidesign,theta_list,lenthlist_l2);

    float lenthlist_l3[3] = {_legparam.lenth_list[0],_legparam.lenth_list[1],_legparam.discomlist[2]};
    Eigen::Matrix3f ja_l3 =get_jacobian(sidesign,theta_list,lenthlist_l3);

    Eigen::Vector3f joint_torque = -_legparam.mass_list[1]*(ja_l2.transpose() * gravity) -_legparam.mass_list[2]*(ja_l3.transpose() * gravity);
    joint_torque = joint_torque + C_mat * joint_array;
    return joint_torque;
}

Eigen::Vector3f get_feedbackward(Eigen::Matrix3f kp,Eigen::Matrix3f kd,Eigen::Vector3f current_pos,Eigen::Vector3f current_vel,Eigen::Vector3f target_pos,Eigen::Vector3f target_vel)
{
    Eigen::Vector3f leg_force = kp * (target_pos - current_pos) + kd*(target_vel - current_vel);
    return leg_force;
}

