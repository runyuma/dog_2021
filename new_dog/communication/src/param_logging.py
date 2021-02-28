#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np
dic = {'totoal_weight':16,
       'hip_mass': 0.605,
       'upper_link_mass': 0.7,
       'lower_link_mass': 0.07,
       'hip_lenth':0.1,
       'upper_link_lenth':0.215,
       'lower_link_lenth':0.215,
       'comb_lenth':0.02,
       'comc_lenth':0.05,
       "body_inertia":[0.3,0.1,0.4],
       "swingleg_P":[20,20,20],
       "swingleg_D":[3,3,3],
       "state_estimation_mode":0, # 0 is getfrom gezebo, 1 is pure leg dynamic, 2 is extended kalman fillter
	"body_lenth":0.255,
	"body_width":0.06

       }
def params_init():
    rospy.init_node("params_logging")
    rate = rospy.Rate(1)
    for i in dic:
        rospy.set_param(i, dic[i])
    # while not rospy.is_shutdown():
    #     for i in dic:
    #         rospy.set_param(i,dic[i])
    #     rate.sleep()
params_init()
