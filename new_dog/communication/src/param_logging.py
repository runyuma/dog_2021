#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np
dic = {'totoal_weight':16,
       'hip_mass': 0.6,
       'upper_link_mass': 0.95,
       'lower_link_mass': 0.25,
       'hip_lenth':0.095,
       'upper_link_lenth':0.23,
       'lower_link_lenth':0.22,
       'comb_lenth':0.04,
       'comc_lenth':0.12,
       "body_inertia":[0.3,0.1,0.4],
       "swingleg_P":[150,150,120],
       "swingleg_D":[35,45,30],
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
