#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np
dic = {'totoal_weight':16,
       'body_mass':10,
       'hip_mass': 0.5,
       'upper_link_mass': 0.6,
       'lower_link_mass': 0.3,
       'hip_lenth':0.05,
       'upper_link_lenth':0.235,
       'lower_link_lenth':0.235,
       'comb_lenth':0.07,
       'comc_lenth':0.15,
       "body_inertia":[2,0.4,2],
       "swingleg_P":[200,150,150],
       "swingleg_D":[10,20,10],
       "state_estimation_mode":0 # 0 is getfrom gezebo, 1 is pure leg dynamic, 2 is extended kalman fillter
       }
# "swingleg_P":[400,350,350],
#        "swingleg_D":[10,20,10]
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
