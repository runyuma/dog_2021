#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np
dic = {'totoal_weight':16,
       'body_mass':10,
       'hip_mass': 0.05,
       'upper_link_mass': 0.3,
       'lower_link_mass': 0.3,
       'hip_lenth':0.05,
       'upper_link_lenth':0.235,
       'lower_link_lenth':0.235,
       'comb_lenth':0.07,
       'comc_lenth':0.15,
       "body_inertia":[2,0.4,2],
       "swingleg_P":[600,200,200],
       "swingleg_D":[50,5,5]
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
