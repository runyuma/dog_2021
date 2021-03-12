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
       'coma_lenth':0.05,
       'comb_lenth':0.07,
       'comc_lenth':0.15,
       "body_inertia":[2,0.4,2],
       "swingleg_P":[450,250,150],
       "swingleg_D":[40,20,30],
       "schedule_groundleg":[1,1,1,1],
       "current_gait": 0,# 0 is standing 1 is trot_runing
       "command_vel":0,
       "command_omega":0,
       "state_estimation_mode":0, # 0 is getfrom gezebo, 1 is pure leg dynamic, 2 is extended kalman fillter
	"body_lenth":0.255,
	"body_width":0.055,
       "use_sim":1,
       }
# "swingleg_P":[200,150,150],
#        "swingleg_D":[10,20,10],
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
