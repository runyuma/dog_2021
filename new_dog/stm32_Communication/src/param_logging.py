#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np
dic = {'total_weight':16,
       'hip_mass': 0.605,
       'upper_link_mass': 0.85,
       'lower_link_mass': 0.1,
       'hip_lenth':0.1,
       'upper_link_lenth':0.215,
       'lower_link_lenth':0.21,
       'coma_lenth':0.06,
       'comb_lenth':0.005,
       'comc_lenth':0.05,
       "body_inertia":[0.3,0.1,0.4],
       "swingleg_P":[80,100,150],
       "swingleg_D":[5,15,18],
       "schedule_groundleg": [1, 1, 1, 1],
       "current_gait": 0,  # 0 is standing 1 is trot_runing
       "command_vel": 0,
       "command_omega": 0,
       "state_estimation_mode":1, # 0 is getfrom gezebo, 1 is pure leg dynamic, 2 is extended kalman fillter
       "USE_TOUCHSENSOR": 0,
	"body_lenth":0.22,
	"body_width":0.055,
       "use_sim":0,
       "damping_compensation":[0.05,0.00,0.5],
       "walking_height": 0.30,
       }
dic["move_reset"] = 0
dic["start_move"] = 0

dic["osqp_unsolve_error"] = 0
dic["motor_outofrange_error"] = 0

dic["fallen_error"] = 0
dic["dog_action"] = "idle"

dic["locomotion_runing"] = 1;
dic["state_estimation_running"] = 1;
if dic["state_estimation_mode"] == 0:
    dic["stand_force_p"] = [2500,800,800]
    dic["stand_force_D"] = [600,350,150]
    dic["stand_troque_p"] = [200,300,300]
    dic["stand_troque_D"] = [30,12,50]
    dic["trot_force_p"] = [400,450,600]
    dic["trot_force_D"] = [250,100,120]
    dic["trot_troque_p"] = [400,600,500]
    dic["trot_troque_D"] = [50,65,50]
elif dic["state_estimation_mode"] == 1:
    dic["stand_force_p"] = [1,1,1]
    dic["stand_force_D"] = [1,1,1]
    dic["stand_troque_p"] = [2,50,2]
    dic["stand_troque_D"] = [1,5,1]
    dic["trot_force_p"] = [400,450,600]
    dic["trot_force_D"] = [200,100,160]
    dic["trot_troque_p"] = [450,650,500]#400,600,500
    dic["trot_troque_D"] = [50,70,50]#50,65
# "damping_compensation":[0.05,0.05,0.85 0r 0.72],
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
