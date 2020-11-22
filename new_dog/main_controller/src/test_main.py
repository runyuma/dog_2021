#!/usr/bin/env python3
# coding:utf-8
import rospy
import time
import numpy as np
import copy
from statemechine import *
from std_msgs.msg import Float32MultiArray,Float32,Int32MultiArray
test_upperconcle = 0
test_singleleg = 1
test_jacobian = 0
test_swing_leg = 0
def test_upperconcole():
    rospy.init_node("uppernode")
    rate = rospy.Rate(1)
    upperpub = rospy.Publisher('/downstream', Float32MultiArray,queue_size=10)
    msg = Float32MultiArray()
    # msg.data = [0 for i in range(12)] + [0, 1., -2.]*4
    msg.data = [-1, -1, 3]+[-1 for i in range(9)] + [0. for i in range(12)]
    upperpub.publish(msg)
    while not rospy.is_shutdown():
        msg.data = [-1, -1, 0] + [-1 for i in range(9)] +[0,0,.0]+ [0 for i in range(9)]
        upperpub.publish(msg)
        rate.sleep()
def test_singleleg():
    rospy.init_node("test_leg")
    rate = rospy.Rate(1000)
    status_publisher = rospy.Publisher('/leg_status', Int32MultiArray,queue_size=10)
    footforce_publisher = rospy.Publisher('/ground_force', Float32MultiArray, queue_size=10)
    swingleg_publisher = rospy.Publisher('/swing_leg', Float32MultiArray, queue_size=10)
    status_msg = Int32MultiArray()
    footforce_msg = Float32MultiArray()
    swingleg_msg = Float32MultiArray()
    rospy.set_param("leg_enable",[1,1,1,1])
    time.sleep(0.5)
    time_index = 0

    if test_swing_leg:
        _statemachine = [statemachine() for i in range(4)]
        init_pos = np.array([[0.05],[0],[-0.26]])
        final_pos =np.array([[0.05],[0.1],[-0.26]])
    while not rospy.is_shutdown():
        if test_jacobian:
            if time_index <=1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 1., -2.] * 4
            else:
                status_msg.data = [0, 0 ,0 , 0]
                footforce_msg.data = [0,0,-20] * 4
        elif test_swing_leg:
            if time_index <= 1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 1., -2.] * 4

            else:
                if _statemachine[0].phase <= 1:
                    status_msg.data = [1,1,1,1]
                    _pos = [None,None,None,None]
                    _vel = [None, None, None, None]
                    for i in range(4):
                        _statemachine[i].generate_point(0.5,init_pos,final_pos)
                        _pos[i] = _statemachine[i].target_pos.T[0].tolist()
                        _vel[i] = _statemachine[i].target_vel.T[0].tolist()
                        _statemachine[i].phase += 0.002
                    # swingleg_msg.data = _pos +[0,0,0]*3+_vel+[0,0,0]*3
                    swingleg_msg.data = _pos[0] + _pos[1]+ _pos[2]+ _pos[3]+ _vel[0]+ _vel[1]+ _vel[2]+ _vel[3]

                    print(_statemachine[0].phase,_pos,_vel)
                    print(footforce_msg.data)
        else:
            status_msg.data = [5, 5, 5, 5]
            footforce_msg.data = [0, 0, 0.] * 4

        time_index += 1
        status_publisher.publish(status_msg)
        footforce_publisher.publish(footforce_msg)
        swingleg_publisher.publish(swingleg_msg)
        rate.sleep()

if test_upperconcle:
    test_upperconcole()
elif test_singleleg:
    test_singleleg()