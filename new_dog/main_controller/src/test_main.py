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
test_swing_leg = 1
test_singleleg_two_point = 0
test_posWalking = 0
test_swing_leg_singlepos = 0
test_gravity = 0
test_pos = 0
recovery = 0


def test_upperconcole():
    rospy.init_node("uppernode")
    rate = rospy.Rate(1000)
    upperpub = rospy.Publisher('/downstream', Float32MultiArray,queue_size=10)
    msg = Float32MultiArray()
    # msg.data = [0 for i in range(12)] + [0, 1., -2.]*4
    msg.data = [-1, -1, 3]+[-1 for i in range(9)] + [0. for i in range(12)]
    upperpub.publish(msg)
    while not rospy.is_shutdown():
        msg.data =  [0, 0, 0]+ [0, 0, 0] + [0, 0, 0]+ [0, 0, 0]  +[0,1,-2]+ [0,1,-2]+[0,1,-2]+ [0,1,-2]
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
    rospy.set_param("leg_enable",[1,0,0,0])
    time.sleep(0.5)
    time_index = 0
    phase_count = 0

    _statemachine = [statemachine() for i in range(4)]
    # init_pos = [np.array([[0.1],[0],[-0.26]]),np.array([[-0.1],[0],[-0.26]]),np.array([[0.1],[0],[-0.26]]),np.array([[-0.1],[0],[-0.26]])]
    # final_pos =[np.array([[0.1],[0.1],[-0.26]]),np.array([[-0.1],[0.1],[-0.26]]),np.array([[0.1],[0.1],[-0.26]]),np.array([[-0.1],[0.1],[-0.26]])]
    init_pos = [np.array([[0.13],[0],[-0.28]]),np.array([[-0.13],[0],[-0.28]]),np.array([[0.13],[0],[-0.28]]),np.array([[-0.13],[0],[-0.28]])]
    final_pos =[np.array([[0.1],[0.1],[-0.31]]),np.array([[-0.1],[0.1],[-0.31]]),np.array([[0.1],[0.1],[-0.31]]),np.array([[-0.1],[0.1],[-0.31]])]
    while not rospy.is_shutdown():
        if test_jacobian:
            if time_index <=1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 0.78, -1.57] * 4
            else:
                status_msg.data = [0, 0 ,0 , 0]
                footforce_msg.data = [0,0,-30] * 4
        elif test_pos:
            status_msg.data = [5, 5, 5, 5]
            footforce_msg.data = [0, 0.78, -1.57] * 4
        elif test_singleleg_two_point:
            if time_index <= 1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 1., -2.] * 4
            else:
                if _statemachine[0].phase <= 1:
                    status_msg.data = [1,1,1,1]
                    _pos = [None,None,None,None]
                    _vel = [None, None, None, None]
                    for i in range(4):
                        T = 0.8
                        if phase_count%2 == 0:
                            _statemachine[i].generate_point(T,init_pos[i],final_pos[i])
                        elif phase_count%2 == 1:
                            _statemachine[i].generate_point(T, final_pos[i], init_pos[i])
                        _pos[i] = _statemachine[i].target_pos.T[0].tolist()
                        _vel[i] = _statemachine[i].target_vel.T[0].tolist()
                        _statemachine[i].phase += 1/(1000*T)
                    swingleg_msg.data = _pos[0] +[0., 0., 0.]*3+_vel[0]+[0., 0., 0.]*3
                   # swingleg_msg.data = _pos[0] + _pos[1]+ _pos[2]+ _pos[3]+ _vel[0]+ _vel[1]+ _vel[2]+ _vel[3]
                    # print(_statemachine[0].phase,_pos,_vel)
                    # print(footforce_msg.data)
                else:
                    for i in range(4):
                        _statemachine[i].phase = 0
                    phase_count += 1

        elif test_posWalking:   # CQS Write it.
            Swing_T = 0.25 # MRY Write this line.
            if time_index <= 3000:
                status_msg.data = [5, 5, 5, 5]
                if time_index < 1000:
                    footforce_msg.data = [0.1, 0.78, -2.5] * 4
                elif time_index < 2000:
                    footforce_msg.data = [0.1, 0.80, -2.] * 4
                elif time_index <= 3000:
                    footforce_msg.data = [0.1, 0.75, -1.5] * 4
                    print(footforce_msg.data)

                # phase init
                _statemachine[0].phase = 0
                _statemachine[1].phase = 2 / 3
                _statemachine[2].phase = 1 / 3
                _statemachine[3].phase = 0
                # phase init
            else:
                Swing_T_ms = Swing_T * 1000
                StartTimeIndex = time_index - 3000
                sign = int((StartTimeIndex / Swing_T_ms) % 2)
                if sign == 0:
                    SwingLegIndex = [0, 3]
                else:
                    SwingLegIndex = [1, 2]
                status_msg.data = [5, 5, 5, 5]
                for i in SwingLegIndex:
                    status_msg.data[i] = 1  # Swing Leg
                _pos = [None, None, None, None]
                _vel = [None, None, None, None]
                _posloop = [None, None, None, None]

                for i in range(4):
                    if i in SwingLegIndex:
                        if i == 0:
                            print("")
                        status_msg.data[i] = 1
                        _statemachine[i].phase += 1 / (1000 * Swing_T)
                        _statemachine[i].generate_point(Swing_T, init_pos[i], init_pos[i])
                        _pos[i] = _statemachine[i].target_pos.T[0].tolist()
                        _vel[i] = _statemachine[i].target_vel.T[0].tolist()
                        _posloop[i] = [0, 0, 0]
                    else:
                        if i == 0:
                            print("")
                        status_msg.data[i] = 5
                        _statemachine[i].phase += 1 / (1000 * Swing_T)
                        # _posloop[i] = [-0.02 + (-0.01 + 0.02) * _statemachine[i].phase, 0.936 + (0.922 - 0.936) * _statemachine[i].phase, -1.596 + (-1.589 + 1.596) * _statemachine[i].phase]
                        _posloop[i] = [0.1, 0.75, -1.5]
                        _pos[i] = [0, 0, 0]
                        _vel[i] = [0, 0, 0]

                    if _statemachine[i].phase >= 1:
                        _statemachine[i].phase = 0

                swingleg_msg.data = _pos[0] + _pos[1] + _pos[2] + _pos[3] + _vel[0] + _vel[1] + _vel[2] + _vel[3]
                footforce_msg.data = _posloop[0] + _posloop[1] + _posloop[2] + _posloop[3]

        elif test_swing_leg:
            if time_index <= 1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 1, -2] * 4

            else:
                if _statemachine[0].phase <= 1:
                    status_msg.data = [1,1,1,1]
                    _pos = [None, None, None, None]
                    _vel = [None, None, None, None]
                    for i in range(4):
                        T = 0.5
                        _statemachine[i].generate_point(T,init_pos[i],init_pos[i])
                        _pos[i] = _statemachine[i].target_pos.T[0].tolist()
                        _vel[i] = _statemachine[i].target_vel.T[0].tolist()

                        _statemachine[i].phase += 1/(1000*T)
                    # swingleg_msg.data = _pos[0] +[0.,0.,0.]*3+_vel[0]+[0.,0.,0.]*3
                    swingleg_msg.data = _pos[0] + _pos[1]+ _pos[2]+ _pos[3]+ _vel[0]+ _vel[1]+ _vel[2]+ _vel[3]

                    # print(_statemachine[0].phase,_pos,_vel)
                    # print(footforce_msg.data)
                else:
                    for i in range(4):
                        _statemachine[i].phase = 0
                    phase_count += 1

        elif test_swing_leg_singlepos:
            if time_index <= 1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 0.78, -1.57] * 4
            else:
                _pos = [0.10,0.0,-0.30]
                status_msg.data = [1, -1, -1, -1]
                swingleg_msg.data = _pos + [0., 0., 0.] * 3 + [0., 0., 0.] * 4
        elif test_gravity:
            if time_index <= 1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 1., -2.] * 4

            else:
                rospy.set_param("swingleg_P",[0,0,0])
                rospy.set_param("swingleg_D",[0,0,0])
                if _statemachine[0].phase <= 1:
                    status_msg.data = [1,1,1,1]
                    _pos = [None,None,None,None]
                    _vel = [None, None, None, None]
                    for i in range(4):
                        T = 0.4
                        _statemachine[i].generate_point(T,init_pos,final_pos)
                        _pos[i] = _statemachine[i].target_pos.T[0].tolist()
                        _vel[i] = _statemachine[i].target_vel.T[0].tolist()
                        _statemachine[i].phase += 1/(1000*T)
                    swingleg_msg.data = _pos[0] +[0.,0.,0.]*3+_vel[0]+[0.,0.,0.]*3
                    # swingleg_msg.data = _pos[0] + _pos[1]+ _pos[2]+ _pos[3]+ _vel[0]+ _vel[1]+ _vel[2]+ _vel[3]

                    print(_statemachine[0].phase,_pos,_vel)
                    print(footforce_msg.data)
                else:
                    _statemachine[0].phase = 0

        elif recovery:
            if time_index%1000<=500:
                status_msg.data = [5, 5, 5, 5]
                # footforce_msg.data = [0., 0.78, -1.57] * 4
                # footforce_msg.data = [0, 0, 0.] * 4
                footforce_msg.data = [0, 1., -2.] * 4
            else:
                status_msg.data = [5, 5, 5, 5]
                # footforce_msg.data = [0., 0.78, -1.57] * 4
                # footforce_msg.data = [0, 0, 0.] * 4
                footforce_msg.data = [0, 1., -2.] * 4
        # else:
        #     status_msg.data = [-1, -1, -1, -1]
        #     footforce_msg.data = [-0, -0, -0.] * 4

        time_index += 1
        status_publisher.publish(status_msg)
        footforce_publisher.publish(footforce_msg)
        swingleg_publisher.publish(swingleg_msg)
        rate.sleep()

if test_upperconcle:
    test_upperconcole()
elif test_singleleg:
    test_singleleg()