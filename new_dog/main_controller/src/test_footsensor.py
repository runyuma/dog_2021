#!/usr/bin/env python3
# coding:utf-8
import rospy
import time
import numpy as np
import pandas as pd
import copy
from statemechine import *
from Sensor import *
from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray

test_upperconcle = 0
test_singleleg = 1
test_jacobian = 0
test_footsensor = 1
# must have leg idx
test_leg_idx = 0

test_swing_leg = 0
test_singleleg_two_point = 0
test_swing_leg_singlepos = 0
test_gravity = 0
recovery = 0
save_data = True
data = []
data_name = []
sensor = Sensor()
joint_pos = []
joint_vel = []
datatime_idx = -1


def alpha_mat(alpha):
    alpha_array = np.array([[np.cos(alpha), 0., np.sin(alpha)],
                            [0., 1., 0.],
                            [- np.sin(alpha), 0., np.cos(alpha)]])
    return alpha_array


def beta_mat(beta):
    beta_array = np.array([[1., 0., 0.],
                           [0., np.cos(beta), - np.sin(beta)],
                           [0., np.sin(beta), np.cos(beta)]])
    return beta_array


def gama_mat(gama):
    gama_array = np.array([[1., 0., 0.],
                           [0., np.cos(gama), - np.sin(gama)],
                           [0., np.sin(gama), np.cos(gama)]])
    return gama_array


def serialport_callback(msg):
    global datatime_idx
    datatime_idx = msg.data


def test_upperconcole():
    rospy.init_node("uppernode")
    rate = rospy.Rate(1000)
    upperpub = rospy.Publisher('/downstream', Float32MultiArray, queue_size=10)
    msg = Float32MultiArray()
    # msg.data = [0 for i in range(12)] + [0, 1., -2.]*4
    msg.data = [-1, -1, 3] + [-1 for i in range(9)] + [0. for i in range(12)]
    upperpub.publish(msg)
    while not rospy.is_shutdown():
        msg.data = [0, 0, 0] + [-1 for i in range(9)] + [0, 0.78, -1.57] + [0 for i in range(9)]
        upperpub.publish(msg)
        rate.sleep()


def test_singleleg():
    global datatime_idx
    friction = 0.3
    rospy.init_node("test_leg")
    rate = rospy.Rate(1000)
    serialport_subscriber = rospy.Subscriber('/timepub', Float32MultiArray, serialport_callback)
    status_publisher = rospy.Publisher('/leg_status', Int32MultiArray, queue_size=10)
    footforce_publisher = rospy.Publisher('/ground_force', Float32MultiArray, queue_size=10)
    swingleg_publisher = rospy.Publisher('/swing_leg', Float32MultiArray, queue_size=10)
    status_msg = Int32MultiArray()
    footforce_msg = Float32MultiArray()
    swingleg_msg = Float32MultiArray()
    rospy.set_param("leg_enable", [1, 0, 0, 0])
    time.sleep(0.5)
    time_index = 0
    phase_count = 0

    _statemachine = [statemachine() for i in range(4)]
    init_pos = [np.array([[0.1], [0], [-0.3]]), np.array([[-0.1], [0], [-0.26]]), np.array([[0.1], [0], [-0.26]]),
                np.array([[-0.1], [0], [-0.26]])]
    final_pos = [np.array([[0.1], [0.1], [-0.3]]), np.array([[-0.1], [0.1], [-0.26]]),
                 np.array([[0.1], [0.1], [-0.26]]), np.array([[-0.1], [0.1], [-0.26]])]
    while not rospy.is_shutdown():
        if test_jacobian:
            if time_index <= 1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 1., -2.] * 4
            else:
                status_msg.data = [0, 0, 0, 0]
                footforce_msg.data = [0, 0, -5] * 4
                if save_data:
                    data.append(footforce_msg)
                    if len(data) % 1000 == 0:
                        test_savedata('footforce', data, form='csv')

        elif test_footsensor:
            if time_index <= 1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 1.21, -1.68] * 4
            else:
                if 1000 < time_index <= 5000:
                    force_max = -20.
                    force_z = force_max * time_index / 5000
                    if force_z > 0: force_z = 0
                    force_x = 0.
                    force_y = 0.
                elif 5000 < time_index <= 10000:
                    force_max = -20.
                    force_z = force_max
                    if force_z > 0: force_z = 0
                    force_x = 1
                    force_y = 0.
                elif 10000 < time_index <= 15000:
                    force_max = -30.
                    force_z = force_max
                    if force_z > 0: force_z = 0
                    force_x = 0
                    force_y = 8
                elif 15000 < time_index <= 20000:
                    force_max = -25.
                    force_z = force_max
                    if force_z > 0: force_z = 0
                    force_x = 1.
                    force_y = 5.
                elif time_index > 20000:
                    force_max = -15.
                    force_z = force_max
                    if force_z > 0: force_z = 0
                    force_x = force_z * friction
                    force_y = 0
                    
                status_msg.data = [0, 0, 0, 0]
                footforce_msg.data = [force_x, force_y, force_z] * 4

                ########################################################################################################
                force_msg = copy.deepcopy(footforce_msg.data)
                joint_pos = sensor.joint_pos
                joint_vel = sensor.joint_vel

                # print("force:", force_x, force_y, force_z)
                alpha_array = alpha_mat(joint_pos[test_leg_idx][0])
                beta_array = beta_mat(-joint_pos[test_leg_idx][1])
                gama_array = gama_mat(-joint_pos[test_leg_idx][2])
                transfor_array = np.kron(np.eye(4), np.dot(gama_array, np.dot(beta_array, alpha_array)))
                foot_sensorData = copy.deepcopy(np.dot(transfor_array, np.array(force_msg)).tolist())
                foot_sensorData.append(joint_pos[test_leg_idx][0:3])
                # foot_sensorData.append(datatime_idx)
                # print(type(foot_sensorData))
                # print(foot_sensorData)
                ########################################################################################################
                if save_data:
                    data_name.append(str(datatime_idx))
                    data.append(foot_sensorData)
                    print(str(datatime_idx), force_msg[0:3])
                    if len(data) % 1000 == 0:
                        test_savedata(data_name, data, form='npy')

        elif test_singleleg_two_point:
            if time_index <= 1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 1., -2.] * 4

            else:
                if _statemachine[0].phase <= 1:
                    status_msg.data = [1, 1, 1, 1]
                    _pos = [None, None, None, None]
                    _vel = [None, None, None, None]
                    for i in range(4):
                        T = 0.4
                        if phase_count % 2 == 0:
                            _statemachine[i].generate_point(T, init_pos[i], final_pos[i])
                        elif phase_count % 2 == 1:
                            _statemachine[i].generate_point(T, final_pos[i], init_pos[i])
                        _pos[i] = _statemachine[i].target_pos.T[0].tolist()
                        _vel[i] = _statemachine[i].target_vel.T[0].tolist()

                        _statemachine[i].phase += 1 / (1000 * T)
                    swingleg_msg.data = _pos[0] + [0., 0., 0.] * 3 + _vel[0] + [0., 0., 0.] * 3
                    # swingleg_msg.data = _pos[0] + _pos[1]+ _pos[2]+ _pos[3]+ _vel[0]+ _vel[1]+ _vel[2]+ _vel[3]

                    print(_statemachine[0].phase, _pos, _vel)
                    print(footforce_msg.data)
                else:
                    for i in range(4):
                        _statemachine[i].phase = 0
                    phase_count += 1
        elif test_swing_leg:
            if time_index <= 1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 0.78, -1.57] * 4

            else:
                if _statemachine[0].phase <= 1:
                    status_msg.data = [1, 1, 1, 1]
                    _pos = [None, None, None, None]
                    _vel = [None, None, None, None]
                    for i in range(4):
                        T = 0.4
                        _statemachine[i].generate_point(T, init_pos[i], init_pos[i])
                        _pos[i] = _statemachine[i].target_pos.T[0].tolist()
                        _vel[i] = _statemachine[i].target_vel.T[0].tolist()

                        _statemachine[i].phase += 1 / (1000 * T)
                    swingleg_msg.data = _pos[0] + [0., 0., 0.] * 3 + _vel[0] + [0., 0., 0.] * 3
                    swingleg_msg.data = _pos[0] + _pos[1] + _pos[2] + _pos[3] + _vel[0] + _vel[1] + _vel[2] + _vel[3]

                    print(_statemachine[0].phase, _pos, _vel)
                    print(footforce_msg.data)
                else:
                    for i in range(4):
                        _statemachine[i].phase = 0
                    phase_count += 1

        elif test_swing_leg_singlepos:
            if time_index <= 1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 0.78, -1.57] * 4
            else:
                _pos = [0.10, 0.0, -0.30]
                status_msg.data = [1, -1, -1, -1]
                swingleg_msg.data = _pos + [0., 0., 0.] * 3 + [0., 0., 0.] * 4
        elif test_gravity:
            if time_index <= 1000:
                status_msg.data = [5, 5, 5, 5]
                footforce_msg.data = [0, 1., -2.] * 4

            else:
                rospy.set_param("swingleg_P", [0, 0, 0])
                rospy.set_param("swingleg_D", [0, 0, 0])
                if _statemachine[0].phase <= 1:
                    status_msg.data = [1, -1, -1, -1]
                    _pos = [None, None, None, None]
                    _vel = [None, None, None, None]
                    for i in range(4):
                        T = 0.4
                        _statemachine[i].generate_point(T, init_pos, final_pos)
                        _pos[i] = _statemachine[i].target_pos.T[0].tolist()
                        _vel[i] = _statemachine[i].target_vel.T[0].tolist()
                        _statemachine[i].phase += 1 / (1000 * T)
                    swingleg_msg.data = _pos[0] + [0., 0., 0.] * 3 + _vel[0] + [0., 0., 0.] * 3
                    # swingleg_msg.data = _pos[0] + _pos[1]+ _pos[2]+ _pos[3]+ _vel[0]+ _vel[1]+ _vel[2]+ _vel[3]

                    print(_statemachine[0].phase, _pos, _vel)
                    print(footforce_msg.data)
                else:
                    _statemachine[0].phase = 0

        elif recovery:
            if time_index % 1000 <= 500:
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


def test_savedata(name1, data1, name2=None, data2=None, form=None):
    dic1 = zip(name1, data1)
    dataF_1 = pd.DataFrame(dic1)
    if form == 'csv':
        dataF_1.to_csv('/home/marunyu/name1' + '_Framework.csv')
    else:
        np.save('/home/marunyu/name1' + '_Framework.npy', dataF_1)

    # if data2 is not None:
    #     dic2 = {name2: data2}
    #     dataF_2 = pd.DataFrame(dic2)
    #     if form == 'csv':
    #         dataF_2.to_csv(name2 + '_Framework.csv')
    #     else:
    #         np.save(name2 + '_Framework.npy', dataF_2)


if test_upperconcle:
    test_upperconcole()
elif test_singleleg:
    test_singleleg()
