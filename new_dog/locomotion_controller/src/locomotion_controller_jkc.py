#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np
import copy
from qp_controller import *
from qp_mpc import *
from gait_schedular import *
from state_machine import *
from state_estimate import *
from cvxopt import matrix, solvers
from scipy import sparse
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray
import math
import time

global obj_init
global swing_leg_init
global init_pos
global final_pos
obj_init = 0
swing_leg_init = 0
init_pos = [[[0.], [0.], [0.]] for i in range(4)]
final_pos = [[[0.], [0.], [0.]] for i in range(4)]


# 0 init obj


class locomotion_controller():
    def __init__(self):
        rospy.init_node('locomotion_controller', anonymous=True)
        self.rate = rospy.Rate(1000)
        self.sensor_subscriber = rospy.Subscriber('/upstream', Float32MultiArray, self.sensor_callback)
        self.footpoint_subscriber = rospy.Subscriber("/foot_points", Float32MultiArray, self.footpoint_callback)
        self.footvel_subscriber = rospy.Subscriber('/foot_vel', Float32MultiArray, self.footvel_callback)
        self.Imu_Subscriber = rospy.Subscriber("/imu", Imu, self.Imu_callback)
        self.state_Subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.state_callback)
        # footpoint: lenth: 12 [4*[[x],[y],[z]]]
        # footvel: lenth: 12 [4*[[vx],[vy],[vz]]]
        # sensor:lower computer joint message

        self.force_publisher = rospy.Publisher('/ground_force', Float32MultiArray, queue_size=10)
        self.swingleg_publisher = rospy.Publisher('/swing_leg', Float32MultiArray, queue_size=10)
        self.leg_status_publisher = rospy.Publisher('/leg_status', Int32MultiArray, queue_size=10)
        # ground_force: lenth:12 4*[x,y,z] [0,0,0] means no ground force control
        # swing_leg: lenth:24 4*[x,y,z] 4*[vx,vy,vz] [0,0,0] means no swing force control
        # leg_status: -1:uninitialized do nothing//0: ground//1:swing//2:pos(data in ground force topic)// 5: jointpos//6:joint_torque

        self.joint_pos = [[None, None, None] for i in range(4)]
        self.joint_vel = [[None, None, None] for i in range(4)]
        self.foot_point = [[None, None, None] for i in range(4)]
        self.foot_joint_point = [[None, None, None] for i in range(4)]
        self.foot_vel = [[None, None, None] for i in range(4)]
        self.body_pos = [0., 0., 0.]
        self.linear_Vel = [0., 0., 0.]
        self.angular_velocity = [0., 0., 0.]
        self.linear_acceleration = [0., 0., 0.]
        self.orientation = [0., 0., 0.]
        # the list above is used for subscribe

        self.force = [[None, None, None] for i in range(4)]
        self.swing_leg = [0., 0., 0., 0., 0., 0.] * 4
        self.leg_status = [-1, -1, -1, -1]
        # the list above is used for publish

        self.force_publishmsg = Float32MultiArray(data=[[None, None, None] for i in range(4)])
        self.swingleg_publishmsg = Float32MultiArray(data=[[float('nan'), float('nan'), float('nan')] for i in range(8)]
                                                     )
        self.leg_status_publishmsg = Int32MultiArray(data=[-1, -1, -1, -1])
        # the list above is for publised to basenode about the force, swingleg_gait, leg_status of control

        self.schedular = gait_schedular()
        self.state_machine = [state_machine() for i in range(4)]
        self.time_index = 0
        self.forcecontroll_mode = 0
        # 0:qp 1:MPC

        self.PD_mode = 4
        # 2:walk 4:stand

        # init some nodes
        self.schedular.gait_init("test")
        self.time_index = 0

    def main(self):
        global obj_init
        global swing_leg_init
        global init_pos
        global final_pos
        while not rospy.is_shutdown():
            start_time = time.time()
            for i in range(4):
                if i % 2 == 0:
                    LorR = 1
                else:
                    LorR = -1
                # LorR: left or right
                _joint_pos = self.joint_pos[i]
                _joint_vel = self.joint_vel[i]

            # if 250 < self.time_index <= 500: # <= 2000:
            #     print("time_index: ", self.time_index)
            #     self.leg_status_publishmsg.data = [5, 5, 5, 5]
            #     self.force = [0., math.pi / 2 - self.time_index / 500 * math.pi/6.,
            #                   -math.pi + self.time_index / 500 * math.pi / 3] * 4
            #
            #     self.force_publish(self.force)
            #     self.swingleg_publish(self.swing_leg)
            #     self.leg_status_publish()
            # >>>
            if 0 < self.time_index < 300:  # <= 2000:
                print("time_index: ", self.time_index)
                # self.leg_status_publishmsg.data = [5, 5, 5, 5]
                # self.force = [0., math.pi / 2 - math.pi / 10, - math.pi + math.pi / 5] * 4
                #
                # self.force_publish(self.force)
                # self.swingleg_publish(self.swing_leg)
                # self.leg_status_publish()
            if self.time_index == 1600:  #####
                obj_init = 0
                swing_leg_init = 0
                self.schedular.gait_init("walk")
                self.schedular.Gait_currentTime = 0.
                self.schedular.Gait_index = 0

            if self.time_index >= 300:  # > 2000:
                print("time_index", self.time_index)
                leg_num = len(self.schedular.get_groundLeg())
                groundLeg = self.schedular.get_groundLeg()

                # swing leg
                for i in range(4):
                    # print("foot_joint_point_check: ", self.foot_joint_point)
                    if i not in groundLeg:
                        if swing_leg_init == 0:
                            init_pos[i] = [[self.foot_joint_point[i][0]], [self.foot_joint_point[i][1]],
                                           [self.foot_joint_point[i][2]]]
                            final_pos[i] = [[0.052 * (-1) ** i], [0.],  # 0.04958769306540489 0.049511801451444626
                                            [self.foot_joint_point[i][2]]]
                            # print("init_pos: ", i, ":", init_pos, init_pos[0][0], init_pos[1][0], init_pos[2][0])
                            # print("init_pos: ", i, ":", init_pos[0][0], init_pos[1][0], init_pos[2][0])
                            # print("final_pos: ", final_pos[0][0], final_pos[1][0], final_pos[2][0])
                            swing_time = self.schedular.Gait_time[self.schedular.Gait_index]

                        print("init_pos: ", i, ":", init_pos[0][0], init_pos[1][0], init_pos[2][0])

                        # velocity compensation
                        final_pos_ammend = [[2000 * self.linear_Vel[0] / 9.8 * self.foot_joint_point[i][2]],
                                            [2000 * self.linear_Vel[1] / 9.8 * self.foot_joint_point[i][2]], [0.]]

                        self.state_machine[i].gait_swingLeg(swing_time, init_pos[i], final_pos[i] + final_pos_ammend)
                        self.state_machine[i].phase = self.schedular.Gait_currentTime / self.schedular.Gait_time[
                            self.schedular.Gait_index]

                        ###
                        self.swing_leg[3 * i] = self.state_machine[i].target_pos[0][0]
                        self.swing_leg[3 * i + 1] = self.state_machine[i].target_pos[1][0]
                        self.swing_leg[3 * i + 2] = self.state_machine[i].target_pos[2][0]
                        self.swing_leg[3 * i + 12] = self.state_machine[i].target_vel[0][0]
                        self.swing_leg[3 * i + 12 + 1] = self.state_machine[i].target_vel[1][0]
                        self.swing_leg[3 * i + 12 + 2] = self.state_machine[i].target_vel[2][0]

                print("swing_leg: ", self.swing_leg)
                swing_leg_init = 1

                # status publishmsg
                for i in range(4):
                    if i in groundLeg:
                        self.leg_status_publishmsg.data[i] = 0
                    else:
                        self.leg_status_publishmsg.data[i] = 1

                # force control
                if obj_init == 0 and self.schedular.name != "test":
                    obj = get_next_state(self.orientation, self.body_pos, self.linear_Vel, self.angular_velocity,
                                         self.schedular.Gait_time[self.schedular.Gait_index], 10)
                    print("Stand")
                    obj_init = 1

                elif obj_init == 0 and self.schedular.name == "test":
                    obj = get_test_state(self.orientation, self.body_pos, self.linear_Vel, self.angular_velocity,
                                         self.schedular.Gait_time[self.schedular.Gait_index],
                                         self.schedular.Gait_currentTime, 10)
                    print("Test")
                    obj_init = 1

                nsim = int(self.schedular.Gait_time[self.schedular.Gait_index] / T) + 10 + 1
                dog_obj = obj[
                          int((self.schedular.Gait_currentTime / self.schedular.Gait_time[self.schedular.Gait_index]) *
                              nsim):
                          int((self.schedular.Gait_currentTime / self.schedular.Gait_time[
                              self.schedular.Gait_index]) * nsim) + 11]
                print("current: ", self.schedular.Gait_currentTime, " gait_time: ",
                      self.schedular.Gait_time[self.schedular.Gait_index])
                print("obj: ", dog_obj)
                x0 = self.orientation + self.body_pos + self.angular_velocity + self.linear_Vel
                print("x0: ", np.array(x0).reshape(4, 3))
                if self.foot_point[0][0] is not None:
                    # _foot_point = [[None, None, None] for i in range(4)]
                    # for i in range(4):
                    #     _foot_point[i][0] = self.foot_point[i][0] + 0.12 * (-1) ** i  #####?
                    #     _foot_point[i][1] = self.foot_point[i][1] + 0.254415 * (-1) ** (int(i / 2) + 1)
                    #     _foot_point[i][2] = self.foot_point[i][2]

                    # print("_foot_point: ", _foot_point)
                    if self.forcecontroll_mode == 1:
                        force = qp_MPC(self.foot_point, dog_obj, self.body_pos, self.orientation, self.angular_velocity,
                                       self.linear_Vel, leg_num, groundLeg)
                        self.force = force

                    if self.forcecontroll_mode == 0:
                        self.PD_mode = leg_num
                        forceTorque = PD_center_forceTorque(self.PD_mode, dog_obj[0], self.body_pos, self.orientation,
                                                            self.angular_velocity, self.linear_Vel)

                        print("forceTorque: ", forceTorque)

                        force = -1 * QP_solve(self.foot_point, groundLeg, forceTorque, self.schedular.name)

                        if leg_num == 4:
                            self.force = force
                            print("self.force: ", -1 * self.force.reshape(4, 3))
                        elif leg_num == 2:
                            # K for hip joint
                            K_joint = 250
                            for i in range(2):
                                force_ammend = K_joint * (0. - self.joint_pos[groundLeg[i]][0])
                                if force_ammend > 5:
                                    force_ammend = 5
                                elif force_ammend < -5:
                                    force_ammend = -5
                                print("joint_vel_error [", groundLeg[i], "]: ", (self.joint_vel[groundLeg[i]][0] - 0.))
                                print("force_ammend [", i, "]: ", (-1) ** groundLeg[i] * force_ammend)
                                force[3 * i] = (-1) ** groundLeg[i] * force_ammend + force[3 * i]

                            self.force = [0. for i in range(12)]
                            self.force[groundLeg[0] * 3: groundLeg[0] * 3 + 3] = force[0:3]
                            self.force[groundLeg[1] * 3: groundLeg[1] * 3 + 3] = force[3:6]

                            print("self.force: ", -1 * np.array(self.force).reshape(4, 3))

                    # self.force = [0., 0., -35., 0., 0., -35., 0., 0., -35., 0., 0., -35.]

                self.force_publish(self.force)
                self.swingleg_publish(self.swing_leg)
                self.leg_status_publish()

            # update gait #####?
            self.schedular.gaittime_update(0.001)

            # time
            self.time_index += 1
            if self.schedular.Gait_currentTime > self.schedular.Gait_time[self.schedular.Gait_index]:
                self.schedular.get_indexChange()
                swing_leg_init = 0
                if self.schedular.name != "test":
                    obj = get_next_state(self.orientation, self.body_pos, self.linear_Vel, self.angular_velocity,
                                         self.schedular.Gait_time[self.schedular.Gait_index], 10)
                elif self.schedular.name == "test":
                    obj = get_test_state(self.orientation, self.body_pos, self.linear_Vel, self.angular_velocity,
                                         self.schedular.Gait_time[self.schedular.Gait_index],
                                         self.schedular.Gait_currentTime, 10)

            if self.time_index % 1000 == 0:
                print("loop_time:", time.time() - start_time)
            self.rate.sleep()
            self.time_index += 1

    def Imu_callback(self, msg):
        # Read the quaternion of the robot IMU
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Read the angular velocity of the robot IMU
        w_x = msg.angular_velocity.x
        w_y = msg.angular_velocity.y
        w_z = msg.angular_velocity.z
        self.angular_velocity = [w_x, w_y, w_z]

        # Read the linear acceleration of the robot IMU
        a_x = msg.linear_acceleration.x
        a_y = msg.linear_acceleration.y
        a_z = msg.linear_acceleration.z
        self.linear_acceleration = [a_x, a_y, a_z]

        # Convert Quaternions to Euler-Angles Z-Y-X
        rpy_angle = [0, 0, 0]  # XYZ
        # rpy_angle[0] = math.atan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
        # rpy_angle[1] = math.asin(2 * (w * y - z * x))
        # rpy_angle[2] = math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
        # Above: Convert Quaternions to rpy-Angles

        rpy_angle[0] = math.atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z)
        rpy_angle[1] = math.asin(-2 * (x * z - w * y))
        rpy_angle[2] = math.atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z)
        self.orientation = rpy_angle
        print("Euler-Angles: ", rpy_angle)

    def state_callback(self, msg):
        # Read the position of the robot
        # print("state_callback")
        x = msg.pose[1].position.x
        y = msg.pose[1].position.y
        z = msg.pose[1].position.z
        self.body_pos = [x, y, z]
        # print("state_callback working")

        # Read the Vel of the robot
        vx = msg.twist[1].linear.x
        vy = msg.twist[1].linear.y
        vz = msg.twist[1].linear.z
        self.linear_Vel = [vx, vy, vz]

    def sensor_callback(self, msg):
        _joint_pos, _joint_vel = msg.data[:12], msg.data[12:]
        self.joint_pos = [i.tolist() for i in np.split(_joint_pos, [3, 6, 9])]
        self.joint_vel = [i.tolist() for i in np.split(_joint_vel, [3, 6, 9])]
        print("joint_pos: ", self.joint_pos)
        for i in range(12):
            if np.isnan(_joint_pos[i]):
                self.joint_pos[i // 3][i % 3] = None
                self.joint_vel[i // 3][i % 3] = None

    def footpoint_callback(self, msg):
        _foot_point = [[None, None, None] for i in range(4)]
        # print(msg.data)
        for i in range(4):
            _foot_point[i][0] = msg.data[i * 3 + 0]  #####?
            _foot_point[i][1] = msg.data[i * 3 + 1]
            _foot_point[i][2] = msg.data[i * 3 + 2]

        self.foot_joint_point = copy.deepcopy(_foot_point)
        print("foot_joint_point_0: ", self.foot_joint_point)

        for i in range(4):
            _foot_point[i][0] = _foot_point[i][0] + 0.06 * (-1) ** i  #####?
            _foot_point[i][1] = _foot_point[i][1] + 0.254415 * (-1) ** (int(i / 2) + 1)
            _foot_point[i][2] = _foot_point[i][2]

        self.foot_point = _foot_point
        # print("foot_joint_point_1: ", self.foot_joint_point)  ###

    def footvel_callback(self, msg):
        _foot_vel = [[None, None, None] for i in range(4)]
        for i in range(4):
            _foot_vel[i][0] = msg.data[i * 3 + 0]  #####?
            _foot_vel[i][1] = msg.data[i * 3 + 1]
            _foot_vel[i][2] = msg.data[i * 3 + 2]
        self.foot_vel = _foot_vel

    #
    def leg_status_publish(self):
        # print(self.leg_status_publishmsg)
        self.leg_status_publisher.publish(self.leg_status_publishmsg)

    def force_publish(self, force):
        # _force = []
        # for i in range(4):
        # _force += force[i]
        # print("force_publish: ", force)
        self.force_publishmsg.data = force
        # print("force_publishmsg", self.force_publishmsg)
        self.force_publisher.publish(self.force_publishmsg)

    def swingleg_publish(self, swing_leg):
        # _swingleg_point = swing_leg[0:4]
        # _swingleg_vel = swing_leg[4:8]
        self.swingleg_publishmsg.data = swing_leg
        # print(self.swingleg_publishmsg)
        self.swingleg_publisher.publish(self.swingleg_publishmsg)


_locomotion_controller = locomotion_controller()
_locomotion_controller.main()
