#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np

from dog_controller import *



from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray
import math
import time
from action import *
DEBUG = 0

class locomotion_controlller():
    def __init__(self):
        rospy.init_node('locomotion_controller', anonymous=True)
        self.rate = rospy.Rate(1000)

        self.footpoint_subscriber = rospy.Subscriber("/foot_points", Float32MultiArray, self.footpoint_callback)
        self.footvel_subscriber = rospy.Subscriber('/foot_vel', Float32MultiArray, self.footvel_callback)
        self.state_estimation_subscriber = rospy.Subscriber('/state', Float32MultiArray, self.state_estimation_callback)
        self.footpoint_gotvalue = 0
        self.footvel_gotvalue = 0
        self.state_gotvalue = 0



        self.force_publisher = rospy.Publisher('/ground_force', Float32MultiArray, queue_size=10)
        self.swingleg_publisher = rospy.Publisher('/swing_leg', Float32MultiArray, queue_size=10)
        self.leg_status_publisher = rospy.Publisher('/leg_status', Int32MultiArray, queue_size=10)

        self.status_msg = Int32MultiArray()
        self.status_msg.data = [0,0,0,0]

        self.time_index = 0
        rospy.set_param("leg_enable", [1, 1, 1, 1])

        self.define_action()
        self.action_queue = [self.moving_action]

    def define_action(self):
        self.moving_action = action("moving",3,float("inf"),self.moving_init,self.moving_fun,self.moving_destructor)

    def moving_init(self):
        self._Dog = Dog()
        self._Dog.body_lenth = rospy.get_param("body_lenth")
        self._Dog.body_width = rospy.get_param("body_width")
        self._Dog.last_rostime = rospy.get_time()
        rospy.set_param("current_gait",0)
        self.time_index = 0
        self.last_time = time.time()
        self.last_rostime = rospy.get_time()
        self.is_moving = 1
        # TODO:restart stateestimation

    def moving_fun(self):
        rostime = rospy.get_time()
        self._Dog.ros_time = rostime
        self._Dog.command_vel = np.array([[0], [rospy.get_param("command_vel")], [0]])
        self._Dog.command_omega = np.array([[0], [0], [rospy.get_param("command_omega")]])
        self._Dog.gait_num = rospy.get_param("current_gait")
        self._Dog.statemachine_update()
        self._Dog.get_TFmat()
        if self.time_index % 8 == 0:
            self._Dog.Force_calculation()
            self.force_publish()
        self._Dog.swingleg_calculation()
        self.set_schedulegroundleg()
        self.status_piblish()
        self.swing_publish()
        self.visual()
        self.time_index += 1

    def moving_destructor(self):
        self.is_moving = 0

    def main(self):
        while not rospy.is_shutdown():
            _action = self.action_queue[0]
            if self.time_index == 0:
                _action.action_initfun()
            _action.action_fun()
            if self.time_index >= _action.actionlenth:
                del(self.action_queue[0])
                self.time_index = 0
            self.rate.sleep()


    def footvel_callback(self, msg):
        if self.is_moving:
            for i in range(4):
                self._Dog.footvel[i][0] = msg.data[i * 3 + 0]
                self._Dog.footvel[i][1] = msg.data[i * 3 + 1]
                self._Dog.footvel[i][2] = msg.data[i * 3 + 2]

    def footpoint_callback(self, msg):
        if self.is_moving:
            body_lenth = self._Dog.body_lenth
            body_width = self._Dog.body_width
            for i in range(4):
                Xside_sigh = (-1) ** i
                Yside_sign = (-1) ** (1+i//2)
                self._Dog.footpoint[i][0] = msg.data[i * 3 + 0] + Xside_sigh * body_width
                self._Dog.footpoint[i][1] = msg.data[i * 3 + 1] + Yside_sign * body_lenth
                self._Dog.footpoint[i][2] = msg.data[i * 3 + 2]

    def state_estimation_callback(self,msg):
        if self.is_moving:
            state = np.array([msg.data]).T
            self._Dog.rpy,self._Dog.body_pos,self._Dog.omega,self._Dog.body_vel = np.vsplit(state,[3,6,9])

    def status_piblish(self):

        for i in range(4):
            if self._Dog.schedualgroundLeg[i]==1:
                self.status_msg.data[i]= 0
            else:
                self.status_msg.data[i]= 1
        self.leg_status_publisher.publish(self.status_msg)

    def set_schedulegroundleg(self):
        if self._Dog.set_shedule:
            schedulegroundleg = self._Dog.schedualgroundLeg
            rospy.set_param("schedule_groundleg",schedulegroundleg)
            self._Dog.set_shedule = 0

    def force_publish(self):
        self.groundforce_msg = Float32MultiArray()
        self.groundforce_msg.data = [0 for i in range(12)]
        for i in range(4):
            if self._Dog.schedualgroundLeg[i]==1:
                self.groundforce_msg.data[3*i] = -self._Dog.force_list[i][0][0]
                self.groundforce_msg.data[3 * i+1] = -self._Dog.force_list[i][1][0]
                self.groundforce_msg.data[3 * i+2] = -self._Dog.force_list[i][2][0]
            else:
                self.groundforce_msg.data[3 * i] = 0
                self.groundforce_msg.data[3 * i + 1] = 0
                self.groundforce_msg.data[3 * i + 2] = 0
        self.force_publisher.publish(self.groundforce_msg)
    def swing_publish(self):
        self.swingleg_msg = Float32MultiArray()
        self.swingleg_msg.data = [0 for i in range(24)]
        body_lenth = self._Dog.body_lenth
        body_width = self._Dog.body_width
        for i in range(4):
            Xside_sigh = (-1) ** i
            Yside_sign = (-1) ** (1 + i // 2)
            if self._Dog.schedualgroundLeg[i]==0:
                self.swingleg_msg.data[3*i] = self._Dog.target_swingpos[i][0][0] - Xside_sigh * body_width
                self.swingleg_msg.data[3 * i+1] = self._Dog.target_swingpos[i][1][0] - Yside_sign * body_lenth
                self.swingleg_msg.data[3 * i+2] = self._Dog.target_swingpos[i][2][0]
                self.swingleg_msg.data[3 * i + 12] = self._Dog.target_swingvel[i][0][0]
                self.swingleg_msg.data[3 * i + 13] = self._Dog.target_swingvel[i][1][0]
                self.swingleg_msg.data[3 * i + 14] = self._Dog.target_swingvel[i][2][0]
            else:
                self.swingleg_msg.data[3 * i] = 0
                self.swingleg_msg.data[3 * i + 1] = 0
                self.swingleg_msg.data[3 * i + 2] = 0
                self.swingleg_msg.data[3 * i + 12] = 0
                self.swingleg_msg.data[3 * i + 13] = 0
                self.swingleg_msg.data[3 * i + 14] = 0
        self.swingleg_publisher.publish(self.swingleg_msg)


    def visual(self):
        self.looptime = time.time() - self.last_time
        self.looprostime = rospy.get_time() - self.last_rostime
        self.last_time = time.time()
        self.last_rostime = rospy.get_time()

        if not DEBUG:
            if self.time_index%1000 == 0:
                print("foot_point: ", self._Dog.footpoint)
                print("foot_vel",self._Dog.footvel)
                print("rpy: ",self._Dog.rpy)
                print("XYZ: ",self._Dog.body_pos)
                print("omega: ",self._Dog.omega)
                print("vel: ",self._Dog.body_vel)
                print("schedualgroundLeg:",self._Dog.schedualgroundLeg)
                print("phase:",self._Dog._statemachine.phase)
                print("target_state:",self._Dog.targetstates[self._Dog.stateindex])
                print("gait_time: ", self._Dog._statemachine._gait.Gait_currentTime)
                print("loop_time:", self.looptime)
                print("target_Force/Torque",self._Dog.target_force,"\n",self._Dog.target_torque)
                print("force_list",self._Dog.force_list,self._Dog.forceerror)
                print("swing_pos:",self._Dog.target_swingpos)
                print("swing_vel:", self._Dog.target_swingvel)

        else:
            if self.time_index % 10 == 0:
                print("foot_point: ", self._Dog.footpoint)
                print("foot_vel", self._Dog.footvel)
                print("rpy: ", self._Dog.rpy)
                print("XYZ: ", self._Dog.body_pos)
                print("omega: ", self._Dog.omega)
                print("vel: ", self._Dog.body_vel)
                print("schedualgroundLeg:", self._Dog.schedualgroundLeg)
                print("phase:", self._Dog._statemachine.phase)
                print("target_state:", self._Dog.targetstates[self._Dog.stateindex])
                print("gait_time: ", self._Dog._statemachine._gait.Gait_currentTime)
                print("loop_time:", self.looptime,"   ",self.looprostime)
                print("target_Force/Torque", self._Dog.target_force, "\n", self._Dog.target_torque)
                print("force_list", self._Dog.force_list, self._Dog.forceerror)
                print("swing_pos:", self._Dog.target_swingpos)
                print("swing_vel:", self._Dog.target_swingvel)


_locomotion_controller = locomotion_controlller()
_locomotion_controller.main()



