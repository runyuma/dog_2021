#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np


from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray

USE_SIM = 1
USE_TOUCHSENSOR = 1
if USE_SIM:
    from sensor_msgs.msg import Imu
    from gazebo_msgs.msg import ModelStates
    from gazebo_msgs.msg import ContactsState
class state_estimation():
    def __init__(self):
        rospy.init_node("state_estimation",anonymous= True)
        self.rate = rospy.Rate(500)
        self.body_lenth = rospy.get_param("body_lenth")
        self.body_width = rospy.get_param("body_width")
        self.state_estimation_mode = rospy.get_param("state_estimation_mode")# 0 is getfrom gezebo, 1 is pure leg dynamic, 2 is extended kalman fillter
        if self.state_estimation_mode == 0:
            self.Imu_Subscriber = rospy.Subscriber("/imu", Imu, self.Imu_callback)
            self.state_Subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.state_callback)
            self.got_imu = 0
            self.got_xyz = 0
        else:
            if USE_SIM:
                self.Imu_Subscriber = rospy.Subscriber("/imu", Imu, self.Imu_callback)
                topic_name = ["/left_front_contact_sensor","/right_front_contact_sensor","/left_back_contact_sensor","/right_back_contact_sensor"]
                callback_fun = [self.left_front_callback,self.right_front_callback,self.left_back_callback,self.right_back_callback]
                self.touchsensors = [None,None,None,None]
                for i in range(4):
                    self.touchsensors[i] = rospy.Subscriber(topic_name[i],ContactsState,callback_fun[i])
        self.footpoint_subscriber = rospy.Subscriber("/foot_points", Float32MultiArray, self.footpoint_callback)
        self.state_publisher = rospy.Publisher("/state",Float32MultiArray,queue_size = 10)
        self.contact_state = [0,0,0,0]
        self.touchsensors = [np.zeros(3) for i in range(4)]
        self.rpy = np.zeros((3,1))
        self.omega = np.zeros((3, 1))
        self.linear_acceleration = np.zeros((3,1))
        self.body_pos = np.zeros((3,1))
        self.body_vel = np.zeros((3,1))
        self.footpoint = [np.zeros((3,1)) for i in range(4)]
        rospy.set_param("USE_TOUCHSENSOR",USE_TOUCHSENSOR)

    def main(self):
        while not rospy.is_shutdown():
            if self.state_estimation_mode == 0:
                self.state_publish()
            elif self.state_estimation_mode == 1:
                pass
            self.rate.sleep()

    def state_publish(self):
        if self.state_estimation_mode == 0:
            if self.got_xyz and self.got_imu:
                self.STATE = np.vstack([self.rpy,self.body_pos,self.omega, self.body_vel])
                state = Float32MultiArray()
                state.data = self.STATE.T[0].tolist()
                self.state_publisher.publish(state)

    def get_imu(self):
        pass
    def get_touchsensor(self):
        pass # TODO:touchsensor real_robot
    def left_front_callback(self,effort_message):
        foot_index = 0
        if effort_message.states == []:
            self.contact_state[foot_index] = 0
            self.touchsensors[foot_index] = np.array([[0], [0], [0]])
        else:
            X = []
            Y = []
            Z = []
            self.contact_state = 1
            for i in effort_message.states:
                X.append(i.total_wrench.force.x)
                Y.append(i.total_wrench.force.y)
                Z.append(i.total_wrench.force.z)
            self.touchsensors[foot_index] = np.array([np.mean(X), np.mean(Y), np.mean(Z)])
    def right_front_callback(self, effort_message):
        foot_index = 1
        if effort_message.states == []:
            self.contact_state[foot_index] = 0
            self.touchsensors[foot_index] = np.array([[0], [0], [0]])
        else:
            X = []
            Y = []
            Z = []
            self.contact_state = 1
            for i in effort_message.states:
                X.append(i.total_wrench.force.x)
                Y.append(i.total_wrench.force.y)
                Z.append(i.total_wrench.force.z)
            self.touchsensors[foot_index] = np.array([np.mean(X), np.mean(Y), np.mean(Z)])
    def left_back_callback(self, effort_message):
        foot_index = 2
        if effort_message.states == []:
            self.contact_state[foot_index] = 0
            self.touchsensors[foot_index] = np.array([[0], [0], [0]])
        else:
            X = []
            Y = []
            Z = []
            self.contact_state = 1
            for i in effort_message.states:
                X.append(i.total_wrench.force.x)
                Y.append(i.total_wrench.force.y)
                Z.append(i.total_wrench.force.z)
            self.touchsensors[foot_index] = np.array([np.mean(X), np.mean(Y), np.mean(Z)])
    def right_back_callback(self, effort_message):
        foot_index = 3
        if effort_message.states == []:
            self.contact_state[foot_index] = 0
            self.touchsensors[foot_index] = np.array([[0], [0], [0]])
        else:
            X = []
            Y = []
            Z = []
            self.contact_state = 1
            for i in effort_message.states:
                X.append(i.total_wrench.force.x)
                Y.append(i.total_wrench.force.y)
                Z.append(i.total_wrench.force.z)
            self.touchsensors[foot_index] = np.array([np.mean(X), np.mean(Y), np.mean(Z)])
    def footpoint_callback(self, msg):
        body_lenth = self.body_lenth
        body_width = self.body_width
        for i in range(4):
            Xside_sigh = (-1) ** i
            Yside_sign = (-1) ** (1+i//2)
            self.footpoint[i][0][0] = msg.data[i * 3 + 0] + Xside_sigh * body_width
            self.footpoint[i][1][0] = msg.data[i * 3 + 1] + Yside_sign * body_lenth
            self.footpoint[i][2][0] = msg.data[i * 3 + 2]
    def Imu_callback(self,msg):
        self.got_imu = 1
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Read the angular velocity of the robot IMU
        self.omega[0][0] = msg.angular_velocity.x
        self.omega[1][0] = msg.angular_velocity.y
        self.omega[2][0] = msg.angular_velocity.z

        # Read the linear acceleration of the robot IMU
        self.linear_acceleration[0][0] = msg.linear_acceleration.x
        self.linear_acceleration[1][0] = msg.linear_acceleration.y
        self.linear_acceleration[2][0] = msg.linear_acceleration.z

        # Convert Quaternions to Euler-Angles Z-Y-X
        rpy_angle = [0, 0, 0]  # XYZ

        self.rpy[0][0] = np.arctan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z)
        self.rpy[1][0] = np.arcsin(2*(w*y - x*z))
        self.rpy[2][0] = np.arctan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z)


    def state_callback(self,msg):
        self.got_xyz = 1
        # Read the position of the robot
        # print("state_callback")
        self.body_pos[0][0] = msg.pose[1].position.x
        self.body_pos[1][0] = msg.pose[1].position.y
        self.body_pos[2][0] = msg.pose[1].position.z-0.05
        # print("state_callback working")

        # Read the Vel of the robot
        self.body_vel[0][0] = msg.twist[1].linear.x
        self.body_vel[1][0] = msg.twist[1].linear.y
        self.body_vel[2][0] = msg.twist[1].linear.z
_state_estimation = state_estimation()
_state_estimation.main()