#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np

from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray

class state_estimation():
    def __init__(self):
        rospy.init_node("state_estimation",anonymous= True)
        self.rate = rospy.Rate(500)
        self.state_estimation_mode = rospy.get_param("state_estimation_mode")# 0 is getfrom gezebo, 1 is pure leg dynamic, 2 is extended kalman fillter
        if self.state_estimation_mode == 0:
            self.Imu_Subscriber = rospy.Subscriber("/imu", Imu, self.Imu_callback)
            self.state_Subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.state_callback)
            self.got_imu = 0
            self.got_xyz = 0
        self.state_publisher = rospy.Publisher("/state",Float32MultiArray,queue_size = 10)
        self.rpy = np.zeros((3,1))
        self.omega = np.zeros((3, 1))
        self.linear_acceleration = np.zeros((3,1))
        self.body_pos = np.zeros((3,1))
        self.body_vel = np.zeros((3,1))

    def main(self):
        while not rospy.is_shutdown():
            if self.state_estimation_mode == 0:
                self.state_publish()
            self.rate.sleep()

    def state_publish(self):
        if self.state_estimation_mode == 0:
            if self.got_xyz and self.got_imu:
                self.STATE = np.vstack([self.rpy,self.body_pos,self.omega, self.body_vel])
                state = Float32MultiArray()
                state.data = self.STATE.T[0].tolist()
                self.state_publisher.publish(state)




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
        self.rpy[1][0] = np.arcsin(-2 * (x * z - w * y))
        self.rpy[2][0] = np.arctan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z)


    def state_callback(self,msg):
        self.got_xyz = 1
        # Read the position of the robot
        # print("state_callback")
        self.body_pos[0][0] = msg.pose[1].position.x
        self.body_pos[1][0] = msg.pose[1].position.y
        self.body_pos[2][0] = msg.pose[1].position.z
        # print("state_callback working")

        # Read the Vel of the robot
        self.body_vel[0][0] = msg.twist[1].linear.x
        self.body_vel[1][0] = msg.twist[1].linear.y
        self.body_vel[2][0] = msg.twist[1].linear.z
_state_estimation = state_estimation()
_state_estimation.main()