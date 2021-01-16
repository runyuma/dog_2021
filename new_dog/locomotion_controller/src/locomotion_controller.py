#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np

from dog_controller import *
from qp_controller import *
from gait_schedular import *
from state_machine import *

from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray
import math
import time

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

