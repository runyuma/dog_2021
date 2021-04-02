import rospy
import time
import numpy as np
import pandas as pd
import copy
from statemechine import *
from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray


class Sensor():
    def __init__(self):
        self.sensor_subscriber = rospy.Subscriber('/upstream', Float32MultiArray, self.sensor_callback)
        self.joint_pos = [[None, None, None] for i in range(4)]
        self.joint_vel = [[None, None, None] for i in range(4)]

    def sensor_callback(self, msg):
        _joint_pos, _joint_vel = msg.data[:12], msg.data[12:]
        self.joint_pos = [i.tolist() for i in np.split(_joint_pos, [3, 6, 9])]
        self.joint_vel = [i.tolist() for i in np.split(_joint_vel, [3, 6, 9])]
        for i in range(12):
            if np.isnan(_joint_pos[i]):
                self.joint_pos[i // 3][i % 3] = None
                self.joint_vel[i // 3][i % 3] = None
