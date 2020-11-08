#!/usr/bin/env python3
# coding:utf-8
import rospy
import time
from serialcommunication import *
from std_msgs.msg import Float32MultiArray,Float32
class communication_node():
    def __init__(self):
        rospy.init_node('base_communication', anonymous=True)
        self.rate = rospy.Rate(1000)  # 1000hz
        self.current_pos_stm32 = [float('nan') for i in range(12)]
        self.current_vel_stm32 = [float('nan') for i in range(12)]
        self.target_value_stm32 = [float('nan') for i in range(12)]
        #control space real motor
        self.current_pos = [float('nan') for i in range(12)]
        self.current_vel = [float('nan') for i in range(12)]
        self.target_value = [float('nan') for i in range(12)]
        #link space geometry relation
        self.target_value = [float('nan') for i in range(12)]
        self.target_mode = [4 for i in range(12)]# 0:pos// 1:vel// 2:torque// 3:ennable// 4//disable
        self.motor_id = [i for i in range(12)]
        self.serial = serialCommunication()

        self.upstreamPublisher= rospy.Publisher('/upstream', Float32MultiArray, queue_size=10)
        self.upstreammsg = Float32MultiArray(data = [float('nan')for i in range(24)])
        # 12*pos 12*vel
        self.downstreamSubscriber = rospy.Subscriber('/downstream', Float32MultiArray, self.downstreamcallback )
        # 12*mode 12 *value
    def downstreamcallback(self,msg):
        for i in range(12):
            self.target_mode = msg.data[i]
            self.target_value = msg.data[i + 12]
    def publish(self):
        for i in range(12):
            self.upstreammsg.data = self.current_pos + self.current_vel
        self.upstreamPublisher.publish(self.upstreammsg)
    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.serial.receive(self.current_pos_stm32,self.current_vel_stm32)
            self.target_mode[7] = 2
            self.target_value_stm32[7] = 30.11
            self.serial.send(self.target_value_stm32,self.target_mode)
            self.publish()

            # print(self.current_pos[7],self.current_vel[7])
    #TODO; stm32->geometry geometry->stm32

_communication_node = communication_node()
_communication_node.main()