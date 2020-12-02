#!/usr/bin/env python3
# coding:utf-8
import rospy
import time
from serialcommunication import *
from std_msgs.msg import Float32MultiArray,Float32
import numpy as np
pos_command_limit = [(-0.5,0.5), (-1, 1), (-2.5,2.5)]
torque_command_limit = [(-10,10), (-15, 15), (-15,15)]
gear_ratio  = 1.25
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
        self.target_mode = [-1 for i in range(12)]#-1:uninitialized// 0:pos// 1:vel// 2:torque
        self.motor_id = [i for i in range(12)]
        self.serial = serialCommunication()

        self.upstreamPublisher= rospy.Publisher('/upstream', Float32MultiArray, queue_size=10)
        self.upstreammsg = Float32MultiArray(data = [float('nan')for i in range(24)])
        # 12*pos 12*vel
        self.downstreamSubscriber = rospy.Subscriber('/downstream', Float32MultiArray, self.downstreamcallback )
        # 12*mode 12 *value
        for i in range(12):
            if i%3 == 0:
                self.serial.set_PD(i,15,1)
            elif i%3 == 1:
                self.serial.set_PD(i, 5, 1)
            else:
                self.serial.set_PD(i, 2, 1)
            time.sleep(0.0001)
        rospy.set_param('motor_enable',[0 for i in range(12)])
        rospy.set_param('leg_enable', [0 for i in range(4)])
        self.time_index = 0

    def downstreamcallback(self,msg):
        self.msg = msg
        for i in range(12):
            self.target_mode[i] = int(msg.data[i])
            self.target_value[i] = msg.data[i + 12]
        self.GeoToControl()
        pass

    def publish(self):
        _data = self.current_pos + self.current_vel
        self.upstreammsg.data = _data
        self.upstreamPublisher.publish(self.upstreammsg)

    def main(self):
        while not rospy.is_shutdown():
            self.motor_enable_list = rospy.get_param('motor_enable')
            self.leg_enable_list = rospy.get_param("leg_enable")
            self.set_able()
            self.serial.receive(self.current_pos_stm32,self.current_vel_stm32)
            self.SensorsToGeo()
            self.serial.send(self.target_value_stm32, self.target_mode, self.motor_enable_list)
            self.publish()

            _motor_enable_list = rospy.get_param("motor_enable")
            if self.motor_enable_list != _motor_enable_list:
                rospy.set_param('motor_enable',self.motor_enable_list)

            if self.time_index % 1000 == 0:
                print( "pos, vel ",self.current_pos_stm32,self.current_vel_stm32)
                print('enable_list',self.leg_enable_list,self.motor_enable_list)
                print("target",self.target_mode,self.target_value)
            self.time_index +=1

            self.rate.sleep()

    def set_able(self):
        #to make the motor enable or diable
        for i in range(12):
            if self.motor_enable_list[i] != self.leg_enable_list[i//3]:
                if self.leg_enable_list[i//3] == 1:
                    self.serial.send_enable(i)
                    # TODO:enable ensurence
                    self.motor_enable_list[i] = 1

                elif self.leg_enable_list[i//3] == 0:
                    self.serial.send_disable(i)
                    # TODO:disable ensurence
                    self.motor_enable_list[i] = 0

    def GeoToControl(self):
        # the upperconcole message mapping to the real control value
        pos_propotion = [-1,-1,-1.25,1,1,1,1,1,1,1,1,1]
        vel_propotion = [-1,-1,-1.25,1,1,1,1,1,1,1,1,1]
        tor_propotion = [-1,-1,-0.8,1,1,1,1,1,1,1,1,1]
        pos_bias = [0,0,0,0,0,0,0,0,0,0,0,0]
        vel_bias = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        tor_bias = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        for i in range(12):
            if self.target_mode[i] == -1:
                pass
            #TODO:transformation
            elif self.target_mode[i] == 0:
                self.target_value_stm32[i] = pos_propotion[i] * self.target_value[i] + pos_bias[i]
                if self.target_value_stm32[i]<= pos_command_limit[i%3][0]:
                    self.target_value_stm32[i] = pos_command_limit[i%3][0]
                elif self.target_value_stm32[i]>= pos_command_limit[i%3][1]:
                    self.target_value_stm32[i] = pos_command_limit[i % 3][1]
                pass
            elif self.target_mode[i] == 1:
                self.target_value_stm32[i] = vel_propotion[i] * self.target_value[i] + vel_bias[i]
            elif self.target_mode[i] == 2:
                self.target_value_stm32[i] = tor_propotion[i] * self.target_value[i] + tor_bias[i]
                if self.target_value_stm32[i]<= torque_command_limit[i%3][0]:
                    self.target_value_stm32[i] = torque_command_limit[i%3][0]
                elif self.target_value_stm32[i]>= torque_command_limit[i%3][1]:
                    self.target_value_stm32[i] = torque_command_limit[i % 3][1]

    def SensorsToGeo(self):
        # real value mapping to upper concole joint space
        _sensor_array = self.current_pos_stm32+self.current_vel_stm32
        propotion = np.array([-1.,-1.,-0.8,1.,1.,1.,1.,1.,1.,1.,1.,1.,-1.,-1.,-0.8,1.,1.,1.,1.,1.,1.,1.,1.,1.])
        bias = np.array([0. for i in range(24)])
        res = propotion * _sensor_array + bias
        _current_pos,_current_vel = np.array_split(res,[12])
        self.current_pos = _current_pos.tolist()
        self.current_vel = _current_vel.tolist()

        # print(self.current_pos[7],self.current_vel[7])
    #TODO; stm32->geometry geometry->stm32

_communication_node = communication_node()
_communication_node.main()