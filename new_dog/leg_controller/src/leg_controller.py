#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray,Float32,Int32MultiArray
from leg_dynamics import *
import time


class leg_controller():
    def __init__(self):
        rospy.init_node('leg_controller', anonymous=True)
        self.rate = rospy.Rate(1000)
        self.sensor_subscriber = rospy.Subscriber('/upstream', Float32MultiArray, self.sensor_callback)
        self.force_subscriber = rospy.Subscriber('/ground_force',Float32MultiArray, self.groundforce_callback)
        self.swingleg_subscriber = rospy.Subscriber('/swing_leg',Float32MultiArray,self.swingleg_callback)
        self.leg_status_subscriber = rospy.Subscriber('/leg_status', Int32MultiArray, self.leg_status_callback)
        # sensor:lower computer joint message
        # ground_force: lenth:12 4*[x,y,z] [0,0,0] means no ground force control
        # swing_leg: lenth:24 4*[x,y,z] 4*[vx,vy,vz] [0,0,0] means no swing force control
        # leg_status: -1:uninitialized do nothing//0: ground//1:swing//2:pos(data in ground force topic)// 5: jointpos//6:joint_torque

        self.footpoint_publisher = rospy.Publisher("/foot_points",Float32MultiArray, queue_size=10)
        self.footvel_publisher = rospy.Publisher('/foot_vel',Float32MultiArray, queue_size=10)
        #foot_point pos of foot
        #foot_vel vel of foot(might be a big error when using ground force control)

        self.foot_points = [np.array([[float('nan') ],[float('nan') ],[float('nan') ]])for i in range(4)]
        self.foot_vel = [np.array([[float('nan') ],[float('nan') ],[float('nan') ]]) for i in range(4)]
        self.footpoint_pubmsg = Float32MultiArray()
        self.footvel_pubmsg = Float32MultiArray()

        self.target_force = [[None,None,None] for i in range(4)]
        self.target_swing = [[None,None,None]for i in range(8)]
        self.joint_pos = [[None,None,None] for i in range(4)]
        self.joint_vel = [[None,None,None] for i in range(4)]
        self.leg_status = [-1,-1,-1,-1]
        # the list above is used for subscribe

        self.jointtarget_publisher = rospy.Publisher('/downstream', Float32MultiArray, queue_size=10)
        self.target_mode = [-1 for i in range(12)]
        self.target_value = [0. for i in range(12)]
        self.jointtargetmsg = Float32MultiArray(data = self.target_mode + self.target_value )
        #the list above is for publised to basenode about the mode of control(pos vel torque)

        leg_mass = [rospy.get_param('hip_mass'),rospy.get_param('upper_link_mass'),rospy.get_param('lower_link_mass')]
        lenth_list = [rospy.get_param('hip_lenth'),rospy.get_param('upper_link_lenth'),rospy.get_param('lower_link_lenth')]
        discomlist = [0,rospy.get_param('comb_lenth'),rospy.get_param('comc_lenth')]
        self.leg_args = leg_args(leg_mass,lenth_list,discomlist)

        self.time_index = 0

    def main(self):
        while not rospy.is_shutdown():
            start_time = time.time()
            for i in range(4):
                if i%2 == 0:
                    LorR = 1
                else:
                    LorR = -1
                #LorR: left or right
                _joint_pos = self.joint_pos[i]
                _joint_vel = self.joint_vel[i]
                if not None in _joint_pos and not None in _joint_vel:
                    # if have got value
                    self.foot_points[i] = get_footpointpos(LorR,_joint_pos,self.leg_args)
                    jacobian = get_jacobian(LorR,_joint_pos,self.leg_args)
                    self.foot_vel[i] = get_footpointvel(jacobian,_joint_vel)
                    # foot point & foot vel all are np.ndarry
                    if self.leg_status[i] == -1:
                        self.target_mode[3*i:3*i+3] = [-1,-1,-1]
                        # do nothing
                    elif self.leg_status[i] == 0:
                        # ground force
                        if not None in self.target_force[i]:
                            _Force = np.array([self.target_force[i]]).T
                            _joint_torque = np.dot(jacobian.T,_Force)
                            self.target_value[3*i:3*i+3] = _joint_torque.T.tolist()[0]
                            self.target_mode[3*i:3*i+3] = [2,2,2]

                        else:
                            print("leg",i ,"didnt get target force")
                    elif self.leg_status[i] == 1:
                        #swing leg
                        if not None in self.target_swing[i]:
                            kp = np.diag([100,50,180])
                            kd = np.diag([10,5,20])
                            # kd = np.diag(rospy.get_param("swingleg_D"))
                            fun_start_time = time.time()
                            _joint_torque = get_tauff(LorR,_joint_pos,_joint_vel,self.leg_args) + np.dot(jacobian.T,get_feedbackward(kp,kd,self.foot_points[i],self.foot_vel[i],np.array(self.target_swing[i]).reshape(3,1),np.array(self.target_swing[i+4]).reshape(3,1)))
                            # print(_joint_torque,self.foot_points[i],self.target_swing[i],"***************")
                            self.target_value[3*i:3*i+3] = _joint_torque.T.tolist()[0]
                            self.target_mode[3*i:3*i+3] = [2,2,2]
                            if self.time_index%1000 == 0:
                                print(time.time() - fun_start_time)
                                # average 0.5ms
                        else:
                            print("leg",i ,"didnt get target force")
                    elif self.leg_status[i] == 5:
                        if not None in self.target_force[i]:
                            self.target_value[3*i:3*i+3] = self.target_force[i]
                            self.target_mode[3*i:3*i+3] = [0,0,0]
                        else:
                            print("leg",i ,"didnt get target joint_pos")
                    elif self.leg_status[i] == 6:
                        if not None in self.target_force[i]:
                            self.target_value[3*i:3*i+3] = self.target_force[i]
                            self.target_mode[3*i:3*i+3] = [2,2,2]
                        else:
                            print("leg",i ,"didnt get target joint_tor")

                else:
                    if self.time_index%10000 == 0:
                        pass
                        # print("leg:",i," not connected")

            self.jointtarget_publish()
            self.footpoint_publish()
            self.time_index+= 1
            if self.time_index % 1000 == 0:
                print("loop_time:",time.time() - start_time)
            self.rate.sleep()

    def sensor_callback(self,msg):
        _joint_pos,_joint_vel= msg.data[:12], msg.data[12:]
        self.joint_pos = [i.tolist() for i in np.split(_joint_pos,[3,6,9])]
        self.joint_vel = [i.tolist() for i in np.split(_joint_vel, [3, 6, 9])]
        for i in range(12):
            if np.isnan(_joint_pos[i]):
                self.joint_pos[i//3][i%3] = None
                self.joint_vel[i//3][i%3] = None

    def groundforce_callback(self,msg):
        _target_force = np.split(msg.data,[3,6,9])
        self.target_force = [i.tolist() for i in _target_force]

    def swingleg_callback(self,msg):
        _target_pos = np.split(msg.data[:12],[3,6,9])
        _target_vel = np.split(msg.data[12:],[3,6,9])
        for i in range(4):
            self.target_swing[i] = _target_pos[i].tolist()
            self.target_swing[i+4] = _target_vel[i].tolist()

    def leg_status_callback(self,msg):
        self.leg_status = msg.data

    def jointtarget_publish(self):
        self.jointtargetmsg.data = self.target_mode + self.target_value
        self.jointtarget_publisher.publish(self.jointtargetmsg)

    def footpoint_publish(self):
        _footpoint = []
        _footvel = []
        for i in range(4):
            _footpoint += self.foot_points[i].T[0].tolist()
            _footvel += self.foot_vel[i].T[0].tolist()
        self.footpoint_pubmsg.data = _footpoint
        self.footvel_pubmsg.data = _footvel
        self.footpoint_publisher.publish(self.footpoint_pubmsg)
        self.footvel_publisher.publish(self.footvel_pubmsg)
_leg_controller = leg_controller()
_leg_controller.main()

