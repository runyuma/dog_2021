#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np
import copy
import time

from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray
TEST = 1
USE_SIM = 1
USE_TOUCHSENSOR = 1
if TEST:
    import pandas as pd
if USE_SIM:
    from sensor_msgs.msg import Imu
    from gazebo_msgs.msg import ModelStates
    from gazebo_msgs.msg import ContactsState
class state_estimation():
    def __init__(self):
        rospy.init_node("state_estimation",anonymous= True)
        self.Hz = 500
        self.rate = rospy.Rate(self.Hz)
        self.body_lenth = rospy.get_param("body_lenth")
        self.body_width = rospy.get_param("body_width")
        self.state_estimation_mode = rospy.get_param("state_estimation_mode")# 0 is getfrom gezebo, 1 is pure leg dynamic, 2 is extended kalman fillter

        self.contact_state = [0,0,0,0]

        self.touchsensors = [np.zeros(3) for i in range(4)]
        self.rpy = np.zeros((3,1))
        self.omega = np.zeros((3, 1))
        self.linear_acceleration = np.zeros((3,1))
        self.body_pos = np.zeros((3,1))
        self.body_vel = np.zeros((3,1))
        self.footpoint = np.zeros((3,4))
        self.foot_vel = np.zeros((3,4))

        self.initialed = 0 # initialized or not
        self.foot_point_received = 0
        self.foot_vel_received = 0
        self.got_imu = 0
        self.current_gait = rospy.get_param("current_gait")
        rospy.set_param("USE_TOUCHSENSOR",USE_TOUCHSENSOR)
        self.time_index = 0
        if TEST:
            self.df_colomn = ["time",
                              "gait",
                              "gazebo_r",
                              "gazebo_p",
                              "gazebo_y",
                              "gazebo_x",
                              "gazebo_y",
                              "gazebo_z",
                              "gazebo_wx",
                              "gazebo_wy",
                              "gazebo_wz",
                              "gazebo_vx",
                              "gazebo_vy",
                              "gazebo_vz",
                              "r",
                              "p",
                              "y",
                              "x",
                              "y",
                              "z",
                              "wx",
                              "wy",
                              "wz",
                              "vx",
                              "vy",
                              "vz",
                              ]
            self.df_data = np.zeros((1,26))
            self.test_body_pos = np.zeros((3, 1))
            self.test_body_vel = np.zeros((3, 1))
            self.test_rpy = np.zeros((3, 1))
            self.test_omega = np.zeros((3, 1))

            if self.state_estimation_mode == 0 or TEST:
                self.Imu_Subscriber = rospy.Subscriber("/imu", Imu, self.Imu_callback)
                self.state_Subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, self.state_callback)
                self.got_xyz = 0
            if USE_SIM:
                self.Imu_Subscriber = rospy.Subscriber("/imu", Imu, self.Imu_callback)
                topic_name = ["/left_front_contact_sensor", "/right_front_contact_sensor", "/left_back_contact_sensor",
                              "/right_back_contact_sensor"]
                callback_fun = [self.left_front_callback, self.right_front_callback, self.left_back_callback,
                                self.right_back_callback]
                self.touchsensors = [None, None, None, None]
                for i in range(4):
                    self.touchsensors[i] = rospy.Subscriber(topic_name[i], ContactsState, callback_fun[i])
            self.footpoint_subscriber = rospy.Subscriber("/foot_points", Float32MultiArray, self.footpoint_callback)
            self.footvel_subscriber = rospy.Subscriber("/foot_vel", Float32MultiArray, self.footvel_callback)
            self.state_publisher = rospy.Publisher("/state", Float32MultiArray, queue_size=10)

    def main(self):
        while not rospy.is_shutdown():
            if self.state_estimation_mode == 0:
                self.state_publish()
                if USE_TOUCHSENSOR:
                    rospy.set_param("contact_state", self.contact_state)
                if TEST:
                    if not self.initialed:
                        if self.foot_point_received and self.got_imu:
                            self.initial()
                            self.rate.sleep()
                    else:
                        self.current_time = rospy.get_time()
                        self.looptime = self.current_time - self.last_rostime
                        self.last_rostime = self.current_time
                        self.leg_dynamic_estimation()
                    if self.time_index%5 == 0:
                        self.restore_df_data()
                    if self.time_index % 5000 == 0:
                        self.restore_dataframe()
                # if self.time_index % 50 == 0:
                #     print(self.linear_acceleration)
            elif self.state_estimation_mode == 1:
                if not self.initialed:
                    if self.foot_point_received and self.got_imu :
                        self.initial()
                        self.state_publish()
                else:
                    self.current_time = rospy.get_time()
                    self.looptime = self.current_time - self.last_rostime
                    self.last_rostime = self.current_time
                    self.leg_dynamic_estimation()
                    self.state_publish()

                    # print("loop_time:", self.looptime)
                    # print("body_pos:", self.body_pos)
                    # print("body_vel", self.body_vel)
                if TEST:
                    if self.time_index%5 == 0:
                        self.restore_df_data()
                    if self.time_index % 5000 == 0:
                        self.restore_dataframe()
            self.rate.sleep()
            self.time_index += 1

    def initial(self):
        C_mat = self.get_TFmat()
        self.footpoint_W = np.dot(C_mat,self.footpoint)
        height = -np.mean(self.footpoint_W[2])
        self.body_pos = np.zeros((3, 1))
        self.body_vel = np.zeros((3, 1))
        self.body_pos[2][0] = height
        self.foot_contactpoint = np.dot(self.body_pos ,np.ones((1,4))) + self.footpoint_W
        self.schedule_leg = [1,1,1,1]
        self.last_schedule_leg = [1,1,1,1]
        self.last_body_pos = copy.deepcopy(self.body_pos)
        self.last_rostime = rospy.get_time()
        self.initialed = 1
        self.body_pos_memory = [copy.deepcopy(self.body_pos)]
        self.time_intervals = [rospy.get_time()]
        self.memory_lenth = 20
        if TEST:
            self.test_body_pos = np.zeros((3, 1))
            self.test_body_vel = np.zeros((3, 1))
            self.test_rpy = np.zeros((3, 1))
            self.test_omega = np.zeros((3, 1))

    def leg_dynamic_estimation(self):
        # cauculate pos
        plan1 = 0
        plan2 = 0
        plan3 = 0
        plan4 = 1

        self.schedule_leg = rospy.get_param("schedule_groundleg")
        C_mat = self.get_TFmat()
        self.footpoint_W = np.dot(C_mat, self.footpoint)
        if USE_TOUCHSENSOR:
            rospy.set_param("contact_state", self.contact_state)
        # update contact point
        if self.state_estimation_mode == 1:
            if not plan4:
                if self.last_schedule_leg != self.schedule_leg:
                    still_contact = np.array(self.last_schedule_leg) * np.array(self.schedule_leg)
                    new_contact = np.array([1,1,1,1]) - still_contact
                    self.foot_contactpoint = self.foot_contactpoint * still_contact + (np.dot(self.body_pos ,np.ones((1,4))) + self.footpoint_W) * new_contact
                    self.last_schedule_leg = self.schedule_leg
                # cauculate pos

                self.body_pos = np.sum((self.foot_contactpoint - self.footpoint_W) * np.array(self.schedule_leg),axis=1).reshape((3,1))/sum(self.schedule_leg)
                self.body_vel = (self.body_pos - self.body_pos_memory[0]) / (rospy.get_time() - self.time_intervals[0])
                if len(self.body_pos_memory)< self.memory_lenth:
                    self.body_pos_memory.append(copy.deepcopy(self.body_pos))
                    self.time_intervals.append(rospy.get_time())
                else:
                    self.body_pos_memory = self.body_pos_memory[1:] + [copy.deepcopy(self.body_pos)]
                    self.time_intervals = self.time_intervals[1:] + [rospy.get_time()]
                self.last_body_pos = copy.deepcopy(self.body_pos)
            elif plan4:
                cal_array = np.array(copy.copy(self.last_schedule_leg))
                if self.last_schedule_leg != self.schedule_leg:
                    if self.contact_state == [0, 0, 0, 0]:
                        self.body_vel += (self.looptime) * (
                                    np.dot(C_mat, self.linear_acceleration) + np.array([[0], [0], [-9.81]]))
                        self.body_pos += (self.looptime) * self.body_pos
                    else:
                        permit = 1
                        for i in range(4):
                            if self.last_schedule_leg[i] == 0 and self.schedule_leg[i] == 1:
                                if self.contact_state[i] == 1:
                                    cal_array[i] = 1
                                    point = self.foot_contactpoint[:, i:i + 1]
                                    if (point == np.zeros((3, 1))).all():
                                        self.foot_contactpoint[:, i:i + 1] = self.body_pos + self.footpoint_W[:,
                                                                                                  i:i + 1]
                                        print("contact")
                                else:
                                    permit = 0
                            if self.last_schedule_leg[i] == 1 and self.schedule_leg[i] == 0:
                                if self.contact_state[i] == 0:
                                    self.foot_contactpoint[:, i:i + 1] = np.zeros((3, 1))
                                    cal_array[i] = 0
                                    print("loss conract")
                        if permit:
                            for i in range(4):
                                if self.last_schedule_leg[i] == 1 and self.schedule_leg[i] == 0:
                                    self.foot_contactpoint[:, i:i + 1] = np.zeros((3, 1))
                                elif self.last_schedule_leg[i] == 0 and self.schedule_leg[i] == 1:
                                    self.foot_contactpoint[:, i:i + 1] = self.foot_contactpoint[:, i:i + 1] - np.array([[0], [0], [0.0046]])
                            self.last_schedule_leg = self.schedule_leg
                            cal_array = np.array(copy.copy(self.last_schedule_leg))

                            print("*************************change")
                            print(self.body_pos)
                            print("cal_array: ", cal_array, self.schedule_leg)
                            print("contact_point:", self.foot_contactpoint)
                            print("foot_point", self.footpoint_W)
                            print("TF", C_mat, self.rpy)
                        self.body_pos = np.sum((self.foot_contactpoint - self.footpoint_W) * cal_array,
                                                    axis=1).reshape((3, 1)) / sum(cal_array.tolist())
                        self.body_vel = (self.body_pos - self.body_pos_memory[0]) / (
                                    rospy.get_time() - self.time_intervals[0])

                else:
                    self.body_pos = np.sum((self.foot_contactpoint - self.footpoint_W) * cal_array,
                                                axis=1).reshape((3, 1)) / sum(cal_array.tolist())
                    self.body_vel = (self.body_pos - self.body_pos_memory[0]) / (
                                rospy.get_time() - self.time_intervals[0])
                if self.last_schedule_leg != self.schedule_leg:
                    print(cal_array)
                    print(self.foot_contactpoint)
                    print("pos", self.body_pos)
                    print("vel: ", self.body_vel)
                if len(self.body_pos_memory) < self.memory_lenth:
                    self.body_pos_memory.append(copy.deepcopy(self.body_pos))
                    self.time_intervals.append(rospy.get_time())
                else:
                    self.body_pos_memory = self.body_pos_memory[1:] + [copy.deepcopy(self.body_pos)]
                    self.time_intervals = self.time_intervals[1:] + [rospy.get_time()]

                self.last_body_pos = copy.deepcopy(self.body_pos)
        elif TEST and self.state_estimation_mode == 0:
            if not plan4:
                if self.last_schedule_leg != self.schedule_leg:
                    still_contact = np.array(self.last_schedule_leg) * np.array(self.schedule_leg)
                    new_contact = np.array([1, 1, 1, 1]) - still_contact
                    self.foot_contactpoint = self.foot_contactpoint * still_contact + (
                                np.dot(self.test_body_pos, np.ones((1, 4))) + self.footpoint_W) * new_contact
                    self.last_schedule_leg = self.schedule_leg

                if not plan3:
                    self.test_body_pos = np.sum((self.foot_contactpoint - self.footpoint_W) * np.array(self.schedule_leg),
                                           axis=1).reshape((3, 1)) / sum(self.schedule_leg)

                if plan1:
                    self.test_body_vel = (self.test_body_pos - self.body_pos_memory[0]) / (rospy.get_time() - self.time_intervals[0])
                    if len(self.body_pos_memory) < self.memory_lenth:
                        self.body_pos_memory.append(copy.deepcopy(self.test_body_pos))
                        self.time_intervals.append(rospy.get_time())
                    else:
                        self.body_pos_memory = self.body_pos_memory[1:] + [copy.deepcopy(self.test_body_pos)]
                        self.time_intervals = self.time_intervals[1:] + [rospy.get_time()]
                elif plan2:
                    self.foot_vel_B = np.dot(omega_matrix(self.omega),self.footpoint) + self.foot_vel
                    self.test_body_vel = - np.sum(np.dot(C_mat, self.footpoint_W) * np.array(self.contact_state),
                                                axis=1).reshape((3, 1)) / sum(self.contact_state)
                elif plan3:
                    self.test_body_vel  += (self.looptime) * (np.dot(C_mat,self.linear_acceleration) + np.array([[0],[0],[-9.81]]))
                    self.test_body_pos += (self.looptime) * self.test_body_vel

                self.last_body_pos = copy.deepcopy(self.test_body_pos)
            if plan4:
                cal_array = np.array(copy.copy(self.last_schedule_leg))
                if self.last_schedule_leg != self.schedule_leg:
                    if self.contact_state == [0,0,0,0]:
                        self.test_body_vel += (self.looptime) * (np.dot(C_mat,self.linear_acceleration) + np.array([[0],[0],[-9.81]]))
                        self.test_body_pos += (self.looptime) * self.test_body_pos
                    else:
                        permit = 1
                        for i in range(4):
                            if self.last_schedule_leg[i] == 0 and self.schedule_leg[i] == 1:
                                if self.contact_state[i] == 1 :
                                    cal_array[i] = 1
                                    point = self.foot_contactpoint[:,i:i+1]
                                    if  (point == np.zeros((3,1))).all():
                                        self.foot_contactpoint[:,i:i+1] = self.test_body_pos + self.footpoint_W[:,i:i+1]
                                        print("contact")
                                else:
                                    permit = 0
                            if self.last_schedule_leg[i] == 1 and self.schedule_leg[i] == 0:
                                if self.contact_state[i] == 0:
                                    self.foot_contactpoint[:, i:i + 1] = np.zeros((3,1))
                                    cal_array[i] = 0
                                    print("loss conract")
                        if permit:
                            for i in range(4):
                                if self.last_schedule_leg[i] == 1 and self.schedule_leg[i] == 0:
                                    self.foot_contactpoint[:, i:i + 1] = np.zeros((3, 1))
                                elif self.last_schedule_leg[i] == 0 and self.schedule_leg[i] == 1:
                                    self.foot_contactpoint[:, i:i + 1] = self.foot_contactpoint[:, i:i + 1] - np.array([[0],[0],[0.0039]])
                            self.last_schedule_leg = self.schedule_leg
                            cal_array = np.array(copy.copy(self.last_schedule_leg))

                            print("*************************change")
                            print(self.test_body_pos)
                            print("cal_array: ",cal_array,self.schedule_leg)
                            print("contact_point:",self.foot_contactpoint)
                            print("foot_point",self.footpoint_W)
                            print("TF",C_mat,self.rpy)
                        self.test_body_pos = np.sum((self.foot_contactpoint - self.footpoint_W) * cal_array,axis=1).reshape((3, 1)) / sum(cal_array.tolist())
                        self.test_body_vel = (self.test_body_pos - self.body_pos_memory[0]) / (rospy.get_time() - self.time_intervals[0])

                else:
                    self.test_body_pos = np.sum((self.foot_contactpoint - self.footpoint_W) * cal_array,
                                                axis=1).reshape((3, 1)) / sum(cal_array.tolist())
                    self.test_body_vel = (self.test_body_pos - self.body_pos_memory[0]) / (rospy.get_time() - self.time_intervals[0])
                if self.last_schedule_leg != self.schedule_leg:
                    print(cal_array)
                    print(self.foot_contactpoint)
                    print("pos",self.test_body_pos)
                    print("vel: ",self.test_body_vel)
                if len(self.body_pos_memory) < self.memory_lenth:
                    self.body_pos_memory.append(copy.deepcopy(self.test_body_pos))
                    self.time_intervals.append(rospy.get_time())
                else:
                    self.body_pos_memory = self.body_pos_memory[1:] + [copy.deepcopy(self.test_body_pos)]
                    self.time_intervals = self.time_intervals[1:] + [rospy.get_time()]

                self.last_body_pos = copy.deepcopy(self.test_body_pos)
    def restore_df_data(self):
        _data = np.zeros(26)
        _data[0] = rospy.get_time()
        _data[1] = self.current_gait
        _data[2:5] = self.test_rpy.reshape(3)
        _data[5:8] = self.test_body_pos.reshape(3)
        _data[8:11] = self.test_omega.reshape(3)
        _data[11:14] = self.test_body_vel.reshape(3)
        _data[14:17] = self.rpy.reshape(3)
        _data[17:20] = self.body_pos.reshape(3)
        _data[20:23] = self.omega.reshape(3)
        _data[23:26] = self.body_vel.reshape(3)
        self.df_data = np.vstack([self.df_data,_data.reshape((1,26))])

    def restore_dataframe(self):
        self.data_frame = pd.DataFrame(self.df_data,columns=self.df_colomn)
        name = "/home/marunyu/catkin_ws/src/new_dog/state_estimation/datas/"+str(time.asctime())+".csv"
        self.data_frame.to_csv(name)
        print("saved")





    def get_TFmat(self):
        r,p,y = self.rpy[0][0],self.rpy[1][0],self.rpy[2][0]
        matr_y = np.array([[np.cos(p), 0, np.sin(p)],
                           [0, 1, 0],
                           [-np.sin(p), 0, np.cos(p)]])
        matr_x = np.array([[1, 0, 0],
                           [0, np.cos(r), - np.sin(r)],
                           [0, np.sin(r), np.cos(r)]])
        matr_z = np.array([[np.cos(y), -np.sin(y), 0],
                           [np.sin(y), np.cos(y), 0],
                           [0, 0, 1]])

        TF_mat = np.dot(np.dot(matr_z, matr_y), matr_x)
        return  TF_mat
    def state_publish(self):
        if self.state_estimation_mode == 0:
            if self.got_xyz and self.got_imu:
                self.STATE = np.vstack([self.rpy,self.body_pos,self.omega, self.body_vel])
                state = Float32MultiArray()
                state.data = self.STATE.T[0].tolist()
                self.state_publisher.publish(state)
        elif self.state_estimation_mode == 1:
            if self.got_imu:
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
            self.contact_state[foot_index] = 1
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
            self.contact_state[foot_index] = 1
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
            self.contact_state[foot_index] = 1
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
            self.contact_state[foot_index] = 1
            for i in effort_message.states:
                X.append(i.total_wrench.force.x)
                Y.append(i.total_wrench.force.y)
                Z.append(i.total_wrench.force.z)
            self.touchsensors[foot_index] = np.array([np.mean(X), np.mean(Y), np.mean(Z)])
    def footpoint_callback(self, msg):
        if self.foot_point_received == 0:
            self.foot_point_received = 1
        body_lenth = self.body_lenth
        body_width = self.body_width
        for i in range(4):
            Xside_sigh = (-1) ** i
            Yside_sign = (-1) ** (1+i//2)
            self.footpoint[0][i] = msg.data[i * 3 + 0] + Xside_sigh * body_width
            self.footpoint[1][i] = msg.data[i * 3 + 1] + Yside_sign * body_lenth
            self.footpoint[2][i] = msg.data[i * 3 + 2]
    def footvel_callback(self,msg):
        if self.foot_vel_received == 0:
            self.foot_vel_received = 1
        for i in range(4):
            self.foot_vel[0][i] = msg.data[i * 3 + 0]
            self.foot_vel[1][i] = msg.data[i * 3 + 1]
            self.foot_vel[2][i] = msg.data[i * 3 + 2]
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
        if TEST:
            self.test_rpy = copy.deepcopy(self.rpy)
            self.test_omega = copy.deepcopy(self.omega)


    def state_callback(self,msg):
        if not TEST or self.state_estimation_mode == 0:
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
        if TEST and self.state_estimation_mode == 1:
            self.test_body_pos[0][0] = msg.pose[1].position.x
            self.test_body_pos[1][0] = msg.pose[1].position.y
            self.test_body_pos[2][0] = msg.pose[1].position.z - 0.05

            self.test_body_vel[0][0] = msg.twist[1].linear.x
            self.test_body_vel[1][0] = msg.twist[1].linear.y
            self.test_body_vel[2][0] = msg.twist[1].linear.z

def omega_matrix(omega_array):
    wx,wy,wz = omega_array[0][0],omega_array[1][0],omega_array[2][0]
    return np.array([[0,-wz,wy],[wz,0,-wx],[-wy,wx,0]])


_state_estimation = state_estimation()
_state_estimation.main()


