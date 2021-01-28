#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray



class gait_schedular():
    def __init__(self):
        self.Gait_phase = []  # phase of each foot
        self.Gait_time = []  # time of each step
        self.Gait_currentTime = 0  # current time of gait
        self.Gait_status = []  # current state of gait
        self.Gait_index = 0  # index of step
        self.Gait_pacePropotion = 0.5

    def gait_init(self, gait):
        if gait == "TROTING_WALKING":
            # 对角慢走步态
            self.name = gait
            self.Gait_control = 0
            self.Gait_index = 0
            self.Gait_phase = [[1, 0, 0, 1], [1, 1, 1, 1], [0, 1, 1, 0], [1, 1, 1, 1]]
            self.Gait_time = [0.25, 0.1, 0.25, 0.1]
            self.Gait_status = ["pace", "land", "pace", "land"]
            self.Gait_pacePropotion = 0.5
            self.Gaitneed_init = 0
        elif gait == "TROTING_RUNING":  # maxvelocity = 0.8m/s maxvel with angularvel is 0.6 0.3
            # 对角快走步态（无腾空期）
            self.name = gait
            self.Gait_control = 0
            self.Gait_index = 0
            self.Gait_phase = [[1, 0, 0, 1], [0, 1, 1, 0]]
            self.Gait_time = [0.25, 0.25]
            self.Gait_status = ["pace", "pace"]
            self.Gait_pacePropotion = 0.5
            self.Gaitneed_init = 0
        elif gait == "SLOW_WALKING":
            # 慢走步态
            self.name = gait
            self.Gait_control = 0
            self.Gait_index = 0
            self.Gait_phase = [[1, 1, 0, 1], [1, 1, 1, 1], [1, 0, 1, 1], [1, 1, 1, 1], [1, 1, 1, 0], [1, 1, 1, 1],
                               [0, 1, 1, 1], [1, 1, 1, 1]]
            self.Gait_time = [0.2, 0.03, 0.2, 0.03, 0.2, 0.03, 0.2, 0.03]

            self.Gait_status = ["pace", "land", "pace", "land", "pace", "land", "pace", "land"]
            self.Gait_pacePropotion = 2
            self.Gaitneed_init = 0
        elif gait == "STANDING":
            # 站立
            self.name = gait
            self.Gait_control = 0
            self.Gait_index = 0
            self.Gait_phase = [[1, 1, 1, 1]]
            self.Gait_time = [2]
            self.Gait_status = ["pace", "pace", "pace", "pace"]
            self.Gaitneed_init = 0
        elif gait == "TROTING_jumping":  # max_vel 1.2
            # 对角快走步态（1/7腾空期）
            self.name = gait
            self.Gait_control = 0
            self.Gait_index = 0
            self.Gait_phase = [[1, 0., 0., 1], [1, 0.71, 0.71, 1], [0, 0.86, 0.86, 0], [0.14, 1, 1, 0.14],
                               [0.71, 1, 1, 0.71], [0.86, 0, 0, 0.86]]
            self.Gait_phase_inited = [[1, 0.14, 0.14, 1], [1, 0.71, 0.71, 1], [0, 0.86, 0.86, 0], [0.14, 1, 1, 0.14],
                                      [0.71, 1, 1, 0.71], [0.86, 0, 0, 0.86]]
            self.Gait_time = [0.1, 0.024, 0.024, 0.1, 0.024, 0.024]
            self.Gait_status = ["pace", "bounce", "jump", "pace", "bounce", 'jump']
            self.Gait_pacePropotion = 0.5
            self.Gaitneed_init = 1
        elif gait == "FAST_running":
            # 快步步态
            self.name = gait
            self.Gait_index = 0
            self.Gait_phase = [[1, 1., 1., 1], [1, 1, 1, 0], [1, 0, 0, 0.42], [0, 0.33, 0.33, 0.71],
                               [0.28, 0.66, 0.66, 1], [0.57, 1, 1, 1], [1, 1., 1., 1], [1, 1, 0, 1], [0, 1, 0.42, 0],
                               [0.33, 0., 0.71, 0.33], [0.66, 0.28, 1, 0.66], [1, 0.57, 1, 1]]
            self.Gait_time = [0.16, 0.024, 0.016, 0.016, 0.016, 0.024, 0.16, 0.024, 0.016, 0.016, 0.016, 0.024]
            self.Gait_status = ["land", "bounce", "left_up", "jump", "right_touch", "pace", "land", "bounce",
                                "right_up", "jump", "left_touch", "pace", ]
            self.Gait_pacePropotion = 3.5

    def get_nextindex(self):
        # get next step's index
        lenth = len(self.Gait_phase)
        if self.Gait_index == lenth - 1:
            return 0
        else:
            return self.Gait_index + 1

    def get_lastindex(self):
        # get last step's index
        lenth = len(self.Gait_phase)
        if self.Gait_index == 0:
            return lenth - 1
        else:
            return self.Gait_index - 1

    def gaittime_update(self, T):
        # update gait time
        self.Gait_currentTime += T

    def get_indexChange(self):
        # change to next step
        if self.Gait_currentTime >= self.Gait_time[self.Gait_index]:
            self.Gait_index = self.get_nextindex()
        self.Gait_currentTime = 0

    def get_schedualgroundLeg(self):
        # groundLeg
        groundLeg = [0,0,0,0]
        for i in range(4):
            if self.Gait_phase[self.Gait_index][i] == 1:
                groundLeg[i] = 1
        return groundLeg

    def get_nextschedualgroundLeg(self):
        groundLeg = [0,0,0,0]
        next_index = self.get_nextindex()
        for i in range(4):
            if self.Gait_phase[next_index][i] == 1:
                groundLeg[i] = 1
        return groundLeg
SWING = 0
GROUND = 1
class state_machine():
    def __init__(self):
        self.phase = [1,1,1,1]
        self.timestep = 0.001
        self._gait = gait_schedular()
        self._gait.gait_init("STANDING")



def gait_swingLeg(phase, Tf, ini_pos, fin_pos):
    # 要摆腿轨迹规划
    #Tf 摇摆时间 ini_pos:初始点 fin_pos:落足点
    height = 0.10
    target_pos = np.array([[ini_pos[0][0] + (fin_pos[0][0] - ini_pos[0][0])*(3 * phase**2 - 2*phase**3)], [ini_pos[1][0] + (fin_pos[1][0] - ini_pos[1][0]) * (3 * phase**2 - 2*phase**3)], [0]])
    target_vel = np.array([[(fin_pos[0][0] - ini_pos[0][0])*(6 * phase - 6*phase**2)/Tf], [ (fin_pos[1][0] - ini_pos[1][0]) * (6 * phase - 6*phase**2)/Tf], [0]])
    target_acc = np.array(
        [[(fin_pos[0][0] - ini_pos[0][0]) * (6 - 12 * phase) / Tf**2],
         [(fin_pos[1][0] - ini_pos[1][0]) * (6 - 12 * phase) / Tf**2], [0]])
    if phase <= 0.5:
        target_pos[2][0] = ini_pos[2][0] + height * (3 * (2*phase)**2 - 2*(2*phase)**3)
        target_vel[2][0] = height * (6 * (2*phase) - 6*(2*phase)**2)/(Tf/2)
        target_acc[2][0] = height * (6 - 12 * (2*phase)) / (Tf/2)**2
    else:
        target_pos[2][0] = ini_pos[2][0] + height - height * (3 * (2 * phase - 1) ** 2 - 2 * (2 * phase - 1) ** 3)
        target_vel[2][0] = - height * (6 * (2 * phase - 1) - 6 * (2 * phase - 1) ** 2) / (Tf/2)
        target_acc[2][0] = - height * (6 - 12 * (2 * phase - 1)) / (Tf/2) ** 2

    return(target_pos,target_vel,target_acc)
    # def Gait_statemachine_init(Dog):
    #     # 状态机步态初始化
    #     Dog.schedule.Gait_currentTime = 0
    #     for i in range(4):
    #         Dog.legs[i].statemachine.phase = Dog.schedule.Gait_phase[Dog.schedule.Gait_index][i]
    #         if Dog.schedule.Gait_phase[Dog.schedule.Gait_index][i] < 1:
    #             Dog.legs[i].statemachine.state = 0  # SWING
    #
    # def Gait_statemachine_update(Dog, T):
    #     # 状态机步态更新
    #     for i in range(4):
    #         if Dog.schedule.Gait_phase[Dog.schedule.Gait_index][i] < 1:
    #             Dog.legs[i].statemachine.phase += T * (Dog.schedule.Gait_phase[Dog.schedule.get_nextindex()][i] -
    #                                                    Dog.schedule.Gait_phase[Dog.schedule.Gait_index][i]) / \
    #                                               Dog.schedule.Gait_time[Dog.schedule.Gait_index]
    #
    # def Gait_statemachine_reset(Dog):
    #     ##状态机步态重设
    #     for i in range(4):
    #         Dog.legs[i].statemachine.phase = Dog.schedule.Gait_phase[Dog.schedule.Gait_index][i]
    #         if Dog.schedule.Gait_phase[Dog.schedule.Gait_index][i] < 1:
    #             Dog.legs[i].statemachine.state = 0  # SWING
    #     if Dog.schedule.Gaitneed_init == 1:
    #         Dog.schedule.Gait_phase = Dog.schedule.Gait_phase_inited
