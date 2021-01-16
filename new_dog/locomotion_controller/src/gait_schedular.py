#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray
from locomotion_config import *


class gait_schedular():
    def __init__(self):
        self.Gait_phase = []  # phase of each foot
        self.Gait_time = []  # time of each step
        self.Gait_currentTime = 0  # current time of gait
        self.Gait_status = []  # current state of gait
        self.Gait_index = 0  # index of step
        self.Gait_control = 0
        self.Gait_force = []
        self.Gait_pacePropotion = 0.5

    def gait_init(self, gait):
        if gait == "walk":
            # Diagonal slow walk gait
            self.name = gait
            self.Gait_control = 0
            self.Gait_index = 0
            self.Gait_phase = [[0, 1, 1, 0], [1, 1, 1, 1], [1, 0, 0, 1], [1, 1, 1, 1]]
            self.Gait_time = [0.08, 0.08, 0.08, 0.08]
            # self.Gait_status = ["pace", "land", "pace", "land"]
            self.Gait_pacePropotion = 0.5
            self.Gaitneed_init = 0

        if gait == "stand":
            # stand
            self.name = gait
            self.Gait_control = 0
            self.Gait_index = 0
            self.Gait_phase = [[1, 1, 1, 1]]
            self.Gait_time = [0.2]
            # self.Gait_status = ["land"]
            self.Gait_pacePropotion = 1
            self.Gaitneed_init = 0

        if gait == "test":
            # test stand
            self.name = gait
            self.Gait_control = 0
            self.Gait_index = 0
            self.Gait_phase = [[1, 1, 1, 1]]
            self.Gait_time = [1.5]
            # self.Gait_status = ["land"]
            self.Gait_pacePropotion = 1
            self.Gaitneed_init = 0

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

    def get_groundLeg(self):
        # groundLeg
        groundLeg = []
        for i in range(4):
            if self.Gait_phase[self.Gait_index][i] == 1:
                groundLeg.append(i)
        return groundLeg

    def get_nextgroundLeg(self):
        groundLeg = []
        next_index = self.get_nextindex()
        for i in range(4):
            if self.Gait_phase[next_index][i] == 1:
                groundLeg.append(i)
        return groundLeg

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
