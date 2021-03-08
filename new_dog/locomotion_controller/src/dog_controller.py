#!/usr/bin/env python3
# coding:utf-8
import numpy as np
import copy
import time
from gait_schedular import *
from QP_solver import *
default_height = 0.32
USE_HIPLOOP = 1
USE_RAIBERT_HEURISTIC = 0

gait_map = {0:"STANDING",1:"TROTING_RUNING"}

class Dog():
    def __init__(self):
        self.body_mass = 10 + (0.3+0.6+0.5)* 4
        self.body_width = 0
        self.body_lenth = 0

        self.rpy = np.zeros((3, 1))
        self.omega = np.zeros((3, 1))
        self.body_pos = np.zeros((3, 1))
        self.body_vel = np.zeros((3, 1))

        self.footpoint = [np.zeros((3, 1)) for i in range(4)]
        self.footvel = [np.zeros((3, 1)) for i in range(4)]

        self.last_targetstate = [np.zeros((3,1)) for i in range(4)]

        self._statemachine = state_machine()
        self.schedualgroundLeg = [1,1,1,1]
        self.stateindex = 0

        self.target_state = [np.zeros((3,1)) for i in range(4)]

        self.target_swingpos = [np.zeros((3,1)) for i in range(4)]
        self.target_swingvel = [np.zeros((3,1)) for i in range(4)]
        self.init_SWINGfootpoint = [np.zeros((3,1)) for i in range(4)]

        self.command_vel = np.zeros((3,1))
        self.command_omega = np.zeros((3,1))

        # self.use_touchtenser = rospy.get_param("USE_TOUCHSENSOR")
        self.ros_time = 0
        self.last_rosime = 0
        self.set_shedule = 1
        
        self.last_gait = 0
        self.gait_num = 0

    def get_TFmat(self):
        matr_y = np.array([[np.cos(self.rpy[1][0]), 0, np.sin(self.rpy[1][0])],
                         [0, 1, 0],
                         [-np.sin(self.rpy[1][0]), 0, np.cos(self.rpy[1][0])]])
        matr_x = np.array([[1, 0, 0],
                         [0, np.cos(self.rpy[0][0]), - np.sin(self.rpy[0][0])],
                         [0, np.sin(self.rpy[0][0]), np.cos(self.rpy[0][0])]])
        matr_z = np.array([[np.cos(self.rpy[2][0]), -np.sin(self.rpy[2][0]), 0],
                              [np.sin(self.rpy[2][0]), np.cos(self.rpy[2][0]), 0],
                              [0, 0, 1]])

        self.TF_mat = np.dot(np.dot(matr_z,matr_y),matr_x)

    def getTargetstate(self,t,n,target_pos = None, target_dir = None,last_targetstate = None):
        # get desired state in a step
        # t the time between one state and another
        # n the number of state
        # target_vel target_angularvel np.ndarray (3,1)
        # gait_schedular struct
        # return list of n  state[rpy,pos,omega,vel]
        if self._statemachine._gait.name == "STANDING":
            if default_height - self.body_pos[2][0] >= 0.05:
                states = [0 for i in range(n)]
                for i in range(n):
                    self.target_rpy = np.zeros((3, 1))
                    self.target_rpy[2][0] = self.rpy[2][0]
                    self.target_vel = np.zeros((3, 1))
                    self.target_pos = copy.deepcopy(self.body_pos)
                    if i <= n / 2:
                        self.target_vel[2][0] = 4 * (default_height - self.body_pos[2][0]) * i / (n * n * t)
                        self.target_pos[2][0] = 2 * (default_height - self.body_pos[2][0]) * i * i / (n * n) + \
                                                self.body_pos[2][0]

                    else:
                        self.target_vel[2][0] = 4 * (default_height - self.body_pos[2][0]) * (n - i) / (n * n * t)
                        self.target_pos[2][0] = default_height - 2 * (default_height - self.body_pos[2][0]) * (
                                    n - i) * (n - i) / (n * n)

                    if target_pos != None:
                        self.target_pos[0][0] += target_pos[0][0]
                        self.target_pos[1][0] += target_pos[1][0]
                    if target_dir != None:
                        self.target_rpy[0][0] += target_dir[0]
                        self.target_rpy[1][0] += target_dir[1]
                        self.target_rpy[2][0] += target_dir[2]

                    self.target_omega = np.zeros((3, 1))
                    states[i] = copy.deepcopy([self.target_rpy, self.target_pos, self.target_omega, self.target_vel])
                return states

            else:
                if last_targetstate == None:
                    self.target_rpy = np.zeros((3, 1))
                    self.target_rpy[2][0] = self.rpy[2][0]
                    self.target_pos = copy.deepcopy(self.body_pos)
                    self.target_pos[2][0] = default_height
                    self.target_vel = np.zeros((3, 1))
                    self.target_omega = np.zeros((3, 1))


                else:
                    self.target_rpy = np.zeros((3, 1))
                    self.target_rpy[2][0] = last_targetstate[0][2][0]
                    self.target_pos = last_targetstate[1]
                    self.target_pos[2][0] = default_height
                    self.target_omega = np.zeros((3, 1))
                    self.target_vel = np.zeros((3, 1))

                if target_dir != None:
                    self.target_rpy[0][0] += target_dir[0]
                    self.target_rpy[1][0] += target_dir[1]
                    self.target_rpy[2][0] += target_dir[2]

                state = [self.target_rpy, self.target_pos, self.target_omega, self.target_vel]
                states = [state for i in range(n)]
                return states
        elif self._statemachine._gait.name == "TROTING_WALKING" or self._statemachine._gait.name == "TROTING_RUNING" or self._statemachine._gait.name == "SLOW_WALKING":
            states = [0 for i in range(n)]
            current_vel = np.dot(np.linalg.inv(self.TF_mat),self.body_vel)
            vel_diff = copy.copy(self.command_vel[1][0]) - current_vel[1][0]
            rpy = last_targetstate[0]
            matr_y = np.array([[np.cos(rpy[1][0]), 0, np.sin(rpy[1][0])],
                               [0, 1, 0],
                               [-np.sin(rpy[1][0]), 0, np.cos(rpy[1][0])]])
            matr_x = np.array([[1, 0, 0],
                               [0, np.cos(rpy[0][0]), - np.sin(rpy[0][0])],
                               [0, np.sin(rpy[0][0]), np.cos(rpy[0][0])]])
            matr_z = np.array([[np.cos(rpy[2][0]), -np.sin(rpy[2][0]), 0],
                               [np.sin(rpy[2][0]), np.cos(rpy[2][0]), 0],
                               [0, 0, 1]])
            target_TFmat = np.dot(np.dot(matr_z,matr_y),matr_x)
            if USE_RAIBERT_HEURISTIC:
                last_targetstate[1][0][0] = self.body_pos[0][0]
            if abs(vel_diff)>0.3:
                ay = np.sign(vel_diff) * min(abs(vel_diff)/ (n * t), 0.8)
                for i in range(n):
                    vy = np.dot(np.linalg.inv(target_TFmat),last_targetstate[3])[1][0] + ay * i * t
                    dy = np.dot(np.linalg.inv(target_TFmat),last_targetstate[3])[1][0]*i*t + ay * i * i * t * t/2
                    self.target_vel = np.dot(target_TFmat,np.array([[0],[vy],[0]]))
                    self.target_pos = last_targetstate[1] + np.dot(target_TFmat,np.array([[0],[dy],[0]]))
                    self.target_omega = self.command_omega
                    self.target_rpy = last_targetstate[0] + np.array([[0],[0],[self.command_omega[2][0] * t * i]])
                    if abs(self.target_rpy[2][0]) >  np.pi:
                        self.target_rpy[2][0] += - np.sign(self.target_rpy[2][0]) * 2 * np.pi
                    states[i] = copy.deepcopy([self.target_rpy, self.target_pos, self.target_omega, self.target_vel])
                return states
            else:
                for i in range(n):
                    dy = copy.deepcopy(self.command_vel[1][0])*t*i
                    self.target_vel = np.dot(self.TF_mat,copy.deepcopy(self.command_vel))
                    self.target_pos = last_targetstate[1] + np.dot(target_TFmat,np.array([[0],[dy],[0]]))
                    self.target_omega = self.command_omega
                    self.target_rpy = last_targetstate[0] + np.array([[0],[0],[self.command_omega[2][0] * t * i]])
                    if abs(self.target_rpy[2][0]) > np.pi:
                        self.target_rpy[2][0] += - np.sign(self.target_rpy[2][0]) * 2 * np.pi
                    states[i] = copy.deepcopy([self.target_rpy, self.target_pos, self.target_omega, self.target_vel])
                return states



    def getTarget_Force(self):
        #tarfetstate:[rpy,xyz,w,vel]
        target_rpy,target_pos,target_omega,target_vel = self.target_state

        if self._statemachine._gait.name == "STANDING":
            Force_limit = np.array([[50], [50], [200]])
            Force_KP = np.diag([2500, 800, 800])
            Force_KD = np.diag([600, 350, 150])
            Torque_limit = np.array([[20], [30], [30]])
            Torque_KP = np.diag([200, 300, 300])
            Torque_KD = np.diag([30, 12, 50])


        elif self._statemachine._gait.name == "TROTING_WALKING" or self._statemachine._gait.name == "TROTING_RUNING":
            Force_limit = np.array([[50],[50],[200]])
            Force_KP = np.diag([400,450,600])
            Force_KD = np.diag([250,100,120])
            Torque_limit = np.array([[20], [30], [30]])
            Torque_KP = np.diag([400, 600, 500])
            Torque_KD = np.diag([50, 50, 50])
        elif self._statemachine._gait.name == "SLOW_WALKING":
            if not USE_RAIBERT_HEURISTIC:
                Force_limit = np.array([[50], [50], [200]])
                Force_KP = np.diag([500, 800, 800])
                Force_KD = np.diag([300, 350, 150])
                Torque_limit = np.array([[20], [30], [30]])
                Torque_KP = np.diag([400, 600, 600])
                Torque_KD = np.diag([50, 65, 60])

            elif USE_RAIBERT_HEURISTIC:
                Force_limit = np.array([[50], [50], [200]])
                Force_KP = np.diag([10, 800, 800])  # 600
                Force_KD = np.diag([500, 350, 150])
                Torque_limit = np.array([[20], [30], [30]])
                Torque_KP = np.diag([400, 500, 600])
                Torque_KD = np.diag([50, 50, 100])

        velerror = target_vel - self.body_vel
        target_force = np.dot(Force_KP, target_pos - self.body_pos) + np.dot(Force_KD, velerror)  + np.array(
            [[0], [0], [self.body_mass * 9.8]])
        target_force = np.dot(np.linalg.inv(self.TF_mat), target_force)
        omegaError = target_omega - self.omega
        if abs(self.rpy[2][0])>= np.pi/2 and abs(target_rpy[2][0])>= np.pi and self.rpy[2][0]*target_rpy[2][0] <0:
            target_rpy[2][0] += np.sign(target_rpy[2][0])  * 2 * np.pi
        target_torque = np.dot(Torque_KP, target_rpy - self.rpy) + np.dot(Torque_KD, omegaError)
        for i in range(3):
            if abs(target_force[i][0]) >= Force_limit[i][0]:
                target_force[i][0] = np.sign(target_force[i][0]) * Force_limit[i][0]
            if abs(target_torque[i][0]) >= Torque_limit[i][0]:
                target_torque[i][0] = np.sign(target_torque[i][0]) * Torque_limit[i][0]
        if target_force[2][0] < 100:
            target_force[2][0] = 100
        return (target_force,target_torque,)

    def statemachine_update(self):
        # start_time = time.time()
        _gait_index = self._statemachine._gait.Gait_index
        _gait_time = self._statemachine._gait.Gait_time[_gait_index]
        # if gait time == 0 set trajectary set scheduleleg
        if self._statemachine._gait.Gait_currentTime == 0:
            self.start_phasetime = self.ros_time
            self.schedualgroundLeg = self._statemachine._gait.get_schedualgroundLeg()
            self._statemachine.phase = copy.copy(self._statemachine._gait.Gait_phase[_gait_index])

            self.targetstates = self.getTargetstate( 0.01, int(_gait_time/0.01),last_targetstate = self.last_targetstate)
            last_index =  self._statemachine._gait.get_lastindex()
            for i in range(4):
                if (self._statemachine._gait.Gait_phase[_gait_index][i] != 1) and (self._statemachine._gait.Gait_phase[last_index][i] == 1):
                    self.init_SWINGfootpoint[i] = copy.deepcopy(self.footpoint[i])



        #update_time
        self._statemachine._gait.gaittime_update(self.ros_time - self.last_rostime)

        #state_index
        self.stateindex = int(self._statemachine._gait.Gait_currentTime/0.01)
        if self.stateindex>= int(_gait_time/0.01):
            self.stateindex = int(_gait_time/0.01)-1
        #update_statemachine
        next_index = self._statemachine._gait.get_nextindex()
        for i in range(4):
            if self.schedualgroundLeg[i] != 1:
                phase_diff = self._statemachine._gait.Gait_phase[next_index][i] - self._statemachine._gait.Gait_phase[_gait_index][i]
                self._statemachine.phase[i] += (self.ros_time - self.last_rostime) * phase_diff/_gait_time

        #check if changethe gait index,reset phase currentTime, set last state,set_initleg
        if self._statemachine._gait.Gait_currentTime >= _gait_time:
            self._statemachine._gait.Gait_index = self._statemachine._gait.get_nextindex()
            self._statemachine.phase = copy.copy(self._statemachine._gait.Gait_phase[self._statemachine._gait.Gait_index])
            self._statemachine._gait.Gait_currentTime = 0
            self.last_targetstate = self.targetstates[self.stateindex]
            if self.gait_num != self.last_gait:
                self._statemachine._gait.gait_init(gait_map[self.gait_num]) # TODO;proper change logic
            self.last_gait = self.gait_num
            self.schedualgroundLeg = self._statemachine._gait.get_schedualgroundLeg()
            self.set_shedule = 1

        self.last_rostime = copy.deepcopy(self.ros_time)

        # print(time.time() - start_time)


    def Force_calculation(self):
        self.target_state = self.targetstates[self.stateindex]
        self.target_force,self.target_torque = self.getTarget_Force()
        _forceTorque = np.vstack([self.target_force,self.target_torque])
        self.force_list,self.forceerror = OSQP_solve(self.footpoint, self._statemachine._gait.Gait_phase[self._statemachine._gait.Gait_index], _forceTorque)
        for i in range(4):
            if self._statemachine.phase[i]>= 0.95 and self._statemachine._gait.Gait_phase[self._statemachine._gait.Gait_index][i]!= 1:
                self.schedualgroundLeg[i] = 1
                self.force_list[i] = np.array([[0],[0],[10]])


    def swingleg_calculation(self):
        self.target_state = self.targetstates[self.stateindex]
        _target_vel = self.target_state[3]
        _target_vel = np.dot(np.linalg.inv(self.TF_mat),_target_vel)
        for i in range(4):
            if self.schedualgroundLeg[i] == 0:
                Xside_sigh = (-1) ** i
                Yside_sign = (-1) ** (1 + i // 2)
                phase1 = self._statemachine._gait.Gait_phase[self._statemachine._gait.Gait_index][i]
                phase2 = self._statemachine._gait.Gait_phase[self._statemachine._gait.get_nextindex()][i]
                time = self._statemachine._gait.Gait_time[self._statemachine._gait.Gait_index]
                swing_time = time/(phase2-phase1)
                side_sign = (-1)**i
                final_point = np.array([[Xside_sigh * self.body_width],[Yside_sign * self.body_lenth],[0]])
                final_point += np.array([[side_sign * 0.06],[0],[-default_height]])#TODO//0.06 tobechanged
                final_point += np.array([[0],[self._statemachine._gait.Gait_pacePropotion * swing_time * _target_vel[1][0]],[0]])
                if USE_RAIBERT_HEURISTIC:
                    final_point += self._statemachine._gait.Gait_pacePropotion * swing_time *(1 - self._statemachine.phase[i]) * np.dot(np.linalg.inv(self.TF_mat),(self.body_vel - self.target_vel))
                _targte_swingpos,_targte_swingvel,_targte_swingacc= gait_swingLeg(self._statemachine.phase[i],swing_time, self.init_SWINGfootpoint[i], final_point)
                _targte_swingpos[0][0] = final_point[0][0]# TODO:// magic change
                self.target_swingpos[i] = _targte_swingpos
                self.target_swingvel[i] = _targte_swingvel





            



