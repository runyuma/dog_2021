# -*-coding:utf-8-*-
import numpy as np
height = 0.15  #步抬起高度

class statemachine():
    def __init__(self):
        self.phase = 0

    def generate_point(self, Tf, ini_pos, fin_pos):
        # 要摆腿轨迹规划
        #Tf 摇摆时间 ini_pos:初始点 fin_pos:落足点
        self.target_pos = np.array([[ini_pos[0][0] + (fin_pos[0][0] - ini_pos[0][0])*(3 * self.phase**2 - 2*self.phase**3)],
                                    [ini_pos[1][0] + (fin_pos[1][0] - ini_pos[1][0]) * (3 * self.phase**2 - 2*self.phase**3)],
                                    [0]])
        self.target_vel = np.array([[(fin_pos[0][0] - ini_pos[0][0])*(6 * self.phase - 6*self.phase**2)/Tf],
                                    [(fin_pos[1][0] - ini_pos[1][0]) * (6 * self.phase - 6*self.phase**2)/Tf],
                                    [0]])
        self.target_acc = np.array(
            [[(fin_pos[0][0] - ini_pos[0][0]) * (6 - 12 * self.phase) / Tf**2],
             [(fin_pos[1][0] - ini_pos[1][0]) * (6 - 12 * self.phase) / Tf**2], [0]])

        if self.phase <= 0.5:
            self.target_pos[2][0] = ini_pos[2][0] + height * (3 * (2*self.phase)**2 - 2*(2*self.phase)**3)
            self.target_vel[2][0] = height * (6 * (2*self.phase) - 6*(2*self.phase)**2)/(Tf/2)
            self.target_acc[2][0] =  height *(6 - 12 * (2*self.phase)) / (Tf/2)**2
        else:
            self.target_pos[2][0] = ini_pos[2][0] + height - height * (3 * (2 * self.phase - 1) ** 2 - 2 * (2 * self.phase - 1) ** 3)
            self.target_vel[2][0] = - height * (6 * (2 * self.phase - 1) - 6 * (2 * self.phase - 1) ** 2) / (Tf/2)
            self.target_acc[2][0] = - height * (6 - 12 * (2 * self.phase - 1)) / (Tf/2) ** 2

    def generate_point_ground(self, Tf, ini_pos, fin_pos):
        # Ground Leg Pos Get
        self.target_pos = np.array([[0],
                                    [ini_pos[1] + (fin_pos[1] - ini_pos[1]) * self.phase],
                                    [ini_pos[2] + (fin_pos[2] - ini_pos[2]) * self.phase]])
        # Ground腿轨迹规划
        # self.target_pos = np.array([[ini_pos[0][0] + (fin_pos[0][0] - ini_pos[0][0])*(3 * self.phase**2 - 2*self.phase**3)],
        #                             [ini_pos[1][0] + (fin_pos[1][0] - ini_pos[1][0]) * (3 * self.phase**2 - 2*self.phase**3)],
        #                             [ini_pos[2][0]]])

        # self.target_vel = np.array([[(fin_pos[0][0] - ini_pos[0][0])*(6 * self.phase - 6*self.phase**2)/Tf],
        #                             [(fin_pos[1][0] - ini_pos[1][0]) * (6 * self.phase - 6*self.phase**2)/Tf],
        #                             [0]])

        # self.target_pos = np.array([[ini_pos[0][0] + (fin_pos[0][0] - ini_pos[0][0]) * (3 * self.phase ** 2 - 2 * self.phase ** 3)],
        #                             [ini_pos[1][0] + (fin_pos[1][0] - ini_pos[1][0]) * (3 * self.phase ** 2 - 2 * self.phase ** 3)],
        #                             [ini_pos[2][0]]])
        #
        # self.target_vel = np.array([[(fin_pos[0][0] - ini_pos[0][0]) * (12 * self.phase - 12*self.phase**2)/Tf],
        #                             [(fin_pos[1][0] - ini_pos[1][0]) * (12 * self.phase - 12*self.phase**2)/Tf],
        #                             [0]])
        #
        # self.target_acc = np.array(
        #     [[(fin_pos[0][0] - ini_pos[0][0]) * (6 - 12 * self.phase) / Tf**2],
        #      [(fin_pos[1][0] - ini_pos[1][0]) * (6 - 12 * self.phase) / Tf**2], [0]])