import math
import numpy as np


class Polygon_machine:
    # Based on Virtual Predictive Support Polygon
    """
    if you want to call this function, use:   [COM_x, COM_y] = Dog().Predictive_Support_Polygon()
    To get the center of gravity of the polygon
    """

    def __init__(self):
        self.Weights = 0.
        self.phases = 0.
        self.leg_idx = 0
        self.foot_point = [[[None], [None], [None]] for i in range(4)]
        self.virtual_point = [None, None]
        self.touch_ground = True
        self.c0 = 0.17
        self.c1 = 0.25

    def get_Weights_update(self):
        K = 0.5 * (math.erf(self.phases / (self.c0 * 1.414)) + math.erf((1 - self.phases) / (self.c1 * 1.414)))
        K_ = 0.5 * (2 + math.erf(-self.phases / (self.c0 * 1.414)) + math.erf((-1 + self.phases) / (self.c1 * 1.414)))
        if self.touch_ground:
            s = 1
            s_ = 0
        else:
            s = 0
            s_ = 1

        self.Weights = s * K + s_ * K_

    def get_virtual_point(self, former_leg_inf, latter_leg_inf):
        # former_leg_inf = former.Weights
        # latter_leg_inf = latter.Weights

        if self.leg_idx == 0:
            former_idx = 2
            latter_idx = 1
        if self.leg_idx == 1:
            former_idx = 0
            latter_idx = 3
        if self.leg_idx == 2:
            former_idx = 3
            latter_idx = 0
        if self.leg_idx == 3:
            former_idx = 1
            latter_idx = 2

        # get v_p_former and v_p_latter
        f_p = np.array([self.foot_point[self.leg_idx][0][0], self.foot_point[self.leg_idx][1][0]])
        f_p_former = np.array([self.foot_point[former_idx][0][0], self.foot_point[former_idx][1][0]])
        f_p_latter = np.array([self.foot_point[latter_idx][0][0], self.foot_point[latter_idx][1][0]])

        f_p_mat = np.vstack([np.hstack([f_p, f_p_latter]), np.hstack([f_p, f_p_former])])
        Weight_mat = np.array([[self.Weights, 0.], [0., self.Weights], [1 - self.Weights, 0.], [0., 1 - self.Weights]])
        v_p_mat = np.dot(f_p_mat, Weight_mat)
        v_p_former = v_p_mat[0]
        v_p_latter = v_p_mat[1]

        # get v_p
        v_p = (self.Weights * f_p + former_leg_inf * v_p_former + latter_leg_inf * v_p_latter) / \
              (self.Weights + former_leg_inf + latter_leg_inf)
        self.virtual_point = v_p
