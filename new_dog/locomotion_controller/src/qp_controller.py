#!/usr/bin/env python3
# coding:utf-8
import rospy
import numpy as np
import osqp
from cvxopt import matrix, solvers
from scipy import sparse
from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray


def arrayToQpmat(array):
    return np.matrix((array.T).tolist())


def crossproduct_matrix(footpoint_array):
    # cross product
    # footpoint_array: lenth: 3 [footpoint_x, footpoint_y, footpoint_z]
    return (np.array([[0, - footpoint_array[2], footpoint_array[1]],
                      [footpoint_array[2], 0, - footpoint_array[0]],
                      [- footpoint_array[1], footpoint_array[0], 0]]))


def QP_solve(_leg_poses, groundLeg, forceTorque, Gait_name):
    # fx = 0.5 * (Af - b).T dot D dot (Af - b)
    # f: length: 12 [fx1, fy1, fz1, ... fz3].T when leg_num is 4
    print("leg_poses: ", np.array(_leg_poses).reshape(4, 3))
    leg_num = len(groundLeg)
    print("QPcontroller_groundleg: ", groundLeg)
    if leg_num == 4:
        b = forceTorque
        aUpperArray = np.hstack([np.eye(3) for i in range(leg_num)])
        adownArray = np.hstack([crossproduct_matrix(_leg_poses[i]) for i in groundLeg])
        aArray = np.vstack([aUpperArray, adownArray])
        D1 = 10 * np.eye(6)
        D1[0][0] = 100
        D1[1][1] = 40
        D1[2][2] = 20
        D1[3][3] = 80
        D1[4][4] = 200
        D1[5][5] = 50
        D2 = np.eye(3 * leg_num)
        D2[2][2] = 2
        D2[5][5] = 2
        p = np.dot(np.dot(aArray.T, D1), aArray)
        P_mat = sparse.csc_matrix(p)
        q = (- np.dot(np.dot(b.T, D1), aArray)).T
        q_mat = arrayToQpmat(q)
        # Af: 12 * 1
        # f: 3 * leg_num
        G = sparse.eye(3 * leg_num, format='csc')
        l = [0 for i in range(3 * leg_num)]
        for i in range(leg_num):
            l[i * 3] = -1.5 * 20.0 / leg_num
            l[i * 3 + 1] = -1.5 * 20.0 / leg_num
            l[i * 3 + 2] = 1. / leg_num
        u = [0 for i in range(3 * leg_num)]
        for i in range(leg_num):
            u[i * 3] = 1.5 * 20.0 / leg_num
            u[i * 3 + 1] = 1.5 * 20.0 / leg_num
            u[i * 3 + 2] = 400.0 / leg_num
        # Create an OSQP object
        prob = osqp.OSQP()
        # Setup workspace and change alpha parameter
        prob.setup(P_mat, q, G, l, u, verbose=False)
        # Solve problem
        res = prob.solve()
        error = b - np.dot(aArray, res.x)
        print("qp_error: ", error)
        return res.x
    # elif leg_num == 3:
    #     use_planB = 1
    #     if use_planB:
    #         if 0 in groundLeg and 3 in groundLeg:
    #             if 1 in groundLeg:
    #                 _groundLeg = [0, 3]
    #                 thirdLeg = 1
    #             else:
    #                 _groundLeg = [0, 3]
    #                 thirdLeg = 2
    #         elif 1 in groundLeg and 2 in groundLeg:
    #             if 0 in groundLeg:
    #                 _groundLeg = [1, 2]
    #                 thirdLeg = 0
    #             else:
    #                 _groundLeg = [1, 2]
    #                 thirdLeg = 3
    #         leg_num = 2
    #         b = forceTorque
    #         aUpperArray = np.hstack([np.eye(3) for i in range(leg_num)])
    #         adownArray = np.hstack([crossproduct_matrix(_leg_poses[i]) for i in _groundLeg])
    #         aArray = np.vstack([aUpperArray, adownArray])
    #         D1 = 10 * np.eye(6)
    #         D1[0][0] = 40
    #         D1[1][1] = 15
    #         D1[2][2] = 1
    #         D1[3][3] = 80
    #         D1[4][4] = 120
    #         D1[5][5] = 10
    #         D2 = np.eye(6)
    #         D2[2][2] = 3
    #         D2[5][5] = 3
    #         p = np.dot(np.dot(aArray.T, D1), aArray)
    #         P_mat = arrayToQpmat(p)
    #         q = (- np.dot(np.dot(b.T, D1), aArray)).T
    #         q_mat = arrayToQpmat(q)
    #         G = [[0.0 for i in range(6 * leg_num)] for j in range(3 * leg_num)]
    #         for i in range(leg_num):
    #             for j in range(3):
    #                 G[3 * i + j][6 * i + 2 * j] = 1.0
    #                 G[3 * i + j][6 * i + 2 * j + 1] = -1.0
    #         G_mat = matrix(G)
    #         h = [0 for i in range(6 * leg_num)]
    #         for i in range(leg_num):
    #             h[i * 6] = 1.5 * 20.0 / leg_num
    #             h[i * 6 + 1] = 1.5 * 20.0 / leg_num
    #             h[i * 6 + 2] = 1.5 * 20.0 / leg_num
    #             h[i * 6 + 3] = 1.5 * 20.0 / leg_num
    #             h[i * 6 + 4] = 1.5 * forceTorque[2][0] / leg_num
    #             h[i * 6 + 5] = -0.5 * forceTorque[2][0] / leg_num
    #         H_mat = matrix(h)
    #         solvers.options['show_progress'] = False
    #         result = solvers.qp(P_mat, q_mat, G_mat, H_mat)
    #         result = np.array(result['x'])
    #         force_list = decompressForce(_groundLeg, result)
    #         error = b - np.dot(aArray, result)
    #         force_list[thirdLeg] = np.array([[error[0][0]], [0], [error[4][0] / _leg_poses[thirdLeg][0][0]]])
    #         if force_list[thirdLeg][2][0] > 5:
    #             force_list[thirdLeg][2][0] = 5
    #         elif force_list[thirdLeg][2][0] < 1:
    #             force_list[thirdLeg][2][0] = 1
    #         error = b - np.dot(aArray, result)
    #         return force_list, error
    #     else:
    #         b = forceTorque
    #         aUpperArray = np.hstack([np.eye(3) for i in range(leg_num)])
    #         adownArray = np.hstack([crossproduct_matrix(_leg_poses[i]) for i in groundLeg])
    #         aArray = np.vstack([aUpperArray, adownArray])
    #         D1 = 10 * np.eye(6)
    #         D1[0][0] = 40
    #         D1[1][1] = 15
    #         D1[2][2] = 1
    #         D1[3][3] = 80
    #         D1[4][4] = 100
    #         D1[5][5] = 10
    #         D2 = np.eye(3 * leg_num)
    #         D2[2][2] = 2
    #         D2[5][5] = 2
    #         p = np.dot(np.dot(aArray.T, D1), aArray)
    #         P_mat = arrayToQpmat(p)
    #         q = (- np.dot(np.dot(b.T, D1), aArray)).T
    #         q_mat = arrayToQpmat(q)
    #         G = [[0.0 for i in range(6 * leg_num)] for j in range(3 * leg_num)]
    #         for i in range(leg_num):
    #             for j in range(3):
    #                 G[3 * i + j][6 * i + 2 * j] = 1.0
    #                 G[3 * i + j][6 * i + 2 * j + 1] = -1.0
    #         G_mat = matrix(G)
    #         h = [0 for i in range(6 * leg_num)]
    #         for i in range(leg_num):
    #             h[i * 6] = 1.5 * 20.0 / leg_num
    #             h[i * 6 + 1] = 1.5 * 20.0 / leg_num
    #             h[i * 6 + 2] = 1.5 * 20.0 / leg_num
    #             h[i * 6 + 3] = 1.5 * 20.0 / leg_num
    #             h[i * 6 + 4] = 1.8 * forceTorque[2][0] / leg_num
    #             h[i * 6 + 5] = -0.3 * forceTorque[2][0] / leg_num
    #         H_mat = matrix(h)
    #         solvers.options['show_progress'] = False
    #         result = solvers.qp(P_mat, q_mat, G_mat, H_mat)
    #         result = np.array(result['x'])
    #         error = b - np.dot(aArray, result)
    elif leg_num == 2:
        b = forceTorque
        aUpperArray = np.hstack([np.eye(3) for i in range(leg_num)])
        adownArray = np.hstack([crossproduct_matrix(_leg_poses[i]) for i in groundLeg])
        aArray = np.vstack([aUpperArray, adownArray])
        D1 = 10 * np.eye(6)
        # D1[0][0] = 100
        # D1[1][1] = 30
        # D1[2][2] = 5
        # D1[3][3] = 80
        # D1[4][4] = 200
        # D1[5][5] = 1

        D1[0][0] = 40
        D1[1][1] = 30
        D1[2][2] = 2
        D1[3][3] = 80
        D1[4][4] = 120
        D1[5][5] = 10

        # D1[0][0] = 60
        # D1[1][1] = 20
        # D1[2][2] = 5
        # D1[3][3] = 55
        # D1[4][4] = 120
        # D1[5][5] = 1
        # D2 = np.eye(6)
        # D2[2][2] = 3
        # D2[5][5] = 3
        p = np.dot(np.dot(aArray.T, D1), aArray)
        P_mat = sparse.csc_matrix(p)
        q = (- np.dot(np.dot(b.T, D1), aArray)).T
        q_mat = arrayToQpmat(q)
        G = sparse.eye(3 * leg_num, format='csc')
        l = [0 for i in range(3 * leg_num)]
        for i in range(leg_num):
            l[i * 3] = -1.5 * 30.0 / leg_num
            l[i * 3 + 1] = -1.5 * 30.0 / leg_num
            l[i * 3 + 2] = 0.0 / leg_num
        u = [0 for i in range(3 * leg_num)]
        for i in range(leg_num):
            u[i * 3] = 1.5 * 30.0 / leg_num
            u[i * 3 + 1] = 1.5 * 30.0 / leg_num
            u[i * 3 + 2] = 400.0 / leg_num
        # Create an OSQP object
        prob = osqp.OSQP()
        # Setup workspace and change alpha parameter
        prob.setup(P_mat, q, G, l, u, verbose=False)
        # Solve problem
        res = prob.solve()
        error = b - np.dot(aArray, res.x)
        print("qp_error: ", error)
        return res.x
    # elif leg_num == 1:
    #     if Gait_name == "FAST_running":
    #         b = forceTorque
    #         aUpperArray = np.hstack([np.eye(3) for i in range(leg_num)])
    #         adownArray = np.hstack([crossproduct_matrix(_leg_poses[i]) for i in groundLeg])
    #         aArray = np.vstack([aUpperArray, adownArray])
    #         D1 = 10 * np.eye(6)
    #         D1[0][0] = 0
    #         D1[1][1] = 0
    #         D1[2][2] = 0
    #         D1[3][3] = 40
    #         D1[4][4] = 160
    #         D1[5][5] = 100
    #         p = np.dot(np.dot(aArray.T, D1), aArray)
    #         P_mat = arrayToQpmat(p)
    #         q = (- np.dot(np.dot(b.T, D1), aArray)).T
    #         q_mat = arrayToQpmat(q)
    #         G = [[0.0 for i in range(6 * leg_num)] for j in range(3 * leg_num)]
    #         for i in range(leg_num):
    #             for j in range(3):
    #                 G[3 * i + j][6 * i + 2 * j] = 1.0
    #                 G[3 * i + j][6 * i + 2 * j + 1] = -1.0
    #         G_mat = matrix(G)
    #         h = [0 for i in range(6 * leg_num)]
    #         for i in range(leg_num):
    #             h[i * 6] = 7.
    #             h[i * 6 + 1] = 7.
    #             h[i * 6 + 2] = 10.
    #             h[i * 6 + 3] = 10.
    #             h[i * 6 + 4] = 15.
    #             h[i * 6 + 5] = -10.
    #         H_mat = matrix(h)
    #         # solvers.options['show_progress'] = False
    #         result = solvers.qp(P_mat, q_mat, G_mat, H_mat)
    #         result = np.array(result['x'])
    #         error = b - np.dot(aArray, result)
    #     else:
    #         b = forceTorque
    #         aUpperArray = np.hstack([np.eye(3) for i in range(leg_num)])
    #         adownArray = np.hstack([crossproduct_matrix(_leg_poses[i]) for i in groundLeg])
    #         aArray = np.vstack([aUpperArray, adownArray])
    #         D1 = 10 * np.eye(6)
    #         D1[0][0] = 40
    #         D1[1][1] = 15
    #         D1[2][2] = 10
    #         D1[3][3] = 80
    #         D1[4][4] = 100
    #         D1[5][5] = 10
    #         D2 = np.eye(6)
    #         D2[2][2] = 3
    #         D2[5][5] = 3
    #         p = np.dot(np.dot(aArray.T, D1), aArray)
    #         P_mat = arrayToQpmat(p)
    #         q = (- np.dot(np.dot(b.T, D1), aArray)).T
    #         q_mat = arrayToQpmat(q)
    #         G = [[0.0 for i in range(6 * leg_num)] for j in range(3 * leg_num)]
    #         for i in range(leg_num):
    #             for j in range(3):
    #                 G[3 * i + j][6 * i + 2 * j] = 1.0
    #                 G[3 * i + j][6 * i + 2 * j + 1] = -1.0
    #         G_mat = matrix(G)
    #         h = [0 for i in range(6 * leg_num)]
    #         for i in range(leg_num):
    #             h[i * 6] = 1.5 * 20.0 / leg_num
    #             h[i * 6 + 1] = 1.5 * 20.0 / leg_num
    #             h[i * 6 + 2] = 1.5 * 20.0 / leg_num
    #             h[i * 6 + 3] = 1.5 * 20.0 / leg_num
    #             h[i * 6 + 4] = 1.5 * forceTorque[2][0] / leg_num
    #             h[i * 6 + 5] = -0.5 * forceTorque[2][0] / leg_num
    #         H_mat = matrix(h)
    #         solvers.options['show_progress'] = False
    #         result = solvers.qp(P_mat, q_mat, G_mat, H_mat)
    #         result = np.array(result['x'])
    #         error = b - np.dot(aArray, result)


def PD_center_forceTorque(PD_mode, obj, body_pos, body_orientation, current_angularVel, current_Vel):
    mass = 14.0
    t = 0.01
    [alpha, beta, gama] = body_orientation[0:3]
    R_z = [[np.cos(gama), - np.sin(gama), 0.],
           [np.sin(gama), np.cos(gama), 0.],
           [0., 0., 1.]]

    R_y = [[np.cos(beta), 0., np.sin(beta)],
           [0., 1., 0.],
           [-np.sin(beta), 0., np.cos(beta)]]

    R_x = [[1., 0., 0.],
           [0., np.cos(alpha), -np.sin(alpha)],
           [0., np.sin(alpha), np.cos(alpha)]]

    # Rd_z = [[np.cos(obj[2]), - np.sin(obj[2]), 0.],
    #         [np.sin(obj[2]), np.cos(obj[2]), 0.],
    #         [0., 0., 1.]]
    #
    # Rd_y = [[np.cos(obj[1]), 0., np.sin(obj[1])],
    #         [0., 1., 0.],
    #         [-np.sin(obj[1]), 0., np.cos(obj[1])]]
    #
    # Rd_x = [[1., 0., 0.],
    #         [0., np.cos(obj[0]), -np.sin(obj[0])],
    #         [0., np.sin(obj[0]), np.cos(obj[0])]]

    R = np.dot(R_x, np.dot(R_y, R_z))
    # R_d = np.dot(Ra_d, np.dot(Rb_d, Rc_d))

    if PD_mode == 4:
        # Force
        # Force_P = np.array([[400, 0, 0], [0, 400, 0], [0, 0, 400]])
        # Force_D = np.array([[200, 0, 0], [0, 100, 0], [0, 0, 150]])

        # Force_P = np.array([[1000, 0, 0], [0, 1000, 0], [0, 0, 500]])
        # Force_D = np.array([[200, 0, 0], [0, 200, 0], [0, 0, 200]])

        Force_P = np.array([[1400, 0, 0], [0, 800, 0], [0, 0, 600]])
        Force_D = np.array([[300, 0, 0], [0, 250, 0], [0, 0, 200]])

        # Torque
        # Torque_P = np.array([[50, 0, 0], [0, 100, 0], [0, 0, 50]])
        # Torque_D = np.array([[30, 0, 0], [0, 30, 0], [0, 0, 30]])

        # Torque_P = np.array([[1000, 0, 0], [0, 1000, 0], [0, 0, 1000]])
        # Torque_D = np.array([[200, 0, 0], [0, 200, 0], [0, 0, 200]])

        # Torque_P = np.array([[600, 0, 0], [0, 600, 0], [0, 0, 1300]])
        # Torque_D = np.array([[70, 0, 0], [0, 30, 0], [0, 0, 250]])

        Torque_P = np.array([[600, 0, 0], [0, 700, 0], [0, 0, 600]])
        Torque_D = np.array([[70, 0, 0], [0, 30, 0], [0, 0, 50]])

        if gama >= np.pi / 180 * 8:
            Torque_P = np.array([[600, 0, 0], [0, 600, 0], [0, 0, 1350]])
            Torque_D = np.array([[70, 0, 0], [0, 30, 0], [0, 0, 400]])

    if PD_mode == 2:
        # Force
        # Force_P = np.array([[300, 0, 0], [0, 400, 0], [0, 0, 600]])
        # Force_D = np.array([[200, 0, 0], [0, 200, 0], [0, 0, 200]])

        Force_P = np.array([[300, 0, 0], [0, 200, 0], [0, 0, 600]])
        Force_D = np.array([[200, 0, 0], [0, 300, 0], [0, 0, 120]])

        # Torque
        # Torque_P = np.array([[300, 0, 0], [0, 200, 0], [0, 0, 600]])
        # Torque_D = np.array([[60, 0, 0], [0, 90, 0], [0, 0, 80]])

        Torque_P = np.array([[100, 0, 0], [0, 500, 0], [0, 0, 150]])
        Torque_D = np.array([[80, 0, 0], [0, 35, 0], [0, 0, 100]])

        if beta >= np.pi / 180 * 10:
            Torque_P = np.array([[300, 0, 0], [0, 300, 0], [0, 0, 600]])
            Torque_D = np.array([[30, 0, 0], [0, 600, 0], [0, 0, 80]])

    # forceTorque = np.array([0. for i in range(6)])

    # target
    target_pos = np.array([obj[3], obj[4], obj[5]])
    target_dir = np.array([obj[0], obj[1], obj[2]])
    target_vel = np.array([obj[9], obj[10], obj[11]])
    target_angVel = np.array([obj[6], obj[7], obj[8]])

    # current
    current_pos = np.array(body_pos)
    current_dir = np.array(body_orientation)
    current_vel = np.array(current_Vel)
    current_angVel = np.array(current_angularVel)

    # force and torque
    target_force = np.dot(Force_P, (target_pos - current_pos)) + np.dot(Force_D, (target_vel - current_vel)) + np.array(
        [0., 0., 9.81 * mass])
    target_torque = np.dot(Torque_P, (target_dir - current_dir)) + np.dot(Torque_D, (target_angVel - current_angVel))
    # target_torque = np.dot(Torque_P, np.dot(Rd, R.T)) + np.dot(Torque_D, (target_angVel - current_angVel))
    forceTorque = np.hstack([target_force, target_torque])

    R_forceTorque = np.kron(np.eye(2), R)
    R_forceTorque[3:6, 3:6] = np.eye(3)
    # print(R_forceTorque)
    forceTorque = np.dot(R_forceTorque, forceTorque.T).T
    # forceTorque = forceTorque.tolist()

    # limit
    fT_limit = [20., 20., 200., 15., 15., 15.]

    for i in range(6):
        if forceTorque[i] > fT_limit[i]:
            forceTorque[i] = fT_limit[i]

        elif forceTorque[i] < - fT_limit[i]:
            forceTorque[i] = - fT_limit[i]

    return forceTorque


if __name__ == '__main__':
    _leg_poses1 = np.array([[0.11, -0.266, -0.22], [-0.10, -0.247, -0.32],
                            [0.11, 0.262, -0.31], [-0.11, 0.244, -0.21]])
    _leg_poses2 = np.array([[0.13, -0.275, -0.30], [-0.13, -0.235, -0.30],
                            [0.13, 0.275, -0.30], [-0.13, 0.235, -0.30]])
    groundLeg = [1, 2]
    forceTorque = np.array([1., 0., 150., -4.0, -5.0, -0.0])
    import time

    a = time.time()
    start_time = time.time()
    force = -1 * QP_solve(_leg_poses2, groundLeg, forceTorque, 'walk')
    # print(time.time() - start_time)
    # print(QP_solve(_leg_poses2,groundLeg,forceTorque,"FAST_running")[1])
    # print(time.time()-a)
