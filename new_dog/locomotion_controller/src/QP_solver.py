#!/usr/bin/env python2
# coding:utf-8
#二次规划解足端力的函数
import numpy as np
import osqp
from scipy import sparse
from cvxopt import matrix, solvers
#这个matrix和np.array np.mat 不一样 np是行优先 cvxopt 是列优先

import time
def arrayToQpmat(array):
    return matrix((array.T).tolist())
def crossproduct_matrix( footpoint_array):
    # 用矩阵表示cross product
    return (np.array([[0, - footpoint_array[2][0], footpoint_array[1][0]],
                      [footpoint_array[2][0], 0, - footpoint_array[0][0]],
                      [- footpoint_array[1][0], footpoint_array[0][0], 0]]))


def decompressForce(ground_leg,array):
    force_list = []
    for i in range(4):
        if ground_leg[i] != 1:
            force_list.append(None)
        else:
            force,array = np.vsplit(array,(3,))
            force_list.append(force)
    return force_list

def QP_solve(_leg_poses,groundLeg,forceTorque):
    leg_num = sum(groundLeg)
    if leg_num == 4:
        dia = [100, 40., 20., 80, 200., 50]

    elif leg_num == 2 or 3:
        dia = [60., 30., 2., 80, 120., 10]

    elif leg_num == 0:
        return None, None
    b = forceTorque
    aUpperArray = np.hstack([np.eye(3) for i in range(leg_num)])
    hstack_list = []
    for i in range(4):
        if groundLeg[i] == 1:
            hstack_list.append(crossproduct_matrix(_leg_poses[i]))
    adownArray = np.hstack(hstack_list)
    aArray = np.vstack([aUpperArray, adownArray])
    D1 = np.diag(dia)
    p = np.dot(np.dot(aArray.T, D1), aArray)
    P_mat = arrayToQpmat(p)
    q = (- np.dot(np.dot(b.T, D1), aArray)).T
    q_mat = arrayToQpmat(q)
    G = [[0.0 for i in range(6 * leg_num)] for j in range(3 * leg_num)]
    for i in range(leg_num):
        for j in range(3):
            G[3 * i + j][6 * i + 2 * j] = 1.0
            G[3 * i + j][6 * i + 2 * j + 1] = -1.0
    G_mat = matrix(G)
    h = [0 for i in range(6 * leg_num)]
    for i in range(leg_num):
        h[i * 6] = 1.5 * 25.0 / leg_num
        h[i * 6 + 1] = 1.5 * 25.0 / leg_num
        h[i * 6 + 2] = 1.5 * 25.0 / leg_num
        h[i * 6 + 3] = 1.5 * 25.0 / leg_num
        h[i * 6 + 4] = 1.5 * forceTorque[2][0] / leg_num
        h[i * 6 + 5] = -0.5 * forceTorque[2][0] / leg_num
    H_mat = matrix(h)
    solvers.options['show_progress'] = False
    result = solvers.qp(P_mat, q_mat, G_mat, H_mat)
    result = np.array(result['x'])
    force_list = decompressForce(groundLeg, result)
    error = b - np.dot(aArray, result)
    error = b - np.dot(aArray, result)
    return force_list, error

def OSQP_solve(_leg_poses,groundLeg,forceTorque):
    miu = 0.5
    leg_num = sum(groundLeg)
    if leg_num == 4:
        dia = [100, 40., 20., 80, 200., 50]

    elif leg_num ==3:
        dia = [80, 40., 20., 160, 200., 50]
    elif leg_num == 2 :
        dia = [40., 20., 5., 80, 120., 10]
        # dia = [120., 20., 1., 80, 160., 10]

    elif leg_num == 0 :
        return None,None

    b = forceTorque
    aUpperArray = np.hstack([np.eye(3) for i in range(leg_num)])
    hstack_list = []
    for i in range(4):
        if groundLeg[i] == 1:
            hstack_list.append(crossproduct_matrix(_leg_poses[i]))
    adownArray = np.hstack(hstack_list)
    aArray = np.vstack([aUpperArray, adownArray])
    Ax = sparse.csc_matrix(aArray.data)
    D = sparse.diags(dia)
    P = Ax.T.dot(D).dot(Ax)
    Q = -(np.dot(b.T,np.diag(dia)).dot(aArray)).T

    A1 = np.zeros((leg_num,leg_num*3))
    A2 = np.zeros((leg_num*4,leg_num*3))
    for i in range(leg_num):
        A1[i][i*3+2] = 1
        A2[i*4][i*3] = 1
        A2[i * 4+1][i * 3] = 1
        A2[i * 4+2][i * 3+1] = 1
        A2[i * 4+3][i * 3+1] = 1
        A2[i * 4][i * 3 +2] = -miu
        A2[i * 4+1][i * 3+2] = miu
        A2[i * 4+2][i * 3+2] = -miu
        A2[i * 4+3][i * 3+2] = miu


    A = np.vstack([A1,A2])
    A = sparse.csc_matrix(A.data)
    l = np.zeros((leg_num*5,1))
    u = np.zeros((leg_num*5,1))
    for i in range(leg_num):
        l[i][0] = 2
        u[i][0] = 1.5 * forceTorque[2][0] / leg_num
        l[i*4 + 0 + leg_num][0] = - float("inf")
        l[i * 4 + 1 + leg_num][0] = 0
        l[i * 4 + 2 + leg_num][0] = - float("inf")
        l[i * 4 + 3 + leg_num][0] = 0
        u[i * 4 + 0 + leg_num][0] = 0
        u[i * 4 + 1 + leg_num][0] = float("inf")
        u[i * 4 + 2 + leg_num][0] = 0
        u[i * 4 + 3 + leg_num][0] = float("inf")



    # A = np.eye(leg_num*3)
    # A = sparse.csc_matrix(A.data)
    # l = np.zeros((leg_num*3,1))
    # u = np.zeros((leg_num*3,1))
    #
    # if leg_num == 2 or leg_num == 4:
    #     for i in range(leg_num):
    #         l[i * 3][0] = - 1.5 * 30.0 / leg_num
    #         u[i * 3][0] = 1.5 * 30.0 / leg_num
    #         l[i * 3 + 1][0] = - 1.5 * 30.0 / leg_num
    #         u[i * 3 + 1][0] = 1.5 * 30.0 / leg_num
    #         l[i * 3 + 2][0] = 0.5 * forceTorque[2][0] / leg_num
    #         u[i * 3 + 2][0] = 1.5 * forceTorque[2][0] / leg_num
    # elif leg_num == 3:
    #     for i in range(leg_num):
    #         l[i * 3][0] = - 1.5 * 30.0 / leg_num
    #         u[i * 3][0] = 1.5 * 30.0 / leg_num
    #         l[i * 3 + 1][0] = - 1.5 * 30.0 / leg_num
    #         u[i * 3 + 1][0] = 1.5 * 30.0 / leg_num
    #         l[i * 3 + 2][0] = 0.5 * forceTorque[2][0] / leg_num
    #         u[i * 3 + 2][0] = 1.5 * forceTorque[2][0] / leg_num
    #     nonground_leg = groundLeg.index(0)
    #     false_leg = 3 - nonground_leg
    #     if nonground_leg > false_leg:
    #         l[false_leg * 3][0] =  - 1.5 * 20.0 / leg_num
    #         u[false_leg * 3][0] = 1.5 * 20.0 / leg_num
    #         l[false_leg * 3 + 1][0] = -1.5 * 20.0 / leg_num
    #         u[false_leg * 3 + 1][0] = 1.5 * 30.0 / leg_num
    #         l[false_leg * 3 + 2][0] = 5
    #         u[false_leg * 3 + 2][0] = 1.5 * forceTorque[2][0] / leg_num
    #     else:
    #         l[(false_leg - 1) * 3][0] = -1.5 * 20.0 / leg_num
    #         u[(false_leg - 1) * 3][0] = 1.5 * 20.0 / leg_num
    #         l[(false_leg - 1) * 3 + 1][0] = -1.5 * 20.0 / leg_num
    #         u[(false_leg - 1) * 3 + 1][0] = 1.5 * 20.0 / leg_num
    #         l[(false_leg - 1) * 3 + 2][0] = 4
    #         u[(false_leg - 1) * 3 + 2][0] = 1.5 * forceTorque[2][0] / leg_num



    prob = osqp.OSQP()
    prob.setup(P,Q,A,l,u,warm_start=True,verbose = 0)
    res = prob.solve()

    force_list = decompressForce(groundLeg,np.array([res.x]).T)
    error = b - np.dot(aArray,np.array([res.x]).T)
    return force_list,error




if __name__ == '__main__':
    _leg_poses1 = [np.array([[0.13], [-0.255], [-0.30]]), np.array([[-0.13], [-0.255], [-0.30]]),
                   np.array([[0.13], [0.255], [-0.30]]), np.array([[-0.13], [0.255], [-0.30]])]
    _leg_poses2 = [np.array([[0.13], [-0.275], [-0.30]]), np.array([[-0.13], [-0.235], [-0.30]]),
                   np.array([[0.13], [0.275], [-0.30]]), np.array([[-0.13], [0.235], [-0.30]])]
    _leg_poses3 = [np.array([[0.13], [-0.275], [-0.1]]), np.array([[-0.13], [-0.235], [-0.1]]),
                   np.array([[0.13], [0.275], [-0.1]]), np.array([[-0.13], [0.235], [-0.10]])]
    foot_point1 = [np.array([[0.11420627], [-0.25916231], [-0.28805986]]),
                   np.array([[-0.11569536], [-0.25253401], [-0.23535509]]),
                   np.array([[0.11801195], [0.25848593], [-0.23491439]]),
                   np.array([[-0.10673481], [0.25096339], [-0.28649375]])]
    foot_point2 = [np.array([[0.1122368], [-0.24855255], [-0.27783638]]),
                   np.array([[-0.11004706], [-0.24085042], [-0.21564755]]),
                   np.array([[0.11904662], [0.26646306], [-0.22344087]]),
                   np.array([[-0.10747138], [0.24402521], [-0.27999446]])]
    groundLeg1 = [0, 1, 1, 0]
    groundLeg2 = [1, 1, 1, 1]
    groundLeg3 = [1, 0, 0, 1]
    groundLeg4 = [1,1,0,1]
    forceTorque1 = np.array([[0], [0], [150], [-4.0], [-5.0], [0.0]])
    forceTorque2 = np.array([[-3.32053612], [3.33314112], [149.18763162], [5.36858006], [-9.35635371], [1.9199625]])
    forceTorque3 = np.array(
        [[28.29341204], [26.56256855], [152.90062488], [-18.27738389], [19.53474339], [-0.41216909]])
    a = time.time()
    import time

    start_time = time.time()
    OSQP_solve(_leg_poses2, groundLeg3, forceTorque1)
    print(time.time() - start_time)
    print(OSQP_solve(foot_point2, groundLeg3, forceTorque3))
    print(OSQP_solve(_leg_poses1, groundLeg4, forceTorque2))
    # print(time.time()-a)


