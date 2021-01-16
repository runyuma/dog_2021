import osqp
import numpy as np
import scipy as sp
from scipy import sparse
import matplotlib.pyplot as plt
import math


# Body coordinate system:
# x forward, y left
# 0, 1; 2, 3
# function
def getArmmat(footpoint_array):
    arm_mat = [[0, - footpoint_array[2], footpoint_array[1]],
               [footpoint_array[2], 0, - footpoint_array[0]],
               [- footpoint_array[1], footpoint_array[0], 0]]

    return arm_mat


def Nproduct(A, N):
    [na, nb] = A.shape
    if na != nb:
        raise ArithmeticError("dimention is wrong in Nproduct")

    Adot = np.eye(na)
    if N == 0:
        return np.eye(na)
    elif N > 0:
        for i in range(0, N):
            Adot = np.dot(Adot, A)

        return Adot

    elif N < 0:
        raise ArithmeticError("N is wrong in Nproduct")


def ifPositive(A):
    B = np.linalg.eigvals(A)
    if np.all(B > 0):
        return True
    else:
        return False


def qp_MPC(leg_poses, dog_obj, body_pos, body_orientation, current_angularVel, current_Vel, leg_num, groundLeg):
    # leg_poses: lenth: 12 4*[rx, ry, rz]
    # dog_obj: lenth:12 [phi, theta, gama, x, y, z, wx, wy, wz, vx, vy, vz, -9.81]
    # body_pos: lenth:3
    # body_orientation: lenth:3
    # current_angularVel: lenth:3
    # current_Vel: lenth:3
    # leg_num: int
    # groundLeg: lenth:leg_num
    # information
    footarm_mat = []
    for i in range(4):
        footarm_mat.append(getArmmat(leg_poses[i]))
    footarm_mat = np.array(footarm_mat)
    mass = 14.0
    Ineb = np.mat([[2, 0., 0.],
                   [0., 0.4, 0.],
                   [0., 0., 2]])

    N = 10
    t = 0.01
    # plotx = []
    # ploty = []
    # plotz = []
    # plotvx = []
    # plotvy = []
    # plotvz = []

    # Objective trajectory
    # dog_obj = []
    # for i in range(300):
    #     if i < 200:
    #         dog_obj.append([0., 0., 0., 0., 0., 0.3 - t * 0.1 * i, 0., 0., 0., 0., 0., -0.1, -9.81])
    #     if 200 <= i < 300:
    #         dog_obj.append([0., 0., 0., 0., 0., 0.14 + 0.5 * np.square(t * (i - 200)), 0., 0., 0., 0., 0., t * (i - 200),
    #                         -9.81])
    #     # if 200 <= i < 300:
    #     #     dog_obj.append([0., 0., 0., 0., 0., 0.3, 0., 0., 0., 0.008 * i, 0., 0., -9.81])
    # dog_obj = np.array(dog_obj)

    # initial statement
    # print("body_orientation: ", body_orientation.reshape(1, 3))
    # print("body_pos: ", body_pos.reshape(1, 3))
    # print("current_angularVel: ", current_angularVel.reshape(1, 3))
    # print("current_Vel: ", current_Vel.reshape(1, 3))
    # [phi, theta, gama] = np.squeeze(body_orientation.reshape(1, 3))
    # [x, y, z] = np.squeeze(body_pos.reshape(1, 3))
    # [wx, wy, wz] = np.squeeze(current_angularVel.reshape(1, 3))
    # [vx, vy, vz] = np.squeeze(current_Vel.reshape(1, 3))
    [phi, theta, gama] = np.squeeze(body_orientation)
    [x, y, z] = np.squeeze(body_pos)
    # print("current)angularVel: ::::::::::::", current_angularVel)
    [wx, wy, wz] = np.squeeze(current_angularVel)
    [vx, vy, vz] = np.squeeze(current_Vel)
    x0 = np.array([phi, theta, gama, x, y, z, wx, wy, wz, vx, vy, vz, -9.81])
    print("mpc_x0: ", x0)

    # print("[phi, ", "theta, ", "gama]: ", phi, theta, gama)
    # print("[x, ", "y, ", "z]: ", x, y, z)
    # print("[wx, ", "wy, ", "wz]: ", wx, wy, wz)
    # print("[vx, ", "vy, ", "vz]: ", vx, vy, vz)
    # print("x0 is here: ", x0)
    [nx, nu] = [13, 12]

    # MPC
    prob = osqp.OSQP()

    # In world coordinate system
    # plotz_hor = []
    # plotx_err_hor = []
    # plotz_force = []

    # Ine
    R_gama = np.mat([[np.cos(gama), np.sin(gama), 0.],
                     [- np.sin(gama), np.cos(gama), 0.],
                     [0., 0., 1.]])
    # R_gama, body to world, which is wrong in the article
    Ine = np.dot(R_gama, np.dot(Ineb, R_gama.T))
    Ine_1 = np.linalg.inv(Ine)
    # Ad
    Ad = np.eye(13)
    omegaTophi = [[np.cos(gama) * t, - np.sin(gama) * t, 0.],
                  [np.sin(gama) * t, np.cos(gama) * t, 0.],
                  [0., 0., 1. * t]]
    Ad[0:3, 6:9] = omegaTophi
    Ad[3:6, 9:12] = [[1. * t, 0., 0.],
                     [0., 1. * t, 0.],
                     [0., 0., 1. * t]]
    Ad[11, 12] = t
    # Ad = sparse.csc_matrix(Ad)
    # Bd
    """
    problem: footarm_mat is defined in body coordinate system
    """
    Bd = np.zeros((13, 12))
    if leg_num == 4:
        Bd[6:9, 0:3] = np.dot(Ine_1, footarm_mat[0]) * t
        Bd[6:9, 3:6] = np.dot(Ine_1, footarm_mat[1]) * t
        Bd[6:9, 6:9] = np.dot(Ine_1, footarm_mat[2]) * t
        Bd[6:9, 9:12] = np.dot(Ine_1, footarm_mat[3]) * t
        Bd[9:12, 0:3] = np.array([[1. / mass * t, 0., 0.], [0., 1. / mass * t, 0.], [0., 0., 1. / mass * t]])
        Bd[9:12, 3:6] = np.array([[1. / mass * t, 0., 0.], [0., 1. / mass * t, 0.], [0., 0., 1. / mass * t]])
        Bd[9:12, 6:9] = np.array([[1. / mass * t, 0., 0.], [0., 1. / mass * t, 0.], [0., 0., 1. / mass * t]])
        Bd[9:12, 9:12] = np.array([[1. / mass * t, 0., 0.], [0., 1. / mass * t, 0.], [0., 0., 1. / mass * t]])
    elif leg_num == 2:
        if 0 in groundLeg and 3 in groundLeg:
            Bd[6:9, 0:3] = np.dot(Ine_1, footarm_mat[0]) * t
            # Bd[6:9, 3:6] = np.dot(Ine_1, footarm_mat[1]) * t
            # Bd[6:9, 6:9] = np.dot(Ine_1, footarm_mat[2]) * t
            Bd[6:9, 9:12] = np.dot(Ine_1, footarm_mat[3]) * t
            Bd[9:12, 0:3] = np.array([[1. / mass * t, 0., 0.], [0., 1. / mass * t, 0.], [0., 0., 1. / mass * t]])
            # Bd[9:12, 3:6] = np.array([[1. / mass * t, 0., 0.], [0., 1. / mass * t, 0.], [0., 0., 1. / mass * t]])
            # Bd[9:12, 6:9] = np.array([[1. / mass * t, 0., 0.], [0., 1. / mass * t, 0.], [0., 0., 1. / mass * t]])
            Bd[9:12, 9:12] = np.array([[1. / mass * t, 0., 0.], [0., 1. / mass * t, 0.], [0., 0., 1. / mass * t]])
        elif 1 in groundLeg and 2 in groundLeg:
            # Bd[6:9, 0:3] = np.dot(Ine_1, footarm_mat[0]) * t
            Bd[6:9, 3:6] = np.dot(Ine_1, footarm_mat[1]) * t
            Bd[6:9, 6:9] = np.dot(Ine_1, footarm_mat[2]) * t
            # Bd[6:9, 9:12] = np.dot(Ine_1, footarm_mat[3]) * t
            # Bd[9:12, 0:3] = np.array([[1. / mass * t, 0., 0.], [0., 1. / mass * t, 0.], [0., 0., 1. / mass * t]])
            Bd[9:12, 3:6] = np.array([[1. / mass * t, 0., 0.], [0., 1. / mass * t, 0.], [0., 0., 1. / mass * t]])
            Bd[9:12, 6:9] = np.array([[1. / mass * t, 0., 0.], [0., 1. / mass * t, 0.], [0., 0., 1. / mass * t]])
            # Bd[9:12, 9:12] = np.array([[1. / mass * t, 0., 0.], [0., 1. / mass * t, 0.], [0., 0., 1. / mass * t]])
    # Constraints
    u0 = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
    umin = np.array([-20., -20., 0., -20., -20., 0., -20., -20., 0., -20., -20., 0.]) - u0
    umax = np.array([20., 20., 200., 20., 20., 200., 20., 20., 200., 20., 20., 200.]) - u0
    xmin = np.array(
        [-np.inf, -np.inf, -np.inf, -np.inf, -np.inf, 0.0, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf,
         -np.inf, -10.])
    xmax = np.array(
        [np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, -5.])

    # Objective function
    # Q = sparse.diags([3000., 4000., 2500., 1., 1., 8000., 0.01, 0.01, 300, 1500, 1500, 5000, 0.])
    Q = sparse.diags([500., 2500., 3000., 0., 0., 5000., 2.5, 2.5, 300, 2500, 2500, 4000, 0.])
    # Q = sparse.diags([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
    QN = Q
    R = 0.001 * sparse.eye(12)

    # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
    # - quadratic objective
    P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                           sparse.kron(sparse.eye(N), R)], format='csc')
    print(ifPositive(P.toarray()))
    # - linear objective
    # print(Q.shape, dog_obj[times].shape)
    # print(x0)
    q = np.hstack([-Q.dot(dog_obj[0]), -Q.dot(dog_obj[1]), -Q.dot(dog_obj[2]),
                   -Q.dot(dog_obj[3]), -Q.dot(dog_obj[4]), -Q.dot(dog_obj[5]),
                   -Q.dot(dog_obj[6]), -Q.dot(dog_obj[7]), -Q.dot(dog_obj[8]),
                   -Q.dot(dog_obj[9]), -QN.dot(dog_obj[10]), np.zeros(N * nu)])
    # q = np.hstack([-np.dot((dog_obj[times + 0]), QA), -np.dot((dog_obj[times + 1]), QA), -np.dot((dog_obj[times + 2]), QA),
    #                -np.dot((dog_obj[times + 3]), QA), -np.dot((dog_obj[times + 4]), QA), -np.dot((dog_obj[times + 5]), QA),
    #                -np.dot((dog_obj[times + 6]), QA), -np.dot((dog_obj[times + 7]), QA), -np.dot((dog_obj[times + 8]), QA),
    #                -np.dot((dog_obj[times + 9]), QA), -np.dot((dog_obj[times + 10]), QN), np.zeros(N * nu)])
    #
    # q = np.hstack([-Q.dot(dog_obj[times + 0]).T, -Q.dot(dog_obj[times + 1]).T, -Q.dot(dog_obj[times + 2]).T,
    #                -Q.dot(dog_obj[times + 3]).T, -Q.dot(dog_obj[times + 4]).T, -Q.dot(dog_obj[times + 5]).T,
    #                -Q.dot(dog_obj[times + 6]).T, -Q.dot(dog_obj[times + 7]).T, -Q.dot(dog_obj[times + 8]).T,
    #                -Q.dot(dog_obj[times + 9]).T, -QN.dot(dog_obj[times + 10]).T, np.zeros(N * nu).T])
    # print("obj1:", dog_obj[times + 0])
    # print("obj2:", dog_obj[times + 1])
    # print("obj3:", dog_obj[times + 2])
    # print("obj4:", dog_obj[times + 3])
    # - linear dynamics
    Ax = sparse.kron(sparse.eye(N + 1), -sparse.eye(nx)) + sparse.kron(sparse.eye(N + 1, k=-1), Ad)
    Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
    Aeq = sparse.hstack([Ax, Bu])
    # leq = np.hstack([-x0, np.kron(np.ones(N), g_mat)])
    leq = np.hstack([-x0, np.zeros(N * nx)])
    ueq = leq
    # - input and state constraints
    Aineq = sparse.eye((N + 1) * nx + N * nu)
    lineq = np.hstack([np.kron(np.ones(N + 1), xmin), np.kron(np.ones(N), umin)])
    uineq = np.hstack([np.kron(np.ones(N + 1), xmax), np.kron(np.ones(N), umax)])
    # - OSQP constraints
    A = sparse.vstack([Aeq, Aineq], format='csc')
    l = np.hstack([leq, lineq])
    u = np.hstack([ueq, uineq])
    # Setup workspace
    prob.setup(P, q, A, l, u, warm_start=True)
    res = prob.solve()
    # Check solver status
    if res.info.status != 'solved':
        raise ValueError('OSQP did not solve the problem!')
    # Apply first control input to the plant
    ctrl = res.x[(N + 1) * nx: (N + 1) * nx + nu]
    # print("ctrl:  ", ctrl.reshape(4, 3))

    # update X0
    print("ctrl: ", ctrl.reshape(4, 3))
    x0 = Ad.dot(x0) + Bd.dot(ctrl)  # - g_mat#######
    print("new_x0: ", x0)
    forces = []
    # forces = [np.array([[ctrl[0]], [ctrl[1]], [ctrl[2]]]), np.array([[ctrl[3]], [ctrl[4]], [ctrl[5]]]),
    #           np.array([[ctrl[6]], [ctrl[7]], [ctrl[8]]]), np.array([[ctrl[9]], [ctrl[10]], [ctrl[11]]])]
    forces = [-ctrl[0], -ctrl[1], -ctrl[2], -ctrl[3], -ctrl[4], -ctrl[5],
              -ctrl[6], -ctrl[7], -ctrl[8], -ctrl[9], -ctrl[10], -ctrl[11]]
    # print("forces: ", forces)
    # print(forces[0][0]) # + forces[3] + forces[6] + forces[9])
    return forces
    # forces: lenth:12 [ 4 * np.array([[fx],[fy],[fz]]) ]
