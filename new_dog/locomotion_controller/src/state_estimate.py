import numpy as np
import math
import time

T = 0.01


def get_next_state(ori, pos, vel, ang_vel, gaittime, N):
    # ori = [theta, phi, gama]
    # pos = [x, y, z]
    # vel = [vx, vy, vz]
    # ang_vel = [wx, wy, wz]
    # gaittime: float*1 time of current gait
    # N: horizon of MPC
    nsim = int(gaittime / T) + N + 1
    obj = []
    # for i in range(nsim):
    #     if -0.1 < pos[1] + vel[1] * T * 0.5 < 0.1:
    #         obj.append([0., 0., 0.,
    #                     pos[0] + vel[0] * i * T, (pos[1] + vel[1] * i * T) * 0.5, 0.4,
    #                     0., 0., 0.,
    #                     0., 0., 0.,
    #                     -9.81])
    #     elif pos[1] + vel[1] * T * 0.5 < -0.1:
    #         obj.append([0., 0., 0.,
    #                     pos[0] + vel[0] * i * T, -0.1, 0.4,
    #                     0., 0., 0.,
    #                     0., 0., 0.,
    #                     -9.81])
    #     elif pos[1] + vel[1] * T * 0.5 > 0.1:
    #         obj.append([0., 0., 0.,
    #                     pos[0] + vel[0] * i * T, 0.1, 0.4,
    #                     0., 0., 0.,
    #                     0., 0., 0.,
    #                     -9.81])

    for i in range(nsim):
        obj.append([0., 0., 0.,
                    0., 0., 0.4,
                    0., 0., 0.,
                    0., 0., 0.,
                    -9.81])

    return obj


def get_test_state(ori, pos, vel, ang_vel, gaittime, gait_currenttime, N):
    test_mode = -1
    # 0:x 1:y 2:z 3:X 4:Y 5:Z -1:stand

    nsim = int(gaittime / T) + N + 1
    obj = []
    if test_mode == 4:
        for i in range(nsim):
            if i < nsim / 2:
                obj.append([0., np.pi * 8 / 180, 0.,
                            0.005, 0., 0.4,
                            0., 0., 0.,
                            0., 0., 0.,
                            -9.81])

                # obj.append([0., np.pi * 1 / 180 * np.sin((i * 1.0 / (nsim / 2))) * np.pi, 0.,
                #             0., 0., 0.4,
                #             0., 0., 0.,
                #             0., 0., 0.,
                #             -9.81])

            elif i >= nsim / 2:
                obj.append([0., np.pi * 8 / 180, 0.,
                            0.005, 0., 0.4,
                            0., 0., 0.,
                            0., 0., 0.,
                            -9.81])


    if test_mode == 5:
        for i in range(nsim):
            # if i < nsim / 2:
            #     obj.append([0., 0., 0.,
            #                 0.008 * np.sin((i * 1.0 / (nsim / 2)) * np.pi), -0.07 * np.sin((i * 1.0 / (nsim / 2)) * np.pi), 0.3,
            #                 0., 0., 0.,
            #                 0., 0., 0.,
            #                 -9.81])
            #
            # elif i >= nsim / 2:
            #     obj.append([0., 0., 0.,
            #                 0.008 * np.sin((i * 1.0 / (nsim / 2)) * np.pi), -0.07 * np.sin((i * 1.0 / (nsim / 2)) * np.pi), 0.3,
            #                 0., 0., 0.,
            #                 0., 0., 0.,
            #                 -9.81])

            if i < nsim / 2:
                obj.append([0., 0., np.pi * 10 / 180,
                            0., 0., 0.4,
                            0., 0., 0.,
                            0., 0., 0.,
                            -9.81])

            elif i >= nsim / 2:
                obj.append([0., 0., np.pi * 10 / 180,
                            0., 0., 0.4,
                            0., 0., 0.,
                            0., 0., 0.,
                            -9.81])

    if test_mode == -1:
        for i in range(nsim):
            obj.append([0., 0., 0.,
                        0., 0., 0.4,
                        0., 0., 0.,
                        0., 0., 0.,
                        -9.81])

    return obj
