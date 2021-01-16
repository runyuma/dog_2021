#!/usr/bin/env python2
# coding:utf-8
# SHOW_GAIT = 1
# JUMP_OVER = 0#跨高栏
# ACROSS_TUBE = 0#过排管
# show_slowwalK = 0 #慢步
# show_fastwalk = 0 #快步
# show_run = 0# 跑步
# joystick_control = 0
# test_mpc = 0
# test_mpc2 = 1
# T = 0.008

TIME_STEP = 1
time_step = 8 #上层主循环timestep
bounce_iter = 3 #起跳弹跳时间
DEFAULT_HEIGHT = 0.28#机器人默认离地高度
import numpy as np
g = np.array([[0],[0],[-9.81]])