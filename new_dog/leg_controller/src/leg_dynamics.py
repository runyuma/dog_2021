#!/usr/bin/env python3
# coding:utf-8
# AUTHER: smart_mry
# calculate the dynamics of leg
import numpy as np
class leg_args():
    def __init__(self,mass_list, lenth_list, discomlist):
        # class of the parameter of leg
        self.mass_list = mass_list
        self.lenth_list = lenth_list
        self.discomlist = discomlist # distance from massenter to joint

def get_footpointpos(LorR,jointpos_list,args):
    #TARGET:function to get foot_point(type: np.ndarry(3*1))
    #MEANING:vector from hip to footpoint
    #Frame:bofy frame
    #INPUT: LorR(left or right):left:1 right:-1
    #       jointpos:joint pos value list[3]
    #       args:class object of argument
    tran_mat = np.array([[LorR * np.cos(jointpos_list[0]),LorR *  np.sin(jointpos_list[0]) * np.cos(jointpos_list[1]),
                            LorR * np.sin(jointpos_list[0]) * np.cos(jointpos_list[1] + jointpos_list[2])],
                           [0, np.sin(jointpos_list[1]), np.sin(jointpos_list[1] + jointpos_list[2])],
                           [np.sin(jointpos_list[0]), - np.cos(jointpos_list[0]) * np.cos(jointpos_list[1]),
                            - np.cos(jointpos_list[0]) * np.cos(jointpos_list[1] + jointpos_list[2])]])
    lentharray =  np.array([args.lenth_list]).T
    foot_point = np.dot(tran_mat,lentharray)
    return foot_point

def get_jacobian(LorR,jointpos_list,args):
    # TARGET:function to get jacobian mat(type: np.ndarry(3*1))
    # MEANING:vector from hip to footpoint
    # Frame:bofy frame
    # INPUT: LorR(left or right):left:1 right:-1
    #       jointpos:joint pos value list[3]
    #       args:class object of argument
    lenth_1, lenth_2, lenth_3 = args.lenth_list
    Jacobian = np.array([[LorR * (-np.sin(jointpos_list[0]) * lenth_1 + np.cos(jointpos_list[0]) * (np.cos(jointpos_list[1]) * lenth_2 + np.cos(jointpos_list[1] + jointpos_list[2]) * lenth_3)),
             LorR * (np.sin(jointpos_list[0]) * (np.sin(jointpos_list[1]) * lenth_2 + np.sin(jointpos_list[1] + jointpos_list[2]) * lenth_3)),
             LorR * (np.sin(jointpos_list[0]) * np.sin(jointpos_list[1] + jointpos_list[2]) * lenth_3)],
            [0,
             np.cos(jointpos_list[1]) * lenth_2 + np.cos(jointpos_list[1] + jointpos_list[2]) * lenth_3,
             np.cos(jointpos_list[1] + jointpos_list[2]) * lenth_3],
            [np.cos(jointpos_list[0]) * lenth_1 - np.sin(jointpos_list[0]) * (np.cos(jointpos_list[1]) * lenth_2 + np.cos(jointpos_list[1] + jointpos_list[2]) * lenth_3),
             np.cos(jointpos_list[0]) * (np.sin(jointpos_list[1]) * lenth_2 + np.sin(jointpos_list[1] + jointpos_list[2]) * lenth_3),
             np.cos(jointpos_list[0]) * np.sin(jointpos_list[1] + jointpos_list[2]) * lenth_3]])
    return Jacobian

def get_footpointvel(jac,jointvel_list):
    # TARGET:function to get foot_pointvel(type: np.ndarry(3*1))
    # MEANING:vector from hip to footpoint its velocity
    # Frame:bofy frame
    # INPUT:jac: jacobian mat of leg
    #       jointvel_list:joint pos value list[3]
    return np.dot(jac,np.array([jointvel_list]).T)

def get_tauff(LorR,joint_pos,joint_vel,args):
    # feed forward troque(coriolis force & gravity)
    # Frame:bofy frame
    # INPUT:joint_pos: joint_pos value list[3]
    #       jointvel_list:joint vel value list[3]
    h = - args.mass_list[2] * args.discomlist[2] * args.lenth_list[1] * np.sin(
        joint_pos[2])
    C_mat = np.array([[0, 0, 0],
                    [0, h * joint_vel[2],h * (joint_vel[1] + joint_vel[2])],
                    [0, - h * joint_vel[1], 0]])
    gravty = np.mat([[0], [0], [- 9.8]])
    taug = np.zeros((3, 1))
    Ja_l2 = np.mat([[LorR * (-np.sin(joint_pos[0]) * args.lenth_list[0] + np.cos(joint_pos[0]) * (np.cos(joint_pos[1]) * args.discomlist[1])),
                     LorR * (np.sin(joint_pos[0]) * (np.sin(joint_pos[1]) * args.discomlist[1])),
                     0],
                    [0,
                     np.cos(joint_pos[1]) * args.discomlist[1],
                     0],
                    [np.cos(joint_pos[0]) * args.lenth_list[0] - np.sin(joint_pos[0]) * (np.cos(joint_pos[1]) * args.discomlist[1]),
                     np.cos(joint_pos[0]) * (np.sin(joint_pos[1]) * args.discomlist[1]),
                     0]])
    Ja_l3 = np.mat([[LorR * (-np.sin(joint_pos[0]) * args.lenth_list[0] + np.cos(joint_pos[0]) * (np.cos(joint_pos[1]) * args.lenth_list[1]+ np.cos(joint_pos[1] + joint_pos[2]) * args.discomlist[2])),
                     LorR * (np.sin(joint_pos[0]) * (np.sin(joint_pos[1]) * args.lenth_list[1] + np.sin(joint_pos[1] + joint_pos[2]) * args.discomlist[2])),
                     LorR * (np.sin(joint_pos[0]) * np.sin(joint_pos[1] + joint_pos[2]) * args.discomlist[2])],
                    [0,
                     np.cos(joint_pos[1]) * args.lenth_list[1] + np.cos(joint_pos[1] + joint_pos[2]) * args.discomlist[2],
                     np.cos(joint_pos[1] + joint_pos[2]) * args.discomlist[2]],
                    [np.cos(joint_pos[0]) * args.lenth_list[0] - np.sin(joint_pos[0]) * (np.cos(joint_pos[1]) * args.lenth_list[1]+ np.cos(joint_pos[1] + joint_pos[2]) * args.discomlist[2]),
                     np.cos(joint_pos[0]) * (np.sin(joint_pos[1]) * args.lenth_list[1]+np.sin(joint_pos[1] + joint_pos[2]) * args.discomlist[2]),
                     np.cos(joint_pos[0]) * np.sin(joint_pos[1] + joint_pos[2]) * args.discomlist[2]]])
    taug[0][0] = -(args.mass_list[1] * gravty.T * Ja_l2[:, 0] + args.mass_list[
        2] * gravty.T * Ja_l3[:, 0])[0, 0]
    taug[1][0] = -(args.mass_list[1] * gravty.T * Ja_l2[:, 1] + args.mass_list[
        2] * gravty.T * Ja_l3[:, 1])[0, 0]
    taug[2][0] = -(args.mass_list[1] * gravty.T * Ja_l2[:, 2] + args.mass_list[
        2] * gravty.T * Ja_l3[:, 2])[0, 0]
    #taug feedforward of gravity
    tauff = np.dot(C_mat, np.array([joint_vel]).T) + taug
    return tauff

def get_feedbackward(kp,kd,current_pos,current_vel,target_pos,target_vel):
    # TARGET:function to get get_feedbackward force(type: np.ndarry(3*1))
    # MEANING:PD control of pos and vel of swing legs
    # Frame:bofy frame
    # INPUT:current_pos,current_pos,target_pos,target_vel: np.ndarray(3,1)
    ep = (target_pos -current_pos )
    ev = (target_vel - current_vel)
    return np.dot(kp,ep) + np.dot(kd,ev)


