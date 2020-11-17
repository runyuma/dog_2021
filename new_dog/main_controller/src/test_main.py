#!/usr/bin/env python3
# coding:utf-8
import rospy
import time
from std_msgs.msg import Float32MultiArray,Float32,Int32MultiArray
test_upperconcle = 0
test_singleleg = 1
def test_upperconcole():
    rospy.init_node("uppernode")
    rate = rospy.Rate(1)
    upperpub = rospy.Publisher('/downstream', Float32MultiArray,queue_size=10)
    msg = Float32MultiArray()
    # msg.data = [0 for i in range(12)] + [0, 1., -2.]*4
    msg.data = [-1, -1, 3]+[-1 for i in range(9)] + [0. for i in range(12)]
    upperpub.publish(msg)
    while not rospy.is_shutdown():
        msg.data = [-1, -1, 0] + [-1 for i in range(9)] +[0,0,.0]+ [0 for i in range(9)]
        upperpub.publish(msg)
        rate.sleep()
def test_singleleg():
    rospy.init_node("test_leg")
    rate = rospy.Rate(1)
    status_publisher = rospy.Publisher('/leg_status', Int32MultiArray,queue_size=10)
    footforce_publisher = rospy.Publisher('/ground_force', Float32MultiArray, queue_size=10)
    swingleg_publisher = rospy.Publisher('/swing_leg', Float32MultiArray, queue_size=10)
    status_msg = Int32MultiArray()
    footforce_msg = Float32MultiArray()
    swingleg_msg = Float32MultiArray()
    status_msg.data = [3,3,3,3]
    footforce_msg.data = [0 for i in range(12)]
    swingleg_msg.data = [0 for i in range(24)]
    status_publisher.publish(status_msg)
    footforce_publisher.publish(footforce_msg)
    swingleg_publisher.publish(swingleg_msg)
    while not rospy.is_shutdown():
        # status_msg.data = [0, 0 ,0 , 0]
        status_msg.data = [5, 5, 5, 5]
        # footforce_msg.data = [0,0,100] * 4
        footforce_msg.data = [0, 0.78, -1.57] * 4
        swingleg_msg.data = [0 for i in range(24)]
        status_publisher.publish(status_msg)
        footforce_publisher.publish(footforce_msg)
        swingleg_publisher.publish(swingleg_msg)
        rate.sleep()

if test_upperconcle:
    test_upperconcole()
elif test_singleleg:
    test_singleleg()