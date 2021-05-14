#!/usr/bin/env python3
# coding:utf-8

import numpy
import matplotlib.pyplot as plt
import pandas as pd

path = "/home/marunyu/catkin_ws/src/new_dog/state_estimation/datas/"+"Fri May 14 17:07:48 2021SIM.csv"
df=pd.read_csv(open(path))

a = df.plot(x = "time",y = "gazebo_vx")
a1 = df.plot(x = "time",y = "raw_vx", ax =a)
df.plot(x = "time",y = "vx", ax =a1 )
b = df.plot(x = "time",y = "gazebo_vz")
c = df.plot(x = "time",y = "raw_vz",ax =b)
df.plot(x = "time",y = "vz", ax =c)
c = df.plot(x = "time",y = "gazebo_x")
df.plot(x = "time",y = "x", ax =c)
d = df.plot(x = "time",y = "gazebo_z")
df.plot(x = "time",y = "z", ax = d)
plt.show()