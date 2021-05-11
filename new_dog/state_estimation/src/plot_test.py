#!/usr/bin/env python3
# coding:utf-8

import numpy
import matplotlib.pyplot as plt
import pandas as pd

path = "/home/marunyu/catkin_ws/src/new_dog/state_estimation/datas/"+"Tue May 11 21:50:17 2021.csv"
df=pd.read_csv(open(path))

a = df.plot(x = "time",y = "gazebo_vx")
df.plot(x = "time",y = "vx", ax =a )
b = df.plot(x = "time",y = "gazebo_vz")
df.plot(x = "time",y = "vz", ax =b)
c = df.plot(x = "time",y = "gazebo_x")
df.plot(x = "time",y = "x", ax =c)
d = df.plot(x = "time",y = "gazebo_z")
df.plot(x = "time",y = "z", ax = d)
plt.show()