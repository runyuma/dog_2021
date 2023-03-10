#!/usr/bin/env python3
# coding:utf-8

import numpy
import matplotlib.pyplot as plt
import pandas as pd

path = "/home/marunyu/catkin_ws/src//dog_2021//new_dog/state_estimation/datas/"+"Mon May 31 22:56:51 2021.csv"
df=pd.read_csv(open(path))
print(df)

df.plot(x = "time",y = "vx",)
df.plot(x = "time",y = "vy",)
df.plot(x = "time",y = "vz",)
df.plot(x = "time",y = "x",)
df.plot(x = "time",y = "y",)
df.plot(x = "time",y = "z",)
r = df.plot(x = "time",y = "r")
p = df.plot(x = "time",y = "p",ax = r)
df.plot(x = "time",y = "y",ax = p)
wx = df.plot(x = "time",y = "wx")
wy = df.plot(x = "time",y = "wy",ax = wx)
df.plot(x = "time",y = "wz",ax = wy)
plt.show()