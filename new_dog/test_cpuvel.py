# -*-coding:utf-8-*-
import time
import numpy as np
start_time = time.time()
a = []
use_for = 1
epoch = 1000000
mat = np.diag([1,1,1])
if use_for:
    for i in range(epoch):
        a.append(np.dot(mat,np.ones((3,1))))
else:
    a = [np.dot(mat,np.ones((3,1))) for i in range(epoch)]

print(time.time() - start_time)
