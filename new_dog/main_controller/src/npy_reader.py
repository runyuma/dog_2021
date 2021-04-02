#!/usr/bin/env python3
# coding:utf-8
import csv
import numpy as np
import pandas as pd


def str2int(strA):
    trans = []
    negative = 0
    for i in strA:
        if i == '-': negative = 1
        if i == '.': break
        if '0' <= i <= '9':
            trans.append(i)

    intA = 0
    times = len(trans)
    for i in trans:
        intA += (int(i) - 0) * 10 ** (times - 1)
        times -= 1
    if negative == 1:
        intA = -intA
        negative == 0
    return intA


def read_npy(read_num):
    # read_num = 1, or 2

    if read_num == 1:
        reader1 = np.load("/home/marunyu/name1_Framework.npy", allow_pickle=True)
        name_1 = []
        for row in reader1:
            name_1.append(str2int(row[0]))
    elif read_num == 2:
        reader2 = np.load("/home/marunyu/name1_serial_Framework.npy", allow_pickle=True)
        name_2 = []
        for row in reader2:
            name_2.append(str2int(row[0]))

    if read_num == 1:
        # print("111111111111111")
        datalist = []
        idx = -1
        for row in reader1:
            print(str2int(row[0]))
            if str2int(row[0]) > idx:
                datalist.append(row[1][0:3])
                print(row[0], ": ", row[1][0:3])
            idx = str2int(row[0])

    elif read_num == 2:
        datalist = []
        idx = -1
        for row in reader2:
            if str2int(row[0]) > idx:
                datalist.append(row[1][0:3])
                print(row[0], ": ", row[1][0:3])
            idx = str2int(row[0])

    return datalist


if __name__ == '__main__':
    read_npy(2)
    # print(len(read_npy(2)))
