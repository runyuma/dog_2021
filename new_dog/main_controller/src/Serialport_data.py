#!/usr/bin/env python3
# coding:utf-8
import serial
import numpy as np
import pandas as pd
import rospy
from std_msgs.msg import Float32MultiArray, Float32, Int32MultiArray

time_idx = -1


def serial_savedata(name1, data1, name2=None, data2=None, form=None):
    dic1 = zip(name1, data1)
    dataF_1 = pd.DataFrame(dic1)
    if form == 'csv':
        dataF_1.to_csv(name1 + '_Framework.csv')
    elif form == 'npy':
        np.save('/home/marunyu/name1_serial' + '_Framework.npy', dataF_1)
        print(dataF_1)

    # if data2 is not None:
    #     dic2 = {name2: data2}
    #     dataF_2 = pd.DataFrame(dic2)
    #     if form == 'csv':
    #         dataF_2.to_csv(name2 + '_Framework.csv')
    #     else:
    #         np.save(name2 + '_Framework.npy', dataF_2)


def hex2dec(string_num):
    return str(int(string_num.upper(), 16))


rospy.init_node("Serialport")
timepub = rospy.Publisher('/timepub', Float32MultiArray, queue_size=10)
time_msg = Float32MultiArray()
time_msg.data = time_idx

try:
    portx = "/dev/ttyACM0"
    bps = 115200

    # 超时设置,None：永远等待操作，0为立即返回请求结果，其他值为等待超时时间(单位为秒）
    timex = None
    ser = serial.Serial(portx, bps, timeout=timex)

    # print("串口详情参数：", ser)
    # 十六进制的读取
    data_idx = -1
    ser_data = [0., 0., 0.]
    data_True = []
    data_name = []
    check = 0
    nsum = 0

    # begin to read the data
    while True:
        serData = ser.read().hex()  # 读一个字节
        # print(serData)
        if data_idx == 0 or data_idx == 2 or data_idx == 4:
            ser_data[int(data_idx / 2)] = int(hex2dec(serData + "00"))
            nsum += int(hex2dec(serData))
        if data_idx == 1 or data_idx == 3 or data_idx == 5:
            ser_data[int(data_idx / 2)] += int(hex2dec(serData))
            nsum += int(hex2dec(serData))
        if data_idx == 6:
            check = int(hex2dec(serData))
            data_idx = -1
            # print("check: ", (nsum + int(hex2dec("7e"))) % 256 == check)
            if (nsum + int(hex2dec("7e"))) % 256 == check:
                # ser_data.append(time_idx)
                time_msg.data = [time_idx]
                # time_msg.data = time_idx
                timepub.publish(time_msg)
                data_True.append(ser_data)
                data_name.append(str(time_idx))

                print(str(time_idx), ser_data)
                time_idx += 1
                if len(data_True) % 1000 == 0:
                    serial_savedata(data_name, data_True, form='npy')
                    # print(data_True)
            nsum = 0
            ser_data = [0., 0., 0.]
        if data_idx != -1:
            data_idx += 1
        if serData == "7e":
            data_idx = 0
        # print(serData)
    print("---------------")
    ser.close()  # 关闭串口
except Exception as e:
    print("---异常---：", e)
