#!/usr/bin/env python3
# coding:utf-8
import serial #导入模块
import serial.tools.list_ports
import os
import time
import binascii
import copy
import numpy as np
import struct

command = "sudo chmod 777 " + "/dev/ttyUSB0"
sudoPassword = "456456456rr"
os.system('echo %s|sudo -S %s' % (sudoPassword, command))
# give chmod right

def floatToBytes(f):
    bs = struct.pack("f",f)
    res = bytearray()
    for i in range(4):
        res.append(bs[3-i])
    return(b''+res)

class serialCommunication():
    def __init__(self):
        command = "sudo chmod 777 " + "/dev/ttyUSB0"
        sudoPassword = "456456456rr"
        os.system('echo %s|sudo -S %s' % (sudoPassword, command))
        # give the permits to /dev/usb
        self.use_virtual_serial = 0
        port_list = list(serial.tools.list_ports.comports())
        self.time_index = 0
        if len(port_list) == 0:
            print('无可用串口')
            self.use_virtual_serial = 1

        else:
            print(port_list[0])
            self.port = port_list[0]
            self.port_name = "/dev/"+port_list[0].name
            self.bps = 921600
            timex = None
            self.ser = serial.Serial(self.port_name, self.bps, timeout=timex)
            self.recmsgQueue = b''
            self.msg = [b'' for i in range(12)]
            self.begin_mark = b'~'
        self.message_count = [0 for i in range(12)]

    def receive(self,pos_list,vel_list):
        if self.use_virtual_serial:
            _pos_list = [0.,0.78,-1.57] * 4
            _vel_list = [0,0,0] * 4
            for i in range(12):
                pos_list[i] = _pos_list[i]
                vel_list[i] = _vel_list[i]
                pass

        else:
            count = self.ser.inWaiting()
            if count > 0:
                rec_str = self.ser.read(count)
                # print(rec_str)
                self.recmsgQueue = self.recmsgQueue + rec_str
                # print(self.recmsgQueue)


                if len(self.recmsgQueue)>= 7:

                    information  = self.recmsgQueue.split(b'~')
                    # print(information)
                    for _information in information[:-1]:
                        if len(_information)==6:
                            index = _information[0]-1
                            sums = sum(_information[0:5])+self.begin_mark[0]
                            check = _information[5]
                            if sums % 256 == _information[5]:
                                self.msg[index] = b'~'+_information
                                pos = (int.from_bytes(self.msg[index][2:4], byteorder='big', signed=True))/1000
                                vel = (int.from_bytes(self.msg[index][4:6], byteorder='big', signed=True))/1000
                                pos_list[index] = pos
                                vel_list[index] = vel
                                # print('set_mes1')
                            else:
                                print('analysis_error')
                    if len(information[-1]) < 6:
                        self.recmsgQueue = b'~' + information[-1]
                    elif len(information[-1]) == 6:
                        _information = information[-1]
                        self.recmsgQueue = b''
                        index = _information[0]-1
                        sums = sum(_information[0:5])+self.begin_mark[0]
                        if sums % 256 == _information[5]:
                            self.msg[index] = b'~' + _information
                            pos = (int.from_bytes(self.msg[index][2:4], byteorder='big', signed=True)) / 1000
                            vel = (int.from_bytes(self.msg[index][4:6], byteorder='big', signed=True)) / 1000
                            pos_list[index] = pos
                            vel_list[index] = vel
                            # print('set_mes2')
                        else:
                            print('analysis_error')
                        self.recmsgQueue = b''

    def send(self,target_value,target_mode):
        kp = 30
        #TODO:new rules
        for i in range(12):
            if target_mode[i] == 3:
                self.message_send = b'~'
                self.message_send += (16 * (i + 1) + 3).to_bytes(length=1, byteorder='big',signed=False) + int(1).to_bytes(length=4, byteorder='big',signed=False)
                sums = sum(self.message_send) % 256
                self.message_send += sums.to_bytes(length=1, byteorder='big', signed=False)
                if self.use_virtual_serial:
                    if self.time_index % 2000 == 0:
                        print("sending:",self.message_send)
                        print("target_mode:", target_mode, 'enabling')
                else:
                    self.ser.write(self.message_send)
                    if self.time_index % 2000 == 0:
                        print("sending", self.message_send)
                        print("target_mode:",i,"   ", target_mode[i], 'enabling')
            elif target_mode[i] == 4:
                self.message_send = b'~'
                self.message_send += (16 * (i + 1) + 3).to_bytes(length=1, byteorder='big',signed=False) + int(0).to_bytes(length=4, byteorder='big',signed=False)
                sums = sum(self.message_send) % 256
                self.message_send += sums.to_bytes(length=1, byteorder='big', signed=False)
                if self.use_virtual_serial:
                    if self.time_index % 2000 == 0:
                        print("sending",self.message_send)
                        print("target_mode: ",i,"   ", target_mode[i], 'disabling')
                else:
                    self.ser.write(self.message_send)
                    if self.time_index % 2000 == 0:
                        print("sending", self.message_send)
                        print("target_mode:",i,"   ", target_mode[i], 'disabling')
            elif target_mode[i] == -1:
                pass #TODO：
            else:
                if not np.isnan(target_value[i]):
                    self.message_send = b'~'
                    self.message_send += (16 * (i+1) + target_mode[i]).to_bytes(length=1,byteorder='big',signed=False) + floatToBytes(target_value[i])
                    sums = sum(self.message_send)%256
                    self.message_send += sums.to_bytes(length=1,byteorder='big',signed=False)
                    # print(self.message_send)
                    if self.use_virtual_serial:
                        if self.time_index%2000 == 0:
                            print("sending:",self.message_send)
                            print("target_mode:",i,"   ",target_mode[i],"target_value",target_value[i])
                    else:
                        self.ser.write(self.message_send)
                        if self.time_index % 2000 == 0:
                            print("sending", self.message_send)
                            print("target_mode:",i,"   ",target_mode[i],"target_value",target_value[i])
                    self.message_count[i] += 1
                time.sleep(0.00002)
        # if self.time_index % 2000 == 0:
        #     print("massge count :   ", self.message_count)
        self.time_index += 1

    def set_PD(self,_motorid,kp,kd):
        self.message_send = b'~'
        self.message_send += (16 * (_motorid + 1) + 4).to_bytes(length=1, byteorder='big', signed=False) + int(kp*100).to_bytes(length=2, byteorder='big', signed=False)+ int(kd*100).to_bytes(length=2, byteorder='big', signed=False)
        sums = sum(self.message_send) % 256
        self.message_send += sums.to_bytes(length=1, byteorder='big', signed=False)
        if self.use_virtual_serial:
            print("sending:", self.message_send)
            print("target_pd",_motorid, kp,kd)
        else:
            print(self.message_send)
            self.ser.write(self.message_send)

#
# a = floatToBytes(1.)
# print(a)










