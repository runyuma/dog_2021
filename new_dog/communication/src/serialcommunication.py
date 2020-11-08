#!/usr/bin/env python3
# coding:utf-8
import serial #导入模块
import serial.tools.list_ports
import os
import time
import binascii
command = "sudo chmod 777 " + "/dev/ttyUSB0"
sudoPassword = "456456456rr"
os.system('echo %s|sudo -S %s' % (sudoPassword, command))
# give chmod right

class serialCommunication():
    def __init__(self):
        command = "sudo chmod 777 " + "/dev/ttyUSB0"
        sudoPassword = "456456456rr"
        os.system('echo %s|sudo -S %s' % (sudoPassword, command))
        # give the permits to /dev/usb
        port_list = list(serial.tools.list_ports.comports())
        if len(port_list) == 0:
            print('无可用串口')
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


    def receive(self,pos_list,vel_list):
        print("aloop")
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
                        index = _information[0]
                        sums = sum(_information[0:5])+self.begin_mark[0]
                        check = _information[5]
                        if sums % 256 == _information[5]:
                            self.msg[index] = b'~'+_information
                            pos = (int.from_bytes(self.msg[index][2:4], byteorder='big', signed=False))/100
                            vel = (int.from_bytes(self.msg[index][4:6], byteorder='big', signed=False))/100
                            pos_list[index] = pos
                            vel_list[index] = vel
                            print('set_mes1')
                        else:
                            print('analysis_error')
                if len(information[-1]) < 6:
                    self.recmsgQueue = b'~' + information[-1]
                elif len(information[-1]) == 6:
                    _information = information[-1]
                    self.recmsgQueue = b''
                    index = _information[0]
                    sums = sum(_information[0:5])+self.begin_mark[0]
                    if sums % 256 == _information[5]:
                        self.msg[index] = b'~' + _information
                        pos = (int.from_bytes(self.msg[index][2:4], byteorder='big', signed=False)) / 100
                        vel = (int.from_bytes(self.msg[index][4:6], byteorder='big', signed=False)) / 100
                        pos_list[index] = pos
                        vel_list[index] = vel
                        print('set_mes2')
                    else:
                        print('analysis_error')
                    self.recmsgQueue = b''

    def send(self,target_value,target_mode):
        kp = 30
        #TODO:new rules
        for i in range(12):
            if target_mode[i] == 3:
                pass #TODO：
            elif target_mode[i] == 4:
                pass #TODO：
            else:
                if target_value[i] != float('nan'):
                    self.message_send = b'~'
                    self.message_send += (16 * i + target_mode[i]).to_bytes(length=1,byteorder='big',signed=False) + int(target_value[i]*100).to_bytes(length=2,byteorder='big',signed=False) +int(kp*100).to_bytes(length=2,byteorder='big',signed=False)
                    sums = sum(self.message_send)%256
                    self.message_send += sums.to_bytes(length=1,byteorder='big',signed=False)
                    # print(self.message_send)
                    self.ser.write(self.message_send)











