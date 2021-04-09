#!/usr/bin/env python3
# coding:utf-8

"""
CRC8计算函数
参数1：待校验的字节串
参数2：字节串长度
输出：CRC计算结果
将需要校验的字符串扔进来，如果最终返回结果为0则认为校验通过
"""
def CRC8Calculate(CheckBytesArray, ArrayLength):
    CRC = 0xAA
    Polynomial = 0x7D
    for i in range(ArrayLength):
        CRC = CRC ^ (CheckBytesArray[i])
        for j in range(8):
            if CRC & 0x01:
                CRC = (CRC >> 1) ^ Polynomial
            else:
                CRC >>= 1
    return CRC


"""
IMU数据解码函数
参数1：经过校验的完整的一帧数据
参数2：存放解码结果的List
"""
def IMUDataDecode(DataBytesArray, ResultList):
    import struct
    # 解析角度值
    for i in range(3):
        start = 2 + i * 4
        end = 2 + i * 4 + 4
        ResultList[i] = struct.unpack('<f', DataBytesArray[start:end])[0]
    # 解析角速度值
    for i in range(3):
        start = 14 + i * 4
        end = 14 + i * 4 + 4
        ResultList[i + 3] = struct.unpack('<f', DataBytesArray[start:end])[0]
    # 解析加速度值
    for i in range(3):
        start = 26 + i * 4
        end = 26 + i * 4 + 4
        ResultList[i + 6] = struct.unpack('<f', DataBytesArray[start:end])[0]

# Demo
if __name__ == '__main__':
    import struct
    Result = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    RawData = bytes([0x14, 0x23, 0x7E ,0x01 ,0xB6 ,0xF3 ,0x9D ,0x3F ,0x2D ,0xB2 ,0xB5 ,0x40 ,0x27 ,0x31 ,0x10 ,0x41 ,0xB6 ,0xF3 ,0x9D ,0x3F ,0x2D,0xB2 ,0xB5 ,0x40 ,0x27 ,0x31 ,0x10 ,0x41 ,0xB6 ,0xF3 ,0x9D ,0x3F ,0x2D ,0xB2 ,0xB5 ,0x40 ,0x27 ,0x31 ,0x10 ,0x41 ,0x32, 0x05])   # 接收缓冲区
    # RawData = b'\x99\x02(~\x81r\x1c\x97=\xf12&\x02]~\xa1\x8e\xe3x=\xc8\xfb2\x02\x7f~\x01UU\x95\xbd_\xb2\xab\x02n~!UU\x15\xbd,\xc7\xff\x02y~A9\x8e\xe3\xba\x8f\xad\x01\x02)~ar\x1c\x87\xbcJ\xb8\x99\x02(~\x81r\x1c\x97=\xf12&\x02]~\xa1\x8e\xe3x=\xc8\xfb2\x02\x7f~\x01UU\x95\xbd_\xb2\xab\x02n~!UU\x15\xbd,\xc7\xff\x02y~A9\x8e\xe3\xba\x8f\xad\x01\x02)~ar\x1c\x87\xbc'

    FrameHeadIndex = RawData.find(0x7E)
    FrameHeadIndex = None
    for i in range(len(RawData)-1):
        if RawData[i] == 0x7E:
            if RawData[i+1] == 0x01:
                FrameHeadIndex = i
                break

    if FrameHeadIndex != None:
        PossibleFrame = RawData[FrameHeadIndex:FrameHeadIndex + 39] # 完整的一帧长度为39个字节
        if CRC8Calculate(PossibleFrame, 39) == 0:
            IMUDataDecode(PossibleFrame, Result)
        RawData = RawData[FrameHeadIndex + 39:] # "出队"

