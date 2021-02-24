/****************************************************************************\
 * @file    UnitreeDriver.h
 * @author  CQS
 * @date    2021/02/17
 * @brief   驱动板通讯类
 ******************************************************************************
 * @attent  类内部自带一个serial对象,在实例化本对象的时候需要将串口名字输入
 ****************************************************************************/
#ifndef UNITREEDRIVER_H
#define UNITREEDRIVER_H

#include <serial/serial.h>
#include <string.h>

typedef enum{
    DISABLE = -1,    // 未启动
    POSMODE = 0,
    VELMODE = 1,
    TORMODE = 2
}MotionMode_t;

typedef struct{
    MotionMode_t MotionMode = DISABLE;  // 运动模式
    float TarTor;
    float TarPos;
    float TarVel;
    float KP;
    float KD;
    float CurVel;
    float CurPos;
}UnitreeMotorData_t;    // 存储控制信息和反馈信息的结构体

class UnitreeDriver
{
    public:
        UnitreeDriver(const std::string PortName = "Null");
        ~UnitreeDriver();
        void SendControlDataToSTM32(void);
        void UpdateMotorData(void);
        void SetKPKD(uint8_t MotorID, float KP, float KD);
        UnitreeMotorData_t MotorData[6];    // 0 1 2 对应 前0 1 2；3 4 5对应后0 1 2；
    private:
        serial::Serial prvSerial;
        uint8_t prvCRCCalculate(uint8_t *pStr, uint8_t Len);
        /* Tx */
        void EncodeAbleFrame(uint8_t *pData, uint8_t MotorID, bool Able);
        void EncodeParaFrame(uint8_t *pData, uint8_t MotorID, float KP, float KD);
        void EncodeTorFrame(uint8_t *pData, uint8_t MotorID, float Torque);
        void EncodeVelFrame(uint8_t *pData, uint8_t MotorID, float Velocity);
        void EncodePosFrame(uint8_t *pData, uint8_t MotorID, float Position);
        /* Rx */
        std::string RxBuffer = {};          // 接收缓冲区，默认为空的
        void DecodeFrame(uint8_t *pData);
};

#endif