/****************************************************************************\
 * @file    stm32_Communication.cpp
 * @author  CQS
 * @date    2021/02/17
 * @brief   下位机通讯主函数,本节点的主要工作逻辑
 ******************************************************************************
 * @attent  这里的STM32指的是两个电机驱动板
 *          串口未像之前的节点里添加权限！
 *          TODO：目前下发的是理想零点下的数据，接收的是实际数据
 ****************************************************************************/
#include "UnitreeDriver.h"
#include <serial/serial.h>
#include <ros/ros.h>
#include <string>
#include <std_msgs/Float32MultiArray.h>

/* User Config */
#define SENDRATE        1000    // Hz,节点通过串口下发的频率
#define UPQUEUESIZE     10      // UpQueue长度
#define DOWNQUEUESIZE   20      // DownQueue长度
#define LEFTINDEX       0       // 左边下标
#define RIGHTINDEX      1       // 右边下标
// KP KD
#define MOTOR0KP        3.0f
#define MOTOR0KD        5.0f
#define MOTOR1KP        4.0f
#define MOTOR1KD        6.0f
#define MOTOR2KP        7.0f
#define MOTOR2KD        8.0f
// 减速比

// 相对零点

/* User Config */

UnitreeDriver *pMotorDriver[2] = {nullptr, nullptr};   // 0 左驱动板 1 右驱动板
void DownStreamCallback(const std_msgs::Float32MultiArray::ConstPtr& DownStreamMsg);
void PublishMotorData(ros::Publisher& Pub);
void DebugTest();
void NodeUserInit();

static double getCurrentTime(){  
    ros::Time CurrentTime = ros::Time::now();
    return double(CurrentTime.toSec());
} 

int main(int argc, char **argv){
    ros::init(argc, argv, "STM32_Node");    // 创建Node
    ros::NodeHandle nh;                     // 和topic service param等交互的公共接口，是操作节点的凭据
    
    ros::Publisher UpStreamPub = nh.advertise<std_msgs::Float32MultiArray>("/upstream", UPQUEUESIZE);   // 发布12个电机的数据
    ros::Subscriber DownStreamSub = nh.subscribe("/downstream", DOWNQUEUESIZE, DownStreamCallback);     // 订阅
    pMotorDriver[0] = new UnitreeDriver("/dev/ttyUSB0");
    pMotorDriver[1] = new UnitreeDriver("/dev/ttyUSB1");
    NodeUserInit();

    ros::Rate loop_rate(SENDRATE);
    while(ros::ok()){
        pMotorDriver[LEFTINDEX]->UpdateMotorData();             // 刷新电机当前数据
        pMotorDriver[RIGHTINDEX]->UpdateMotorData();
        PublishMotorData(UpStreamPub);                  // 发布电机当前数据
        ros::spinOnce();                                // 刷新控制数据(调用本函数之后，会直接调用CallBack函数，所以应该是不用担心数据还没来得及刷新的问题的)
        pMotorDriver[LEFTINDEX]->SendControlDataToSTM32();      // 下发新的数据到STM32
        pMotorDriver[RIGHTINDEX]->SendControlDataToSTM32();      // 
        loop_rate.sleep();
    }
}  

/** @brief 节点部分数据初始化函数 */
void NodeUserInit(void){
    for(int count = 0;count < 2;count ++){
        pMotorDriver[count]->SetKPKD(0, MOTOR0KP, MOTOR0KD);
        pMotorDriver[count]->SetKPKD(1, MOTOR1KP, MOTOR1KD);
        pMotorDriver[count]->SetKPKD(2, MOTOR2KP, MOTOR2KD);
        pMotorDriver[count]->SetKPKD(3, MOTOR0KP, MOTOR0KD);
        pMotorDriver[count]->SetKPKD(4, MOTOR1KP, MOTOR1KD);
        pMotorDriver[count]->SetKPKD(5, MOTOR2KP, MOTOR2KD);
    }
}

/**
 * @brief 订阅回调函数
 * @param DownStreamMsg：DownStream数据，前12个字节表示运动模式，对应的后12个字节表示数据(位置模式就是位置数据,速度模式就是速度数据)
 *        左前 右前 左后 右后
 *        位置数据会经过处理转变为实际目标位置数据
 */
void DownStreamCallback(const std_msgs::Float32MultiArray::ConstPtr& DownStreamMsg){
    for(uint8_t Count = 0;Count < 2;Count ++){
        /* 运动模式赋值 */
        // 左
        pMotorDriver[LEFTINDEX]->MotorData[0 + Count * 3].MotionMode = MotionMode_t(DownStreamMsg->data.at(0 + Count * 6));
        pMotorDriver[LEFTINDEX]->MotorData[1 + Count * 3].MotionMode = MotionMode_t(DownStreamMsg->data.at(1 + Count * 6));
        pMotorDriver[LEFTINDEX]->MotorData[2 + Count * 3].MotionMode = MotionMode_t(DownStreamMsg->data.at(2 + Count * 6));
        // 右
        pMotorDriver[RIGHTINDEX]->MotorData[0 + Count * 3].MotionMode = MotionMode_t(DownStreamMsg->data.at(3 + Count * 6));
        pMotorDriver[RIGHTINDEX]->MotorData[1 + Count * 3].MotionMode = MotionMode_t(DownStreamMsg->data.at(4 + Count * 6));
        pMotorDriver[RIGHTINDEX]->MotorData[2 + Count * 3].MotionMode = MotionMode_t(DownStreamMsg->data.at(5 + Count * 6));
    }
    // 更新控制数据
    for(uint8_t Count = 0;Count < 2;Count ++){// 前 后
        for(uint8_t MotorCount = 0;MotorCount < 3;MotorCount ++){   // 0 1 2
            UnitreeMotorData_t* pMotorData = nullptr;
            // 左
            pMotorData = &(pMotorDriver[LEFTINDEX]->MotorData[MotorCount + Count * 3]);
            switch(pMotorData->MotionMode)
            {
                case DISABLE:break;
                case TORMODE:pMotorData->TarTor = DownStreamMsg->data.at(12 + MotorCount + Count * 6);break;
                case VELMODE:pMotorData->TarVel = DownStreamMsg->data.at(12 + MotorCount + Count * 6);break;
                case POSMODE:pMotorData->TarPos = DownStreamMsg->data.at(12 + MotorCount + Count * 6);break;
                default:break;
            }
            // 右
            pMotorData = &(pMotorDriver[RIGHTINDEX]->MotorData[MotorCount + Count * 3]);
            switch(pMotorData->MotionMode)
            {
                case DISABLE:break;
                case TORMODE:pMotorData->TarTor = DownStreamMsg->data.at(15 + MotorCount + Count * 6);break;
                case VELMODE:pMotorData->TarVel = DownStreamMsg->data.at(15 + MotorCount + Count * 6);break;
                case POSMODE:pMotorData->TarPos = DownStreamMsg->data.at(15 + MotorCount + Count * 6);break;
                default:break;
            }
        }
    }
    for(int Count = 0;Count < 12;Count ++)
    {
     int sidesign = Count/6;
     int motor_count = Count%6;
     UnitreeMotorData_t* pMotorData = nullptr;
     pMotorData = &(pMotorDriver[sidesign]->MotorData[motor_count]);
     switch(pMotorData->MotionMode)
     {
        case DISABLE:std::cout<<"motor"<<Count<<"disable"<<std::endl;break;
        case TORMODE:std::cout<<"motor"<<Count<<"TarTor: "<<pMotorData->TarTor<<std::endl;break;
        case VELMODE:std::cout<<"motor"<<Count<<"TarVel: "<<pMotorData->TarVel<<std::endl;break;
        case POSMODE:std::cout<<"motor"<<Count<<"TarPos: "<<pMotorData->TarPos<<std::endl;break;
        default:break;
     }

    }
} 

/**
 * @brief 发布电机数据
 * @param Pub：发布者的引用
 */
void PublishMotorData(ros::Publisher& Pub){
    std_msgs::Float32MultiArray MotorDataArray;
    MotorDataArray.data.clear();
    MotorDataArray.data.resize(24); // 12位置 + 12速度
    /* 向量构建 */
    for(int count = 0;count < 3;count ++){
        // 左012 右012 左3(0)4(1)5(2) 右3(0)4(1)5(2)
        MotorDataArray.data[0 + count] = pMotorDriver[LEFTINDEX]->MotorData[count].CurPos;
        MotorDataArray.data[3 + count] = pMotorDriver[RIGHTINDEX]->MotorData[count].CurPos;
        MotorDataArray.data[6 + count] = pMotorDriver[LEFTINDEX]->MotorData[count + 3].CurPos;
        MotorDataArray.data[9 + count] = pMotorDriver[RIGHTINDEX]->MotorData[count + 3].CurPos;

        MotorDataArray.data[12 + count] = pMotorDriver[LEFTINDEX]->MotorData[count].CurVel;
        MotorDataArray.data[15 + count] = pMotorDriver[RIGHTINDEX]->MotorData[count].CurVel;
        MotorDataArray.data[18 + count] = pMotorDriver[LEFTINDEX]->MotorData[count + 3].CurVel;
        MotorDataArray.data[21 + count] = pMotorDriver[RIGHTINDEX]->MotorData[count + 3].CurVel;
    }
    Pub.publish(MotorDataArray);
}

/** @brief 调试用的一个函数 */
void DebugTest(void){
    
}
