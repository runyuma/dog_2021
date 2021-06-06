/****************************************************************************\
 * @file    stm32_Communication.cpp
 * @author  CQS
 * @date    2021/02/17
 * @brief   下位机通讯主函数,本节点的主要工作逻辑
 ******************************************************************************
 * @attent  这里的STM32指的是两个电机驱动板
 *          串口未像之前的节点里添加权限！
 *          TODO：目前下发的是理想零点下的数据，接收的是实际数据
 *          2.23 加入了电机正反方向数组的设定
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
#define FRONTINDEX      0       // 前驱动板下标
#define BACKINDEX       1       // 后驱动板下标
// KP KD
#define MOTOR0KP        0.05f    // 0号KP
#define MOTOR0KD        12.0f
#define MOTOR1KP        0.05f    // 1号KP
#define MOTOR1KD        5.0f
#define MOTOR2KP        0.1f    // 2号KP
#define MOTOR2KD        5.0f
// 逻辑零点实际位置数组：下发命令的时候，加上本数组；接收的时候，减掉本数组；第一排是前面的电机，第二排是后面的电机
#define LOGIZZEROPOSARRAY   {{0.f, -0.f, 0.f, 0.f, 0.f, 0.f},  \
                            {0.f,0.f, 0.f, 0.f, 0.f, 0.f}}
// 正反 + 减速比数组：下发命令时乘本数组，接收的时候除以本数组
#define MOTORDIRARRAY       {{1, 1, 1, 1, 1, 1},    \
                            {1, 1, 1, 1, 1, 1}}
/* User Config */

static UnitreeDriver *pMotorDriver[2] = {nullptr, nullptr};     // 0 前驱动板 1 后驱动板
static const float LogicZeroPosArray[2][6] = LOGIZZEROPOSARRAY; // 逻辑零位实际位置数组
static const float MotorRatioArray[2][6] = MOTORDIRARRAY;         // 电机正反减速比数组
void DownStreamCallback(const std_msgs::Float32MultiArray::ConstPtr& DownStreamMsg);
void Map_PublishMotorData(ros::Publisher& Pub);
void DebugTest();
void NodeUserInit();
void FrontLowerTimercallback(const ros::TimerEvent&);
void BackLowerTimercallback(const ros::TimerEvent&);

/** @brief 获取当前时间 */
static double getCurrentTime(){
    ros::Time CurrentTime = ros::Time::now();
    return double(CurrentTime.toSec());
}

/** @brief 主函数 */
int main(int argc, char **argv){
    ros::init(argc, argv, "STM32_Node");    // 创建Node
    ros::NodeHandle nh;                     // 和topic service param等交互的公共接口，是操作节点的凭据
    ros::Timer FrontLowerTimer = nh.createTimer(ros::Duration(0.1), FrontLowerTimercallback);
    ros::Timer BackLowerTimer = nh.createTimer(ros::Duration(0.1), BackLowerTimercallback);

    ros::Publisher UpStreamPub = nh.advertise<std_msgs::Float32MultiArray>("/upstream", UPQUEUESIZE);   // 发布12个电机的数据
    ros::Subscriber DownStreamSub = nh.subscribe("/downstream", DOWNQUEUESIZE, DownStreamCallback);     // 订阅
    pMotorDriver[FRONTINDEX] = new UnitreeDriver("/dev/front");
    pMotorDriver[BACKINDEX] = new UnitreeDriver("/dev/back");
    NodeUserInit(); // 参数初始化
    ros::Rate loop_rate(SENDRATE);
    FrontLowerTimer.start();
    BackLowerTimer.start();
    while(ros::ok()){
        if(pMotorDriver[FRONTINDEX]->UpdateMotorData()){
            FrontLowerTimer.stop();
            FrontLowerTimer.start();
        }
        if(pMotorDriver[BACKINDEX]->UpdateMotorData()){
            BackLowerTimer.stop();
            BackLowerTimer.start();
        }

#if 0
    // 显示下位机上发的数据
    static int UpdateCount = 0;
    if(++UpdateCount == 200){
        UpdateCount = 0;
        std::cout << "MotorData front :";
        for(int i = 0;i < 6;i ++){
            std::cout << pMotorDriver[FRONTINDEX]->MotorData[i].CurPos << " ";
        }
        std::cout << std::endl;
        std::cout << "MotorData back :";
        for(int i = 0;i < 6;i ++){
            std::cout << pMotorDriver[BACKINDEX]->MotorData[i].CurPos << " ";
        }
        std::cout << std::endl;
    }
    
#endif

        Map_PublishMotorData(UpStreamPub);                      // 发布电机当前数据
        ros::spinOnce();                                        // 刷新控制数据(调用本函数之后，会直接调用CallBack函数，所以应该是不用担心数据还没来得及刷新的问题的)
        // DebugTest();
        pMotorDriver[FRONTINDEX]->SendControlDataToSTM32();     // 下发新的数据到STM32
        pMotorDriver[BACKINDEX]->SendControlDataToSTM32();      // 
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
 *        前左 前右 后左 后右
 */
void DownStreamCallback(const std_msgs::Float32MultiArray::ConstPtr& DownStreamMsg){
    static int CoutCount = 0;
    for(uint8_t Count = 0;Count < 2;Count ++){  // 前驱动板 后驱动板
        for(uint8_t MotorCount = 0;MotorCount < 6;MotorCount ++){   // 先左后右
            UnitreeMotorData_t* pMotorData = &(pMotorDriver[Count]->MotorData[MotorCount]);
            pMotorData->MotionMode = MotionMode_t(DownStreamMsg->data.at(MotorCount + Count * 6));
            switch(pMotorData->MotionMode){
                case DISABLE:break;
                case TORMODE:{
                    float LogTarTor = DownStreamMsg->data.at(12 + MotorCount + Count * 6);
                    pMotorData->TarTor = LogTarTor / MotorRatioArray[Count][MotorCount];
                }break;
                case VELMODE:{
                    float LogTarVel = DownStreamMsg->data.at(12 + MotorCount + Count * 6);
                    pMotorData->TarVel = LogTarVel * MotorRatioArray[Count][MotorCount];
                }break;
                case POSMODE:{
                    float LogTarPos = DownStreamMsg->data.at(12 + MotorCount + Count * 6);
                    pMotorData->TarPos = LogTarPos * MotorRatioArray[Count][MotorCount] + LogicZeroPosArray[Count][MotorCount];
                }break;
            }
        }
    }
    // if(++CoutCount == 1000){
    //     CoutCount = 0;
    //     std::cout << "STM32 will receive :";
    //     for(int Count = 0;Count < 12;Count){
    //         UnitreeMotorData_t* pMotorData = &(pMotorDriver[Count / 6]->MotorData[Count % 6]);
    //         std::cout << "Index: " << Count / 6 << "MotorIndex: " << Count % 6;
    //         switch(pMotorData->MotionMode){
    //             case DISABLE:break;
    //             case TORMODE:{
    //                 std::cout << "Tor" << pMotorData->TarTor << std::endl;
    //             }break;
    //             case VELMODE:{
    //                 std::cout << "Vel" << pMotorData->TarVel << std::endl;
    //             }break;
    //             case POSMODE:{
    //                 std::cout << "Pos" << pMotorData->TarPos << std::endl;
    //             }break;
    //         }
    //     }
    // }
}

/**
 * @brief 依据原始数据做映射，并且发布电机数据
 * @param Pub：发布者的引用
 */
void Map_PublishMotorData(ros::Publisher& Pub){
    std_msgs::Float32MultiArray MotorDataArray;
    MotorDataArray.data.clear();
    MotorDataArray.data.resize(24); // 12位置 + 12速度
    for(uint8_t Count = 0;Count < 12;Count ++){ // 前左右 后左右
        uint8_t Index = Count / 6;              // 前还是后
        uint8_t MotorIndex = Count % 6;         // 前or后的哪个电机
        /* 实际->逻辑 */
        MotorDataArray.data[Count] = (pMotorDriver[Index]->MotorData[MotorIndex].CurPos - LogicZeroPosArray[Index][MotorIndex]) / MotorRatioArray[Index][MotorIndex];
        MotorDataArray.data[12 + Count] = pMotorDriver[Index]->MotorData[MotorIndex].CurVel / MotorRatioArray[Index][MotorIndex];
    }
    Pub.publish(MotorDataArray);  // 发布

}

/** @brief 调试用的一个函数 */
void DebugTest(void){
    
}

/** @brief 前下位机看门狗回调函数 */
void FrontLowerTimercallback(const ros::TimerEvent&){
    ROS_ERROR_STREAM("Front Lower Disconnected!");
    // 后续操作
}

/** @brief 后下位机看门狗回调函数 */
void BackLowerTimercallback(const ros::TimerEvent&){
    ROS_ERROR_STREAM("Back Lower Disconnected!");
    // 后续操作
}

#if 0
    // 显示下位机上发的数据
    static int UpdateCount = 0;
    if(++UpdateCount == 200){
        UpdateCount = 0;
        std::cout << "MotorData:";
        for(int i = 0;i < 6;i ++){
            std::cout << pMotorDriver[FRONTINDEX]->MotorData[i].CurPos << " ";
        }
        std::cout << std::endl;
    }
#endif

