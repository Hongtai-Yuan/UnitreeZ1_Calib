#ifndef _UNITREE_ARM_ARM_COMMON_H_
#define _UNITREE_ARM_ARM_COMMON_H_

#include <stdint.h>

#pragma pack(1)

namespace UNITREE_ARM {
// 4 Byte
enum class ArmFSMState{
    INVALID,
    PASSIVE,//所有电机进入阻尼状态
    JOINTCTRL,//进入关节控制状态机，长按键盘控制六个关节和夹爪的速度，控制其运动
    CARTESIAN,//给定机械臂末端的期望笛卡尔坐标和速度，机械臂会自动规划路径并运动
    MOVEJ,//末端点到点运行，路径不固定
    MOVEL,//末端点到点运行，沿直线路径
    MOVEC,//给定中间点和终点，末端沿圆弧运动
    TRAJECTORY,
    TOSTATE,//以movej的方式将机械臂末端移动到目标位置
    SAVESTATE,//保存当前机械臂末端位置
    TEACH,//示教模式
    TEACHREPEAT,//将示教的轨迹保存进z1_controller/config目录下的一个新.csv文件中
    CALIBRATION,//设置机械臂当前位置为初始位置，并自动转为阻尼状态
    SETTRAJ,
    BACKTOSTART,//回到起始点
    NEXT,
    LOWCMD
};

enum class TrajType{
    MoveJ,
    MoveL,
    MoveC,
    Stop
};

// 20 Byte
struct JointCmd{
    float T;
    float W;
    float Pos;
    float K_P;
    float K_W;
};

typedef struct{
    uint8_t reserved : 6 ;
    uint8_t state    : 2 ;//whether motor is connected; 0-ok, 1-disconnected, 2-CRC error
}Motor_Connected;

typedef struct{
    int8_t temperature;
    /* 0x01: phase current is too large
     * 0x02: phase leakage
     * 0x04: overheat(including the motor windings and the motor shell)
     * 0x20: jumped
     * 0x40: nothing
     */
    uint8_t error;
    Motor_Connected isConnected;
}Motor_State;

struct JointState{
    float T;
    float W;
    float Acc;
    float Pos;
    Motor_State state[2];
};

struct Posture{
    double rx;
    double ry;
    double rz;
    double x;
    double y;
    double z;
};

struct TrajCmd{
    TrajType trajType;
    Posture posture[2];
    double gripperPos;
    double maxSpeed;
    double stopTime;
    int trajOrder;
};

union ValueUnion{
    char name[10];
    JointCmd jointCmd[7];
    TrajCmd trajCmd;
};

struct SendCmd{
    uint8_t head[2];
    ArmFSMState state;
    bool track;// whether let arm track jointCmd in State_JOINTCTRL or posture[0] in State_CARTESIAN
    ValueUnion valueUnion;
};


struct RecvState{
    uint8_t head[2];
    ArmFSMState state;
    JointState jointState[7];
    Posture cartesianState;
};

constexpr int SENDCMD_LENGTH    = (sizeof(SendCmd));
constexpr int RECVSTATE_LENGTH  = (sizeof(RecvState));
constexpr int JointCmd_LENGTH   = (sizeof(JointCmd));
constexpr int JointState_LENGTH = (sizeof(JointState));

#pragma pack()
}
#endif  // _UNITREE_ARM_ARM_MSG_H_