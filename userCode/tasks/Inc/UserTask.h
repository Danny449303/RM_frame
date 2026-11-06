//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_USERTASK_H
#define RM_FRAME_C_USERTASK_H
#include "main.h"
#include "Device.h"
#include "Servo.h"
#include "Motor.h"

typedef enum {
    ROBOT_MODE_DRIVE = 0,   // 第三个按钮上档：驾驶
    ROBOT_MODE_ARM,         // 中档：锁轮 + 机械臂
    ROBOT_MODE_ARM_OPEN     // 下档：锁轮 + 机械臂 + 夹爪打开
} ROBOT_MODE_E;

extern volatile ROBOT_MODE_E robotMode;
extern volatile uint8_t robotEmergency;

/*外部函数声明-------------------------------------------------------------*/

void UserStop();
void UserHandle();
void UserInit();
void UserOutput();
#endif //RM_FRAME_C_USERTASK_H
