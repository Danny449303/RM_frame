//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"

// 车体方向符号：装车后如果前后/平移/自转反了，只改这里
#define CHASSIS_FB_SIGN   (-1.0f)
#define CHASSIS_LR_SIGN   (-1.0f)
#define CHASSIS_RT_SIGN   (-1.0f)

void CtrlHandle(){
    // B 按钮下档：急停
    robotEmergency = (RemoteControl::rcInfo.buttonB == SWITCH_DOWN) ? 1u : 0u;

    if (robotEmergency) {
        ChassisStop();
        UserStop();
        return;
    }

    switch (RemoteControl::rcInfo.button3)
    {
    case SWITCH_UP:   // 第三个按钮上档：正常驾驶
        robotMode = ROBOT_MODE_DRIVE;
        ChassisSetVelocity(CHASSIS_FB_SIGN * RemoteControl::rcInfo.right_col * 4.2f,
                           CHASSIS_LR_SIGN * RemoteControl::rcInfo.right_rol * 4.2f,
                           CHASSIS_RT_SIGN * RemoteControl::rcInfo.left_rol * 60.0f);
        break;

    case SWITCH_MID:  // 中档：底盘锁死，右摇杆控制机械臂
        robotMode = ROBOT_MODE_ARM;
        ChassisStop();
        break;

    case SWITCH_DOWN: // 下档：底盘锁死，机械臂可动，夹爪打开
        robotMode = ROBOT_MODE_ARM_OPEN;
        ChassisStop();
        break;

    default:
        robotMode = ROBOT_MODE_ARM;
        ChassisStop();
        break;
    }
}
