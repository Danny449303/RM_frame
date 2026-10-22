//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"
// 摇杆非线性映射函数：二次曲线，消除小摇杆灵敏问题
static float JoystickCurve(float input, float maxOut, float deadZone)
{
    if(fabsf(input) < deadZone)
    {
        return 0.0f;
    }
    // 归一化 [-1,1] 二次曲线
    float sign = (input > 0) ? 1.0f : -1.0f;
    float absIn = fabsf(input);
    return sign * maxOut * absIn * absIn;
}

void CtrlHandle(){
    // 超过100ms没有收到有效遥控器帧时，底盘立即停机，防止沿用最后一次速度指令。
    if (!RemoteControl::IsOnline(100u)) {
        ChassisStop();
        return;
    }
    if (RemoteControl::rcInfo.sRight == DOWN_POS){
        //右侧下三档，急停模式，请勿更改！！！
        ChassisStop();
        UserStop();
    } else {
        //摇杆参数
        const float DEAD_ZONE = 0.02f;
        const float MAX_FB = 4.2f;   //前后最大速度 m/s
        const float MAX_LR = 4.2f;   //左右最大速度 m/s
        const float MAX_RT = 60.0f;  //旋转最大转速 RPM
        const float SLOW_MODE_SCALE = 0.10f; //左拨杆中档：首轮架空调试限速

        float fbV = JoystickCurve(RemoteControl::rcInfo.right_col, MAX_FB, DEAD_ZONE);
        float lrV = JoystickCurve(RemoteControl::rcInfo.right_rol, MAX_LR, DEAD_ZONE);
        float rtV = JoystickCurve(RemoteControl::rcInfo.left_rol, MAX_RT, DEAD_ZONE);
        //其他正常模式
        switch (RemoteControl::rcInfo.sLeft)
        {
        case UP_POS: //左侧一档
            ChassisSetVelocity(fbV, lrV, rtV);
            break;
        case MID_POS: //左侧二档：低速调试模式
            ChassisSetVelocity(fbV * SLOW_MODE_SCALE,
                               lrV * SLOW_MODE_SCALE,
                               rtV * SLOW_MODE_SCALE);
            break;
        case DOWN_POS: //左侧三档：底盘停止
            ChassisStop();
            break;
        default:
            break;
        }
    }
}

