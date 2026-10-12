//
// Created by LEGION on 2021/10/4.
//
#include "ControlTask.h"
void CtrlHandle(){
    if (RemoteControl::rcInfo.sRight == DOWN_POS){
        //右侧下三档，急停模式，请勿更改！！！
        ChassisStop();
        UserStop();
    } else {
        //其他正常模式
        switch (RemoteControl::rcInfo.sLeft)
        {
        case UP_POS: //左侧一档
            ChassisSetVelocity(RemoteControl::rcInfo.right_col*4.2,
                               RemoteControl::rcInfo.right_rol*4.2,RemoteControl::rcInfo.left_rol*60);
            break;
        case MID_POS: //左侧二档

            break;
        case DOWN_POS: //左侧三档

            break;
        default:
            break;
        }
    }
}

