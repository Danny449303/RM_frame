//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_CONTROLTASK_H
#define RM_FRAME_C_CONTROLTASK_H

#include "main.h"
#include "ChassisTask.h"
#include "Motor.h"
#include "UserTask.h"
#include "RemoteControl.h"

/*枚举类型定义------------------------------------------------------------*/

/*
 * 控制任务错误信息枚举
 */
typedef enum{
    NONE1,
}CTRL_ERROR_E;

/*结构体定义--------------------------------------------------------------*/

/*
 * 控制任务状态结构体
 */
typedef struct{

}CTRL_STATE_T;

/*
 * 控制任务控制结构体
 */
typedef struct{


}CTRL_CONTROL_T;



/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/

extern CTRL_STATE_T     ctrlState;
extern CTRL_CONTROL_T   ctrlControl;

/*外部函数声明-------------------------------------------------------------*/

CTRL_ERROR_E CtrlStart(CTRL_CONTROL_T _ctrlControl);
CTRL_ERROR_E CtrlHandle(CTRL_CONTROL_T _ctrlControl);

#endif //RM_FRAME_C_CONTROLTASK_H
