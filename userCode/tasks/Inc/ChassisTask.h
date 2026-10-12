//
// Created by LEGION on 2021/10/4.
//

#ifndef RM_FRAME_C_CHASSISTASK_H
#define RM_FRAME_C_CHASSISTASK_H
#include "main.h"
#include "Motor.h"

#define RPM2RADpS(num)(num * (3.1415926f/30.0f))
#define RADpS2RPM(num)(num / (3.1415926f/30.0f))
#define WHEEL_DIAMETER             0.1525f//麦轮直径 m
#define REDUCTION_RATIO 19.0f
#define TRACK_WIDTH            0.40f //轮距
#define WHEEL_BASE             0.38f //轴距
/*枚举类型定义------------------------------------------------------------*/

/*
 * 底盘任务错误信息枚举
 */
typedef enum{
    NONE
}CHASSIS_ERROR_E;

/*结构体定义--------------------------------------------------------------*/

/*
 * 底盘任务状态结构体
 */
typedef struct{

}CHASSIS_STATE_T;

/*
 * 底盘任务控制结构体
 */
typedef struct{


}CHASSIS_CONTROL_T;



/*结构体成员取值定义组------------------------------------------------------*/
/*外部变量声明-------------------------------------------------------------*/

extern CHASSIS_STATE_T     chassisState;
extern CHASSIS_CONTROL_T   chassisControl;

extern float FBVelocity,LRVelocity,RTVelocity;

/*外部函数声明-------------------------------------------------------------*/

void WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity);
void ChassisStop();
void ChassisSetVelocity(float _fbV,float _lrV,float _rtV);
#endif //RM_FRAME_C_CHASSISTASK_H
