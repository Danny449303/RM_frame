//
// Created by LEGION on 2021/10/5.
//

#ifndef RM_FRAME_C_SERVO_H
#define RM_FRAME_C_SERVO_H

#include "Device.h"

#define AFFINE(BEFORE_MIN,BEFORE_MAX,TARGET_MIN,TARGET_MAX,X)\
(X-BEFORE_MIN)/(float)(BEFORE_MAX - BEFORE_MIN)*(TARGET_MAX - TARGET_MIN)\
+ TARGET_MIN
#define GET_SERVO_POS(ID)  (uint32_t)(log2(ID))
/*枚举类型定义------------------------------------------------------------*/
typedef enum{
    POSITION_180,SPEED_360
}SERVO_TYPE_E;
/*结构体定义--------------------------------------------------------------*/
/*
 * 舵机初始化结构体
 */
typedef struct{
    SERVO_TYPE_E servoType;
    uint32_t servoID;
    float firstAngle,angleLimit_Min,angleLimit_Max;//only valid for POSITION_180 servo
}SERVO_INIT_T;

typedef struct{
    TIM_HandleTypeDef* handleTypeDef;
    uint32_t timChannel;
}SERVO_HANDLE_T;

/*类型定义----------------------------------------------------------------*/
class Servo :public Device
{
    static uint32_t servo_IDs;
    static void Init();

    uint8_t stopFlag{1};
    SERVO_TYPE_E servoType;

    float angleLimit_Min,angleLimit_Max;

    float duty{0};
    float targetAngle{0};
    float targetSpeed{0};

    void SetTargetSpeed(float _targetSpeed);

    void ErrorHandle() override;


public:
    explicit Servo(SERVO_INIT_T* servoInit);

    void stop();

    void SetTargetAngle(float _targetAngle);

    void Handle() override;
};
/*结构体成员取值定义组------------------------------------------------------*/

/*
 * SEVRO_ID
 */
#define SERVO_ID_1 0x001
#define SERVO_ID_2 0x002
#define SERVO_ID_3 0x004
#define SERVO_ID_4 0x008
#define SERVO_ID_5 0x010
#define SERVO_ID_6 0x020
#define SERVO_ID_7 0x040

/*外部变量声明-------------------------------------------------------------*/

/*外部函数声明-------------------------------------------------------------*/
#endif //RM_FRAME_C_SERVO_H
