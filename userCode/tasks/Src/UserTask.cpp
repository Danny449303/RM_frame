//
// Created by LEGION on 2021/10/4.
//
#include "UserTask.h"
#include "RemoteControl.h"
#include "Servo.h"

PID_Regulator_t userPidRegulator = {
        .kp = 60,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 10000,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 10000 //2006电机输出电流上限，可以调小，勿调大
};

MOTOR_INIT_t userMotorInit = {
        .speedPIDp = &userPidRegulator,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 36.0f,
        .ctrlType = SPEED_Single,
};

// 在这里初始化除了4个M3508轮电机之外的电机
Motor UserMotor(MOTOR_ID_5,&userMotorInit);

//舵机抓取
SERVO_INIT_T clawServoInit = {
    .servoType = POSITION_180,
    .servoID = SERVO_ID_8,   // 对应我们新增的TIM2通道
    .firstAngle = 90.0f,     // 上电默认90°中位
    .angleLimit_Min = 0.0f,
    .angleLimit_Max = 180.0f
};
Servo clawServo(&clawServoInit);
/***
 * 在这里放入xxx.stop()即可使舵机,电机在遥控器急停挡位断电
 */
void UserStop(){
    UserMotor.Stop();
    clawServo.stop();
}

/***
 * 在这里写入初始化内容
 */
void UserInit(){
    printf("夹爪舵机初始化完成！\r\n");
    printf("ET08遥控器控制夹爪就绪\r\n");
}

static bool clawOpenFlag = true;
static uint32_t tickCount = 0;
/***
 * 用户自定义任务主循环
 */
void UserHandle(){
    UserMotor.Handle();

    // 读取ET08拨码开关 SW1 s[0]
    uint32_t sw1_state = RemoteControl::rc_ctrl.rc.s[0];

    if(sw1_state == UP_POS || sw1_state == MID_POS)
    {
        // 上挡 / 中挡：夹爪张开
        clawServo.SetTargetAngle(0.0f);
    }
    else if(sw1_state == DOWN_POS)
    {
        // 下挡：夹爪闭合
        clawServo.SetTargetAngle(180.0f);
    }

    clawServo.Handle(); //必须每次调用刷新PWM
}