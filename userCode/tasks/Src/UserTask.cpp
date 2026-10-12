//
// Created by LEGION on 2021/10/4.
//
#include "UserTask.h"

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
}

static bool clawOpenFlag = true;
static uint32_t tickCount = 0;
/***
 * 用户自定义任务主循环
 */
void UserHandle(){
    UserMotor.Handle();
    // ========= 舵机测试逻辑，每1000个周期切换一次开合 =========
    tickCount ++;
    if(tickCount >= 1000)
    {
        tickCount = 0;
        if(clawOpenFlag)
        {
            clawServo.SetTargetAngle(0.0f);   // 张开
        }
        else
        {
            clawServo.SetTargetAngle(180.0f); // 闭合
        }
        clawOpenFlag = !clawOpenFlag;
    }
    clawServo.Handle(); // 必须周期调用，刷新PWM输出！
}