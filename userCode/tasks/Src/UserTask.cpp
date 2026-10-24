//
// Created by LEGION on 2021/10/4.
//
#include "UserTask.h"
#include "RemoteControl.h"
#include "Motor.h"
#include "Servo.h"


// ===================== PID 电机（你原来的MOTOR_ID_5） =====================
PID_Regulator_t userPidRegulator = {
    .kp = 60,
    .ki = 0,
    .kd = 0,
    .componentKpMax = 10000,
    .componentKiMax = 0,
    .componentKdMax = 0,
    .outputMax = 10000
};
MOTOR_INIT_t userMotorInit = {
    .speedPIDp = &userPidRegulator,
    .anglePIDp = nullptr,
    ._motorID = MOTOR_ID_5,
    .reductionRatio = 36.0f,
    .ctrlType = SPEED_Single,
};
Motor UserMotor(MOTOR_ID_5, &userMotorInit);

// ===================== 4个舵机初始化（TIM2通道） =====================
// 1.底盘旋转舵机
SERVO_INIT_T baseServoInit = {
    .servoType = POSITION_180,
    .servoID = SERVO_ID_5,
    .firstAngle = 90.0f,
    .angleLimit_Min = 0.0f,
    .angleLimit_Max = 180.0f
};
Servo baseServo(&baseServoInit);

// 2.底盘-一级臂关节舵机
SERVO_INIT_T joint1ServoInit = {
    .servoType = POSITION_180,
    .servoID = SERVO_ID_6,
    .firstAngle = 60.0f,
    .angleLimit_Min = 0.0f,
    .angleLimit_Max = 120.0f
};
Servo joint1Servo(&joint1ServoInit);

// 3.二级臂舵机
SERVO_INIT_T joint2ServoInit = {
    .servoType = POSITION_180,
    .servoID = SERVO_ID_7,
    .firstAngle = 65.0f,
    .angleLimit_Min = 0.0f,
    .angleLimit_Max = 130.0f
};
Servo joint2Servo(&joint2ServoInit);

// 4.夹爪舵机（你原来的）
SERVO_INIT_T clawServoInit = {
    .servoType = POSITION_180,
    .servoID = SERVO_ID_8,
    .firstAngle = 90.0f,
    .angleLimit_Min = 0.0f,
    .angleLimit_Max = 180.0f
};
Servo clawServo(&clawServoInit);

// ===================== 参数：死区、平滑滤波 =====================
#define RC_DEAD_ZONE 15
#define SMOOTH_ALPHA 0.2f

static float ang_base_last  = 90.0f;
static float ang_j1_last    = 60.0f;
static float ang_j2_last    = 65.0f;

// 底盘停机标志，给底盘任务读取
uint8_t chassis_stop_flag = 0;

// 摇杆映射+死区+一阶低通滤波
static float Rc_ChannelToAngle_Dead(int16_t ch, float minAng, float maxAng, float *lastAng)
{
    if(ch > -RC_DEAD_ZONE && ch < RC_DEAD_ZONE)
    {
        return *lastAng;
    }
    if(ch > 660) ch = 660;
    if(ch < -660) ch = -660;

    float norm = (ch + 660.0f) / 1320.0f;
    float target = minAng + norm * (maxAng - minAng);
    float out = (*lastAng) * (1 - SMOOTH_ALPHA) + target * SMOOTH_ALPHA;
    *lastAng = out;
    return out;
}

/***
 * 急停函数，底层急停档位调用
 */
void UserStop(){
    UserMotor.Stop();
    baseServo.stop();
    joint1Servo.stop();
    joint2Servo.stop();
    clawServo.stop();
}

/***
 * 上电初始化，只跑一次
 */
void UserInit(){
    printf("机械臂四舵机初始化完成！\r\n");
    printf("ET08遥控机械臂就绪\r\n");
}

/***
 * 用户循环入口：底层框架自动周期性调用，不需要while
 */
void UserHandle(){
    UserMotor.Handle();

    uint8_t sw1 = RemoteControl::rc_ctrl.rc.s[0];  // SW1：模式开关
    uint8_t sw2 = RemoteControl::rc_ctrl.rc.s[1];  // SW2：夹爪开关

    if (sw1 == UP_POS)
    {
        // SW1上拨：底盘停机，进入机械臂手动控制模式
        chassis_stop_flag = 1;

        int16_t ch_left_h  = RemoteControl::rc_ctrl.rc.ch[0];
        int16_t ch_left_v  = RemoteControl::rc_ctrl.rc.ch[1];
        int16_t ch_right_h = RemoteControl::rc_ctrl.rc.ch[2];

        float ang_base = Rc_ChannelToAngle_Dead(ch_left_h, 0.0f, 180.0f, &ang_base_last);
        float ang_j1   = Rc_ChannelToAngle_Dead(ch_left_v, 0.0f, 120.0f, &ang_j1_last);
        float ang_j2   = Rc_ChannelToAngle_Dead(ch_right_h, 0.0f, 130.0f, &ang_j2_last);

        baseServo.SetTargetAngle(ang_base);
        joint1Servo.SetTargetAngle(ang_j1);

        joint2Servo.SetTargetAngle(ang_j2);

        // SW2控制夹爪
        if (sw2 == UP_POS)
        {
            clawServo.SetTargetAngle(70.0f);  // 开
        }
        else
        {
            clawServo.SetTargetAngle(10.0f);  // 合
        }
    }
    else
    {
        // SW1 中/下：底盘允许运行，舵机保持当前角度（不刷新目标）
        chassis_stop_flag = 0;
    }


    baseServo.Handle();
    joint1Servo.Handle();
    joint2Servo.Handle();
    clawServo.Handle();
}
