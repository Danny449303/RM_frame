//
// Created by LEGION on 2021/10/4.
//
#include "UserTask.h"
#include "RemoteControl.h"
#include <math.h>

volatile ROBOT_MODE_E robotMode = ROBOT_MODE_DRIVE;
volatile uint8_t robotEmergency = 0;

/* ================= 机械臂参数 ================= */
#define PI_F                3.1415926f
#define DEG2RAD(x)          ((x) * PI_F / 180.0f)
#define RAD2DEG(x)          ((x) * 180.0f / PI_F)

#define L1                  102.75f     // mm
#define L2                  121.75f     // mm
#define JOINT1_DX           13.4f       // 关节1相对底座中心的水平前移
#define JOINT1_DY           39.0f       // 关节1相对底座中心的竖直高度

#define TH1_MIN_DEG         (-28.0f)
#define TH1_MAX_DEG         (68.0f)
#define TH2_MIN_DEG         (2.0f)
#define TH2_MAX_DEG         (88.0f)

#define ARM_TARGET_SPEED    80.0f       // mm/s
#define JOINT_SPEED_DEG     120.0f      // deg/s
#define BASE_SPEED_DEG      90.0f
#define WRIST_SPEED_DEG     90.0f
#define PLATFORM_SPEED_DEG  90.0f
#define GRIPPER_SPEED_DEG   180.0f

/* ================= 舵机通道 / 角度 ================= */
#define PCA_CH_PLATFORM     0
#define PCA_CH_BASE         1
#define PCA_CH_JOINT1       2
#define PCA_CH_JOINT2       3
#define PCA_CH_WRIST        4
#define PCA_CH_GRIPPER      5

#define PLATFORM_DOWN_ANGLE 15.0f
#define PLATFORM_UP_ANGLE   165.0f
#define GRIPPER_CLOSE_ANGLE 20.0f
#define GRIPPER_OPEN_ANGLE  160.0f

#define BASE_MIN_ANGLE      5.0f
#define BASE_MAX_ANGLE      175.0f
#define BASE_ZERO_ANGLE     90.0f
#define WRIST_MIN_ANGLE     2.0f
#define WRIST_MAX_ANGLE     43.0f

/*
 * 机构角 -> 舵机命令角
 * 如果装好后方向反了，只改这里的 DIR/ZERO
 */
#define J1_SERVO_ZERO       90.0f
#define J1_SERVO_DIR        1.0f
#define J2_SERVO_ZERO       0.0f
#define J2_SERVO_DIR        1.0f
#define WRIST_SERVO_ZERO    90.0f
#define WRIST_SERVO_DIR     1.0f

static float g_th1 = 0.0f;              // 关节1，度
static float g_th2 = 88.0f;             // 关节2，度
static float g_base = BASE_ZERO_ANGLE;
static float g_wrist = 0.0f;
static float g_platform = PLATFORM_DOWN_ANGLE;
static float g_gripper = GRIPPER_CLOSE_ANGLE;

static float g_targetX = 0.0f;
static float g_targetY = 0.0f;

static float moveToward(float cur, float target, float maxStep)
{
    float d = target - cur;
    if (d > maxStep) d = maxStep;
    if (d < -maxStep) d = -maxStep;
    return cur + d;
}

static float clampf(float v, float minV, float maxV)
{
    if (v < minV) return minV;
    if (v > maxV) return maxV;
    return v;
}

/* 目标点相对底座旋转中心 (x 前, y 上)，输出关节角（度） */
static void armSolveIK(float xb, float yb, float *th1Deg, float *th2Deg)
{
    float x = xb - JOINT1_DX;
    float y = yb - JOINT1_DY;

    float r2 = x * x + y * y;
    float c2 = (r2 - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

    // 只取 θ2 = 0~90° 的解
    c2 = clampf(c2, 0.0f, 1.0f);
    float th2 = acosf(c2);

    float th1 = atan2f(x, y) - atan2f(L2 * sinf(th2), L1 + L2 * cosf(th2));

    float th1DegTmp = RAD2DEG(th1);
    float th2DegTmp = RAD2DEG(th2);

    *th1Deg = clampf(th1DegTmp, TH1_MIN_DEG, TH1_MAX_DEG);
    *th2Deg = clampf(th2DegTmp, TH2_MIN_DEG, TH2_MAX_DEG);
}

static void limitArmTarget(void)
{
    float x1 = g_targetX - JOINT1_DX;
    float y1 = g_targetY - JOINT1_DY;
    float r = sqrtf(x1 * x1 + y1 * y1);

    const float RMAX = L1 + L2;
    const float RMIN = fabsf(L1 - L2);

    if (r > RMAX) {
        float s = RMAX / r;
        x1 *= s;
        y1 *= s;
    } else if (r < RMIN) {
        if (r < 0.001f) {
            x1 = RMIN;
            y1 = 0.0f;
        } else {
            float s = RMIN / r;
            x1 *= s;
            y1 *= s;
        }
    }

    g_targetX = x1 + JOINT1_DX;
    g_targetY = y1 + JOINT1_DY;
}

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

void UserStop(){
    UserMotor.Stop();
}

void UserInit(){
    robotMode = ROBOT_MODE_DRIVE;
    robotEmergency = 0;

    // 关节初始位置：θ1=0°, θ2=88°
    float x1 = L1 * sinf(DEG2RAD(0.0f)) + L2 * sinf(DEG2RAD(88.0f));
    float y1 = L1 * cosf(DEG2RAD(0.0f)) + L2 * cosf(DEG2RAD(88.0f));
    g_targetX = x1 + JOINT1_DX;
    g_targetY = y1 + JOINT1_DY;

    g_th1 = 0.0f;
    g_th2 = 88.0f;
    g_base = BASE_ZERO_ANGLE;
    g_wrist = 0.0f;
    g_platform = PLATFORM_DOWN_ANGLE;
    g_gripper = GRIPPER_CLOSE_ANGLE;

    PCA9685::Init();

    PCA9685::SetAngle(PCA_CH_PLATFORM, g_platform);
    PCA9685::SetAngle(PCA_CH_BASE, g_base);
    PCA9685::SetAngle(PCA_CH_JOINT1, J1_SERVO_ZERO + J1_SERVO_DIR * g_th1);
    PCA9685::SetAngle(PCA_CH_JOINT2, J2_SERVO_ZERO + J2_SERVO_DIR * g_th2);
    PCA9685::SetAngle(PCA_CH_WRIST, WRIST_SERVO_ZERO + WRIST_SERVO_DIR * g_wrist);
    PCA9685::SetAngle(PCA_CH_GRIPPER, g_gripper);
}

void UserHandle(){
    // 急停：保持所有舵机当前位置
    if (robotEmergency) {
        return;
    }

    // 平台：A 按钮，上=上升，下=下降
    float platformTarget = (RemoteControl::rcInfo.buttonA == SWITCH_UP)
                           ? PLATFORM_UP_ANGLE : PLATFORM_DOWN_ANGLE;
    g_platform = moveToward(g_platform, platformTarget, PLATFORM_SPEED_DEG * 0.001f);

    // 底座：左旋钮
    float baseTarget = BASE_ZERO_ANGLE + RemoteControl::rcInfo.left_knob * 85.0f;
    baseTarget = clampf(baseTarget, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
    g_base = moveToward(g_base, baseTarget, BASE_SPEED_DEG * 0.001f);

    // 腕部：右旋钮，顺时针向下，逆时针向上
    float wristTarget = WRIST_MIN_ANGLE +
                        (1.0f - RemoteControl::rcInfo.right_knob) * 0.5f *
                        (WRIST_MAX_ANGLE - WRIST_MIN_ANGLE);
    wristTarget = clampf(wristTarget, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE);
    g_wrist = moveToward(g_wrist, wristTarget, WRIST_SPEED_DEG * 0.001f);

    // 夹爪：第三个按钮下档打开，其他档闭合
    float gripperTarget = (robotMode == ROBOT_MODE_ARM_OPEN)
                          ? GRIPPER_OPEN_ANGLE : GRIPPER_CLOSE_ANGLE;
    g_gripper = moveToward(g_gripper, gripperTarget, GRIPPER_SPEED_DEG * 0.001f);

    // 机械臂关节：上档驾驶时保持，中/下档右摇杆控制
    if (robotMode != ROBOT_MODE_DRIVE) {
        g_targetX += RemoteControl::rcInfo.right_rol * ARM_TARGET_SPEED * 0.001f;
        g_targetY += RemoteControl::rcInfo.right_col * ARM_TARGET_SPEED * 0.001f;
        limitArmTarget();

        float th1, th2;
        armSolveIK(g_targetX, g_targetY, &th1, &th2);

        g_th1 = moveToward(g_th1, th1, JOINT_SPEED_DEG * 0.001f);
        g_th2 = moveToward(g_th2, th2, JOINT_SPEED_DEG * 0.001f);
    }
}

/*
 * PCA9685 的 I2C 写在主循环里做，避免在 1ms 中断里阻塞。
 * 50Hz 更新一次。
 */
void UserOutput(){
    static uint32_t lastWrite = 0;

    if ((uint32_t)(HAL_GetTick() - lastWrite) < 20u) {
        return;
    }
    lastWrite = HAL_GetTick();

    PCA9685::SetAngle(PCA_CH_PLATFORM, g_platform);
    PCA9685::SetAngle(PCA_CH_BASE, g_base);
    PCA9685::SetAngle(PCA_CH_JOINT1, J1_SERVO_ZERO + J1_SERVO_DIR * g_th1);
    PCA9685::SetAngle(PCA_CH_JOINT2, J2_SERVO_ZERO + J2_SERVO_DIR * g_th2);
    PCA9685::SetAngle(PCA_CH_WRIST, WRIST_SERVO_ZERO + WRIST_SERVO_DIR * g_wrist);
    PCA9685::SetAngle(PCA_CH_GRIPPER, g_gripper);
}
