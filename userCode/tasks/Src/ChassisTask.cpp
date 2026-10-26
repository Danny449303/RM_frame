//
// Created by LEGION on 2021/10/4.
//
#include "ChassisTask.h"
// 四个独立PID
PID_Regulator_t pidFL = {.kp = 60,.ki = 0,.kd = 0,.componentKpMax = 16384,.componentKiMax = 0,.componentKdMax = 0,.outputMax = 16384};
PID_Regulator_t pidFR = {.kp = 60,.ki = 0,.kd = 0,.componentKpMax = 16384,.componentKiMax = 0,.componentKdMax = 0,.outputMax = 16384};
PID_Regulator_t pidBL = {.kp = 60,.ki = 0,.kd = 0,.componentKpMax = 16384,.componentKiMax = 0,.componentKdMax = 0,.outputMax = 16384};
PID_Regulator_t pidBR = {.kp = 60,.ki = 0,.kd = 0,.componentKpMax = 16384,.componentKiMax = 0,.componentKdMax = 0,.outputMax = 16384};

MOTOR_INIT_t chassisMotorInitFL = {.speedPIDp = &pidFL,.anglePIDp = nullptr,._motorID = MOTOR_ID_1,.reductionRatio = 19.0f,.ctrlType = SPEED_Single};
MOTOR_INIT_t chassisMotorInitFR = {.speedPIDp = &pidFR,.anglePIDp = nullptr,._motorID = MOTOR_ID_2,.reductionRatio = 19.0f,.ctrlType = SPEED_Single};
MOTOR_INIT_t chassisMotorInitBL = {.speedPIDp = &pidBL,.anglePIDp = nullptr,._motorID = MOTOR_ID_3,.reductionRatio = 19.0f,.ctrlType = SPEED_Single};
MOTOR_INIT_t chassisMotorInitBR = {.speedPIDp = &pidBR,.anglePIDp = nullptr,._motorID = MOTOR_ID_4,.reductionRatio = 19.0f,.ctrlType = SPEED_Single};

Motor CMFL(MOTOR_ID_1,&chassisMotorInitFL);
Motor CMFR(MOTOR_ID_2,&chassisMotorInitFR);
Motor CMBL(MOTOR_ID_3,&chassisMotorInitBL);
Motor CMBR(MOTOR_ID_4,&chassisMotorInitBR);
//这里把四个电机的pidRegulator改成四个独立的，防止积分污染
/*PID_Regulator_t pidRegulator = {//此为储存pid参数的结构体，四个底盘电机共用
        .kp = 60,
        .ki = 0,
        .kd = 0,
        .componentKpMax = 16384,
        .componentKiMax = 0,
        .componentKdMax = 0,
        .outputMax = 16384 //3508电机输出电流上限，可以调小，勿调大
};

MOTOR_INIT_t chassisMotorInit = {//四个底盘电机共用的初始化结构体
        .speedPIDp = &pidRegulator,
        .anglePIDp = nullptr,
        ._motorID = MOTOR_ID_1,
        .reductionRatio = 19.0f,
        .ctrlType = SPEED_Single,
};
Motor CMFL(MOTOR_ID_1,&chassisMotorInit);//定义左前轮电机
Motor CMFR(MOTOR_ID_2,&chassisMotorInit);//定义右前轮电机
Motor CMBL(MOTOR_ID_3,&chassisMotorInit);//定义左后轮电机
Motor CMBR(MOTOR_ID_4,&chassisMotorInit);//定义右后轮电机
*/
uint8_t ChassisStopFlag = 1;
float FBVelocity,LRVelocity,RTVelocity;

void ChassisStart(){

}
/**
 * @brief 底盘任务的处理函数，定时执行
 * @callergraph void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) in Device.cpp
 */
void ChassisHandle() {
    if(ChassisStopFlag == 0) {
        WheelsSpeedCalc(FBVelocity, LRVelocity, RTVelocity);
    }
    CMFL.Handle();
    CMFR.Handle();
    CMBL.Handle();
    CMBR.Handle();
}
/**
 * @brief 用于控制任务控制底盘速度
 * @param _fbV 底盘前后方向速度
 * @param _lrV 底盘左右方向速度
 * @param _rtV 底盘旋转速度
 */
void ChassisSetVelocity(float _fbV,float _lrV,float _rtV){
    ChassisStopFlag = 0;
    FBVelocity = _fbV;
    LRVelocity = _lrV;
    RTVelocity = _rtV;
}
/**
 * @brief 执行急停模式的底盘任务处理
 */
void ChassisStop(){
    ChassisStopFlag = 1;
    CMFL.Stop();
    CMFR.Stop();
    CMBL.Stop();
    CMBR.Stop();
}
/*麦轮四个轮子速度在这里，后续修改需要修改正负号的问题*/
void WheelsSpeedCalc(float fbVelocity, float lrVelocity, float rtVelocity) {
    float CMFLSpeed, CMFRSpeed, CMBLSpeed, CMBRSpeed;
    // ========== 摇杆输入死区（兜底） ==========
    const float DEAD_ZONE = 0.02f;
    if(fabsf(fbVelocity) < DEAD_ZONE) fbVelocity = 0.0f;
    if(fabsf(lrVelocity) < DEAD_ZONE) lrVelocity = 0.0f;
    if(fabsf(rtVelocity) < DEAD_ZONE) rtVelocity = 0.0f;
    // =========================================
    // 注意！！！controltask里传入rt是60，单位是RPM，所以这里保留RPM2RADpS转换
    rtVelocity = RPM2RADpS(rtVelocity);

    const float L = (WHEEL_BASE + TRACK_WIDTH) / 2.0f;

    //计算四个轮子线速度 m/s
    CMFLSpeed = fbVelocity - lrVelocity - L * rtVelocity;
    CMFRSpeed = fbVelocity + lrVelocity + L * rtVelocity;
    CMBLSpeed = fbVelocity + lrVelocity - L * rtVelocity;
    CMBRSpeed = fbVelocity - lrVelocity + L * rtVelocity;

    // ========== 轮子线速度限幅，m/s，3508麦轮常用值3~5m/s，按需修改 ==========
    const float MAX_WHEEL_LINE_SPEED = 4.0f;
    CMFLSpeed = fminf(fmaxf(CMFLSpeed, -MAX_WHEEL_LINE_SPEED), MAX_WHEEL_LINE_SPEED);
    CMFRSpeed = fminf(fmaxf(CMFRSpeed, -MAX_WHEEL_LINE_SPEED), MAX_WHEEL_LINE_SPEED);
    CMBLSpeed = fminf(fmaxf(CMBLSpeed, -MAX_WHEEL_LINE_SPEED), MAX_WHEEL_LINE_SPEED);
    CMBRSpeed = fminf(fmaxf(CMBRSpeed, -MAX_WHEEL_LINE_SPEED), MAX_WHEEL_LINE_SPEED);
    // ========================================================================
    //
    //
    //线速度转轮子角速度 rad/s
    CMFLSpeed = CMFLSpeed / (WHEEL_DIAMETER / 2.0f);
    CMFRSpeed = CMFRSpeed / (WHEEL_DIAMETER / 2.0f);
    CMBLSpeed = CMBLSpeed / (WHEEL_DIAMETER / 2.0f);
    CMBRSpeed = CMBRSpeed / (WHEEL_DIAMETER / 2.0f);

    //轮子角速度转电机RPM
    CMFL.SetTargetSpeed(RADpS2RPM(CMFLSpeed));
    CMFR.SetTargetSpeed(RADpS2RPM(CMFRSpeed));
    CMBL.SetTargetSpeed(RADpS2RPM(CMBLSpeed));
    CMBR.SetTargetSpeed(RADpS2RPM(CMBRSpeed));
}
