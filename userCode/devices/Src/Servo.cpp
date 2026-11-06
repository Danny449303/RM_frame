//
// Created by LEGION on 2021/10/5.
//

#include "Servo.h"

const SERVO_HANDLE_T servoInfo[7]={
        {&htim1, TIM_CHANNEL_1},
        {&htim1, TIM_CHANNEL_2},
        {&htim1, TIM_CHANNEL_3},
        {&htim1, TIM_CHANNEL_4},
        {&htim8, TIM_CHANNEL_1},
        {&htim8, TIM_CHANNEL_2},
        {&htim8, TIM_CHANNEL_3},
};

uint32_t Servo::servo_IDs;
/**
 * @brief 舵机类的初始化函数，开启了相关定时器
 */
void Servo::Init() {
    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim8);
}
/**
 * @brief 舵机的构造函数，进行舵机的初始化设置
 * @param servoInit 舵机的初始化结构体指针
 */
Servo::Servo(SERVO_INIT_T *servoInit) {
    deviceID = servoInit->servoID;
    deviceType = SERVO;

    angleLimit_Min = servoInit->angleLimit_Min;
    angleLimit_Max = servoInit->angleLimit_Max;

    servo_IDs |= deviceID;
    servoType = servoInit->servoType;
    float duty;
    if(servoType == POSITION_180) {
        duty = AFFINE(0,180,0.025,0.125,servoInit->firstAngle);
    }else if(servoType == SPEED_360){
        duty = 0.075;
    }
    auto pos = GET_SERVO_POS(deviceID);

    __HAL_TIM_SET_COMPARE(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel,
            htim1.Instance->ARR*duty);

}

void Servo::stop() {
    if (stopFlag == 0){
        stopFlag = 1;
        auto pos = GET_SERVO_POS(deviceID);
        HAL_TIM_PWM_Stop(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel);
    }
}
/**
 * @brief 设置舵机角度
 * @param _targetAngle
 */
void Servo::SetTargetAngle(float _targetAngle) {
    //TODO 检查舵机类型,检查角度限位
    if(stopFlag == 1){
        stopFlag = 0;
        auto pos = GET_SERVO_POS(deviceID);
        HAL_TIM_PWM_Start(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel);
    }
    INRANGE(_targetAngle,angleLimit_Min,angleLimit_Max);
    targetAngle = _targetAngle;
}
/**
 * @设置舵机速度
 * @param _targetSpeed 舵机速度
 */
void Servo::SetTargetSpeed(float _targetSpeed) {
    //TODO 检查舵机类型
    if(stopFlag == 1){
        stopFlag = 0;
        auto pos = GET_SERVO_POS(deviceID);
        HAL_TIM_PWM_Start(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel);
    }
    targetSpeed = _targetSpeed;
}
/**
 * @brief 舵机的控制驱动函数
 */
void Servo::Handle() {
    if (servoType == POSITION_180){
        duty = AFFINE(0,180,0.025,0.125,targetAngle);
        auto pos = GET_SERVO_POS(deviceID);

        __HAL_TIM_SET_COMPARE(servoInfo[pos].handleTypeDef,servoInfo[pos].timChannel,
                              htim1.Instance->ARR*duty);

    }else if(servoType == SPEED_360){

    }
}

void Servo::ErrorHandle() {

}

/* ================= PCA9685 ================= */
#define PCA9685_I2C_ADDR   (0x40u << 1)
#define PCA9685_MODE1      0x00u
#define PCA9685_MODE2      0x01u
#define PCA9685_PRESCALE   0xFEu

void PCA9685::WriteReg(uint8_t reg, uint8_t value)
{
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_I2C_ADDR, reg,
                      I2C_MEMADD_SIZE_8BIT, &value, 1, 5);
}

void PCA9685::WriteRegs(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    HAL_I2C_Mem_Write(&hi2c2, PCA9685_I2C_ADDR, reg,
                      I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, 5);
}

void PCA9685::Init()
{
    // 50Hz：prescale = round(25MHz / (4096 * 50)) - 1 = 121
    WriteReg(PCA9685_MODE1, 0x10);   // sleep
    WriteReg(PCA9685_PRESCALE, 121);
    HAL_Delay(2);
    WriteReg(PCA9685_MODE1, 0xA0);   // restart + auto-increment
    HAL_Delay(5);
    WriteReg(PCA9685_MODE2, 0x04);   // totem pole output

    // 上电先全部置 0，避免舵机乱跳
    for (uint8_t ch = 0; ch < 16; ch++) {
        SetPulse(ch, 0);
    }
}

void PCA9685::SetPulse(uint8_t channel, uint16_t pulseUs)
{
    if (channel >= 16) return;

    uint32_t count = ((uint32_t)pulseUs * 4096u) / 20000u;
    if (count > 4095u) count = 4095u;

    uint8_t buf[4];
    buf[0] = 0;                  // LEDn_ON_L
    buf[1] = 0;                  // LEDn_ON_H
    buf[2] = (uint8_t)(count & 0xFF);
    buf[3] = (uint8_t)(count >> 8);

    WriteRegs((uint8_t)(0x06u + 4u * channel), buf, 4);
}

void PCA9685::SetAngle(uint8_t channel, float angle)
{
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    // 180° 舵机：0.5ms~2.5ms
    uint16_t pulse = (uint16_t)(500.0f + angle * (2000.0f / 180.0f));
    SetPulse(channel, pulse);
}

