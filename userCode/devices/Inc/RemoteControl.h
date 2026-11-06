//
// Created by LEGION on 2021/10/10.
//

#ifndef RM_FRAME_C_REMOTECONTROL_H
#define RM_FRAME_C_REMOTECONTROL_H

#include "Device.h"

#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 25u
#define RC_CH_VALUE_OFFSET 1024
/*枚举类型定义------------------------------------------------------------*/
typedef enum{
    SWITCH_DOWN = 0u,
    SWITCH_MID  = 1u,
    SWITCH_UP   = 2u
}SWITCH_STATE_E;
/*结构体定义--------------------------------------------------------------*/
typedef struct {
    int32_t ch[4];
    uint32_t s[2];
    int16_t optional[2];
}rc_t;

typedef struct {
    int32_t x,y,z,press_l,press_r;
}mouse_t;

typedef struct {
    uint32_t v;
}keyboard_t;

typedef struct {
    rc_t rc;
    mouse_t mouse;
    keyboard_t key;
}RC_ctrl_t;

/*
 * ET08 通道分配（不改遥控器通道）：
 * CH1 = J1 右摇杆左右     -> right_rol
 * CH2 = J3 右摇杆上下     -> right_col
 * CH3 = LD 左旋钮         -> left_knob
 * CH4 = J4 左摇杆左右     -> left_rol
 * CH5 = SA 第一个按钮 A   -> buttonA（2 档）
 * CH6 = SB 第二个按钮 B   -> buttonB（3 档，下档急停）
 * CH7 = SC 第三个按钮     -> button3（3 档）
 * CH8 = RD 右旋钮         -> right_knob
 */
typedef struct {
    float right_col, right_rol;
    float left_rol;
    float left_knob, right_knob;   // -1.0f ~ +1.0f
    SWITCH_STATE_E buttonA;        // 2 档
    SWITCH_STATE_E buttonB;        // 3 档
    SWITCH_STATE_E button3;        // 3 档
    int16_t optional[2];
}RC_Info_t;
/*类型定义----------------------------------------------------------------*/
class RemoteControl : public Device {
public:
    static RC_Info_t rcInfo;
    static RC_ctrl_t rc_ctrl;

    static void init();
    static void ITHandle();
    static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
    static void sbus_to_rc(volatile const uint8_t *sbus_buf);
};

/*结构体成员取值定义组------------------------------------------------------*/

/*外部变量声明-------------------------------------------------------------*/

/*外部函数声明-------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif
extern void USART3_IRQHandler(void);
#ifdef __cplusplus
}
#endif

#endif //RM_FRAME_C_REMOTECONTROL_H
