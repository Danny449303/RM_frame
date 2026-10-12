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
    UP_POS = 97u,MID_POS = 0u,DOWN_POS = 158u
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
typedef struct {
    float left_col,left_rol;
    float right_col,right_rol;
    SWITCH_STATE_E sLeft;
    uint32_t pat;
    SWITCH_STATE_E sRight;
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
