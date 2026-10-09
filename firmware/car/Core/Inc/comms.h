/**
 * @file    comms.h
 * @brief   UART 通信（→ ESP32 透传）：遥测上报 + 指令接收
 */
#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>

typedef enum {
    CMD_NONE = 0,
    CMD_ESTOP,
    CMD_RESUME
} gs_cmd_t;

void comms_init(void);

/* 一键启动指令：S,<MODE>\n */
void comms_send_start(uint8_t mode);

/* 遥测：C,<mode>,<x>,<y>,<s>,<v>,<state>\n */
void comms_send_telemetry(uint8_t mode, float x, float y, float s, float v,
                          const char *state);

/* 完成：F,<lap_time>\n */
void comms_send_done(float lap_time);

/* 轮询接收并解析指令（在控制循环中调用） */
gs_cmd_t comms_poll_cmd(void);

/* UART 中断接收回调（HAL_UART_RxCpltCallback 中调用） */
void comms_rx_byte(uint8_t b);

#endif /* COMMS_H */
