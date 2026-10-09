/**
 * @file    comms.c
 * @brief   UART 帧收发：ASCII 行协议（\n 结尾），115200
 */
#include "main.h"
#include "car_config.h"
#include "comms.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

static char txbuf[TX_BUF_SIZE];
static uint8_t rxbuf[RX_BUF_SIZE];
static volatile uint16_t rx_len;
static volatile gs_cmd_t pending_cmd;

void comms_init(void)
{
    rx_len = 0;
    pending_cmd = CMD_NONE;
    /* 启动首字节接收 */
    HAL_UART_Receive_IT(&huart1, &rxbuf[0], 1);
}

void comms_rx_byte(uint8_t b)
{
    if (rx_len >= RX_BUF_SIZE - 1) rx_len = 0;  /* 溢出保护 */

    rxbuf[rx_len++] = b;

    if (b == '\n') {
        rxbuf[rx_len - 1] = '\0';
        /* 解析：G,ESTOP / G,RESUME */
        if (strncmp((const char *)rxbuf, "G,ESTOP", 7) == 0) {
            pending_cmd = CMD_ESTOP;
        } else if (strncmp((const char *)rxbuf, "G,RESUME", 8) == 0) {
            pending_cmd = CMD_RESUME;
        }
        rx_len = 0;
    }
    /* 继续接收下一字节 */
    HAL_UART_Receive_IT(&huart1, &rxbuf[rx_len], 1);
}

gs_cmd_t comms_poll_cmd(void)
{
    gs_cmd_t c = pending_cmd;
    pending_cmd = CMD_NONE;
    return c;
}

static void uart_send(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 50);
}

void comms_send_start(uint8_t mode)
{
    const char *m = (mode == MODE_DELIVERY) ? "DELIVERY" : "DYNAMIC_LAND";
    snprintf(txbuf, sizeof(txbuf), "S,%s\n", m);
    uart_send(txbuf);
}

void comms_send_telemetry(uint8_t mode, float x, float y, float s, float v,
                          const char *state)
{
    const char *m = (mode == MODE_DELIVERY) ? "DELIVERY" : "DYNAMIC_LAND";
    snprintf(txbuf, sizeof(txbuf), "C,%s,%.1f,%.1f,%.1f,%.1f,%s\n",
             m, x, y, s, v, state);
    uart_send(txbuf);
}

void comms_send_done(float lap_time)
{
    snprintf(txbuf, sizeof(txbuf), "F,%.1f\n", lap_time);
    uart_send(txbuf);
}
