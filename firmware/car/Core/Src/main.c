/**
 * @file    main.c
 * @brief   循线小车主程序（STM32F103C8T6, HAL 库, 72MHz）
 *
 * 任务流程：
 *   IDLE ──短按启动──► STARTING(发 UAV_START+软启动) ──► RUNNING(循线+里程计)
 *        ──里程≥一圈──► DONE(停车+上报)
 *   长按 1s 在抛投/动态起降两种任务模式间切换
 *   地面站 ESTOP 可随时急停
 */
#include "main.h"
#include "car_config.h"
#include "pid.h"
#include "line_sensor.h"
#include "motor.h"
#include "odometry.h"
#include "comms.h"
#include <string.h>

/* ================= 外设句柄 ================= */
TIM_HandleTypeDef htim2;   /* PWM */
TIM_HandleTypeDef htim3;   /* 左编码器 */
TIM_HandleTypeDef htim4;   /* 右编码器 */
UART_HandleTypeDef huart1; /* ESP32 */

/* ================= 应用状态 ================= */
typedef enum {
    CAR_IDLE = 0,
    CAR_STARTING,
    CAR_RUNNING,
    CAR_DONE
} car_state_t;

static volatile car_state_t car_state = CAR_IDLE;
static volatile uint8_t     task_mode  = MODE_DELIVERY;

static odom_t  odom;
static pid_t   yaw_pid;
static float   cur_speed_l, cur_speed_r;
static uint8_t uav_start_sent;
static uint8_t estop;
static uint32_t lap_start_ms;

/* ================= 速度档位与软启动 ================= */
static float speed_profile(float s)
{
    if (task_mode == MODE_DYN_LAND) {
        if (s < SEG_B) return V_DYN_S1;
        if (s < SEG_D) return V_DYN_S2;
        return V_DYN_S3;
    }
    /* 抛投任务 */
    if (s < SEG_B) return V_DEL_S1;
    if (s < SEG_D) return V_DEL_S2;
    return V_DEL_S3;
}

static void ramp_to(float target, float dt)
{
    float dv = ACCEL_LIMIT * dt;
    float dl = target - cur_speed_l;
    float dr = target - cur_speed_r;
    if (dl >  dv) dl =  dv;
    if (dl < -dv) dl = -dv;
    if (dr >  dv) dr =  dv;
    if (dr < -dv) dr = -dv;
    cur_speed_l += dl;
    cur_speed_r += dr;
}

/* ================= 200Hz 控制周期 ================= */
static void control_tick(void)
{
    static uint32_t last_t = 0;
    uint32_t now = HAL_GetTick();
    float dt = (now - last_t) / 1000.0f;
    if (dt <= 0.0f || dt > 0.05f) dt = 0.005f;
    last_t = now;

    gs_cmd_t cmd = comms_poll_cmd();
    if (cmd == CMD_ESTOP) { estop = 1; motor_stop(); return; }
    if (cmd == CMD_RESUME) { estop = 0; }

    switch (car_state) {
    case CAR_IDLE:
        motor_stop();
        cur_speed_l = cur_speed_r = 0.0f;
        break;

    case CAR_STARTING:
        /* 发启动指令（重发 3 次防丢），软启动至目标速度 */
        if (!uav_start_sent) {
            for (int i = 0; i < 3; i++) comms_send_start(task_mode);
            uav_start_sent = 1;
            lap_start_ms = HAL_GetTick();
            odom_reset(&odom);
        }
        ramp_to(speed_profile(odom.s), dt);
        if (HAL_GetTick() - lap_start_ms > 300) car_state = CAR_RUNNING;
        break;

    case CAR_RUNNING: {
        float err = line_sensor_error();
        float diff = pid_update(&yaw_pid, err, dt);

        float base = speed_profile(odom.s);
        if (estop) base = 0.0f;
        ramp_to(base, dt);
        motor_set_speed(cur_speed_l + diff, cur_speed_r - diff);

        /* 里程更新 */
        int32_t dl, dr;
        motor_encoder_delta(&dl, &dr);
        float dsl = (float)dl / PULSE_PER_CM;
        float dsr = (float)dr / PULSE_PER_CM;
        odom_update(&odom, (dsl + dsr) * 0.5f, dt);

        /* 完成一圈 */
        if (odom.s > TRACK_L * 0.98f && !estop) {
            car_state = CAR_DONE;
            motor_stop();
            comms_send_done((HAL_GetTick() - lap_start_ms) / 1000.0f);
        }
        break;
    }

    case CAR_DONE:
        motor_stop();
        break;
    }
}

/* ================= 10Hz 遥测 ================= */
static void telemetry_tick(void)
{
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();
    if (now - last < TELE_PERIOD_MS) return;
    last = now;

    const char *st = "IDLE";
    switch (car_state) {
    case CAR_STARTING: st = "STARTING"; break;
    case CAR_RUNNING:  st = "RUNNING";  break;
    case CAR_DONE:     st = "DONE";     break;
    default: break;
    }
    comms_send_telemetry(task_mode, odom.x, odom.y, odom.s, odom.v, st);
}

/* ================= 按键 ================= */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == BTN_PIN) {
        static uint32_t press_ms = 0;
        if (HAL_GPIO_ReadPin(BTN_PORT, BTN_PIN) == GPIO_PIN_SET) {
            press_ms = HAL_GetTick();
        } else {
            uint32_t held = HAL_GetTick() - press_ms;
            if (held >= BTN_LONG_MS) {
                /* 长按：切换任务模式（蜂鸣 2 声） */
                task_mode = (task_mode == MODE_DELIVERY) ? MODE_DYN_LAND : MODE_DELIVERY;
                HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_SET);
                HAL_Delay(60);
                HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_RESET);
                HAL_Delay(60);
                HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_SET);
                HAL_Delay(60);
                HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_RESET);
            } else if (held > 50 && car_state == CAR_IDLE) {
                /* 短按：启动（蜂鸣 1 声） */
                car_state = CAR_STARTING;
                uav_start_sent = 0;
                HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_SET);
                HAL_Delay(80);
                HAL_GPIO_WritePin(BUZ_PORT, BUZ_PIN, GPIO_PIN_RESET);
            }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        comms_rx_byte((uint8_t)(huart->Instance->DR & 0xFF));
    }
}

/* ================= CubeMX 初始化（可直接替换为生成的 MX_* 函数） ================= */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void GPIO_Init_App(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* 电机 PWM 与方向 */
    g.Mode = GPIO_MODE_AF_PP; g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = L_PWM_PIN | R_PWM_PIN;
    HAL_GPIO_Init(GPIOA, &g);

    g.Mode = GPIO_MODE_OUTPUT_PP; g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = L_DIR1_PIN | L_DIR2_PIN | R_DIR1_PIN | R_DIR2_PIN | BUZ_PIN;
    HAL_GPIO_Init(GPIOA, &g);

    /* 灰度输入：上拉 */
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP; g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 |
            GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &g);

    /* 按键：下拉 + EXTI 上升/下降沿 */
    g.Mode = GPIO_MODE_IT_RISING_FALLING; g.Pull = GPIO_PULLDOWN;
    g.Pin = BTN_PIN;
    HAL_GPIO_Init(GPIOA, &g);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    /* LED */
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Pin = LED_PIN;
    HAL_GPIO_Init(GPIOC, &g);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
}

static void TIM2_PWM_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 72 - 1;      /* 1MHz 计数 */
    htim2.Init.Period = 1000 - 1;       /* 1kHz PWM */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_4);
}

static void TIM3_Encoder_Init(TIM_HandleTypeDef *htim)
{
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim->Instance = TIM3;
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 0xFFFF;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(htim, &(TIM_Encoder_InitTypeDef){
        .EncoderMode = TIM_ENCODERMODE_TI12,
        .IC1Polarity = TIM_ICPOLARITY_RISING,
        .IC1Selection = TIM_ICSELECTION_DIRECTTI,
        .IC1Prescaler = TIM_ICPSC_DIV1,
        .IC1Filter = 0,
        .IC2Polarity = TIM_ICPOLARITY_RISING,
        .IC2Selection = TIM_ICSELECTION_DIRECTTI,
        .IC2Prescaler = TIM_ICPSC_DIV1,
        .IC2Filter = 0 });
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

static void TIM4_Encoder_Init(TIM_HandleTypeDef *htim)
{
    __HAL_RCC_TIM4_CLK_ENABLE();
    htim->Instance = TIM4;
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 0xFFFF;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Encoder_Init(htim, &(TIM_Encoder_InitTypeDef){
        .EncoderMode = TIM_ENCODERMODE_TI12,
        .IC1Polarity = TIM_ICPOLARITY_RISING,
        .IC1Selection = TIM_ICSELECTION_DIRECTTI,
        .IC1Prescaler = TIM_ICPSC_DIV1,
        .IC1Filter = 0,
        .IC2Polarity = TIM_ICPOLARITY_RISING,
        .IC2Selection = TIM_ICSELECTION_DIRECTTI,
        .IC2Prescaler = TIM_ICPSC_DIV1,
        .IC2Filter = 0 });
    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
}

static void UART1_Init(void)
{
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_10;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &g);

    huart1.Instance = USART1;
    huart1.Init.BaudRate = UART_BAUD;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init_App();
    TIM2_PWM_Init();
    TIM3_Encoder_Init(&htim3);
    TIM4_Encoder_Init(&htim4);
    UART1_Init();

    line_sensor_init();
    motor_init();
    comms_init();
    pid_init(&yaw_pid, PID_KP, PID_KI, PID_KD, PID_ILIM, PID_OUTLIM);
    odom_reset(&odom);
    cur_speed_l = cur_speed_r = 0.0f;
    uav_start_sent = 0;
    estop = 0;

    while (1) {
        control_tick();
        telemetry_tick();
        HAL_Delay(2);   /* 控制循环按 CONTROL_HZ 由 control_tick 内部节拍约束 */
    }
}

void Error_Handler(void)
{
    while (1) { }
}
