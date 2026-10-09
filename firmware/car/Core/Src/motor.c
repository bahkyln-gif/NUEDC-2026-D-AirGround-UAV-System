/**
 * @file    motor.c
 * @brief   电机控制：TIM2 PWM（PA0=左 CH1, PA3=右 CH4）
 *          编码器：TIM3 Encoder（左 PA6/PA7），TIM4 Encoder（右 PB6/PB7）
 */
#include "main.h"
#include "car_config.h"
#include "motor.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* PWM 满量程（ARR 值，CubeMX 配置 20kHz） */
#define PWM_MAX           1000u

static void set_dir(GPIO_TypeDef *p1, uint16_t n1, GPIO_TypeDef *p2, uint16_t n2,
                    uint8_t forward)
{
    HAL_GPIO_WritePin(p1, n1, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(p2, n2, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void motor_init(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

void motor_set_speed(float left_cmps, float right_cmps)
{
    int32_t pwm_l = (int32_t)(left_cmps  * PWM_PER_CMPS);
    int32_t pwm_r = (int32_t)(right_cmps * PWM_PER_CMPS);

    if (pwm_l >  PWM_MAX) pwm_l =  PWM_MAX;
    if (pwm_l < -PWM_MAX) pwm_l = -PWM_MAX;
    if (pwm_r >  PWM_MAX) pwm_r =  PWM_MAX;
    if (pwm_r < -PWM_MAX) pwm_r = -PWM_MAX;

    /* 左轮 */
    if (pwm_l >= 0) set_dir(L_DIR1_PORT, L_DIR1_PIN, L_DIR2_PORT, L_DIR2_PIN, 1);
    else            set_dir(L_DIR1_PORT, L_DIR1_PIN, L_DIR2_PORT, L_DIR2_PIN, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)(pwm_l > 0 ? pwm_l : -pwm_l));

    /* 右轮 */
    if (pwm_r >= 0) set_dir(R_DIR1_PORT, R_DIR1_PIN, R_DIR2_PORT, R_DIR2_PIN, 1);
    else            set_dir(R_DIR1_PORT, R_DIR1_PIN, R_DIR2_PORT, R_DIR2_PIN, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)(pwm_r > 0 ? pwm_r : -pwm_r));
}

void motor_encoder_delta(int32_t *dl, int32_t *dr)
{
    /* TIM3/TIM4 均配置为 Encoder Mode（4 倍频），CNT 为有符号增量 */
    *dl = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    *dr = (int32_t)(int16_t)__HAL_TIM_GET_COUNTER(&htim4);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

void motor_stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
    set_dir(L_DIR1_PORT, L_DIR1_PIN, L_DIR2_PORT, L_DIR2_PIN, 1);
    set_dir(R_DIR1_PORT, R_DIR1_PIN, R_DIR2_PORT, R_DIR2_PIN, 1);
}
