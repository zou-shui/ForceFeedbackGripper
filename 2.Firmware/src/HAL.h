#pragma once

#include <Arduino.h>

/* HAL_Motor 电机控制 */
void motor_init();
void motorA_set_pwm(float duty);
void motorB_set_pwm(float duty);
void motor_stop();

/* HAL_Current 电流检测 */
void current_init(void);
float current_read(void);

/* HAL_Angle 角度检测 */
void angle_init(void);
float angleA_read(void);
float angleB_read(void);
