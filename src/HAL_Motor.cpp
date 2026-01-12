#include "HAL.h"

// 电机控制引脚定义
#define M_A1 27 // 置高这个脚，A电机正转
#define M_A2 26
#define M_B1 33 // 置高这个脚，B电机正转
#define M_B2 25

// 电机 PWM 通道定义
#define M_A1_CH 0
#define M_A2_CH 1
#define M_B1_CH 2
#define M_B2_CH 3

// 初始化电机控制 PWM
void motor_init()
{
    // 设置 PWM 参数
    int freq = 20000;   // 20kHz
    int resolution = 8; // 8 位分辨率 (0~255)

    // 电机 A
    ledcSetup(M_A1_CH, freq, resolution);
    ledcAttachPin(M_A1, M_A1_CH);

    ledcSetup(M_A2_CH, freq, resolution);
    ledcAttachPin(M_A2, M_A2_CH);

    // 电机 B
    ledcSetup(M_B1_CH, freq, resolution);
    ledcAttachPin(M_B1, M_B1_CH);

    ledcSetup(M_B2_CH, freq, resolution);
    ledcAttachPin(M_B2, M_B2_CH);

    motor_stop();
}

// 设置电机 A 占空比
// duty -1.0 ~ 1.0
// 不考虑正反转，正值夹紧，负值放松
void motorA_set_pwm(float duty)
{
    duty = constrain(duty, -1.0f, 1.0f);

    if (duty >= 0)
    {
        ledcWrite(M_A1_CH, duty * 255);
        ledcWrite(M_A2_CH, 0);
    }
    else
    {
        ledcWrite(M_A1_CH, 0);
        ledcWrite(M_A2_CH, -duty * 255);
    }
}

// 设置电机 B 占空比
// duty -1.0 ~ 1.0
// 不考虑正反转，正值夹紧，负值放松
void motorB_set_pwm(float duty)
{
    duty = constrain(duty, -1.0f, 1.0f);

    if (duty >= 0)
    {       
        ledcWrite(M_B1_CH, 0);
        ledcWrite(M_B2_CH, duty * 255);

    }
    else
    {
        ledcWrite(M_B1_CH, -duty * 255);
        ledcWrite(M_B2_CH, 0);
    }
}

// 停止
void motor_stop()
{
    ledcWrite(M_A1_CH, 0);
    ledcWrite(M_A2_CH, 0);
    ledcWrite(M_B1_CH, 0);
    ledcWrite(M_B2_CH, 0);
}