#include "HAL.h"
#include "KalmanFilter.hpp"

#define ADC_MAX 4095.0f    // 12位ADC最大值
#define VREF 3.3f          // ADC最大量程
#define POT_VMAX 2.5f      // 电位器供电电压
#define ANGLE_RANGE 360.0f // 角度行程

#define ANGLE_OFFSET 224.0f // 角度偏移量
/*
    角度偏移量ANGLE_OFFSET将夹爪开合至共线时的角度补偿为0°
    夹紧过程中两臂平行时的角度补偿为90°
    这个值需要根据实际机械结构调试获得
*/

#define ANGLE_A 15
#define ANGLE_B 13

// 初始化ADC引脚
void angle_init(void)
{
    analogSetPinAttenuation(ANGLE_A, ADC_11db);
    analogSetPinAttenuation(ANGLE_B, ADC_11db);
}

// 电机A角度读取，单位：度
float angleA_read(void)
{
    int raw = analogRead(ANGLE_A);

    float v = raw / ADC_MAX * VREF;

    // 限制不超过电位器供电
    if (v < 0)
        v = 0;
    if (v > POT_VMAX)
        v = POT_VMAX;

    float angle = v / POT_VMAX * ANGLE_RANGE; // 比例换算成角度

    // 使用静态卡尔曼滤波器平滑角度读数
    static KalmanFilter1D kf(1e-3f, 1e-2f, 0.0f, 1.0f);
    float Af = kf.update(angle);
    return Af - ANGLE_OFFSET; // 减去偏移量
}

// 电机B角度读取，单位：度
float angleB_read(void)
{
    int raw = analogRead(ANGLE_B);

    float v = raw / ADC_MAX * VREF;

    if (v < 0)
        v = 0;
    if (v > POT_VMAX)
        v = POT_VMAX;

    float angle = v / POT_VMAX * ANGLE_RANGE; // 比例换算成角度

    // 使用静态卡尔曼滤波器平滑角度读数
    static KalmanFilter1D kf(1e-3f, 1e-2f, 0.0f, 1.0f);
    float Af = kf.update(angle);

    return Af - ANGLE_OFFSET; // 减去偏移量
}

// 打印角度信息，测试用
void angle_task(void *pvParameters)
{
    while (1)
    {
        Serial.print("AngleA = ");
        Serial.print(angleA_read());
        Serial.print("; AngleB = ");
        Serial.println(angleB_read());

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
