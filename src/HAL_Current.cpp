#include "HAL.h"
#include "KalmanFilter.hpp"

// 电流检测ADC引脚初始化
void current_init(void)
{
    // 设置 ADC 量程（≈3.3V）
    analogSetPinAttenuation(14, ADC_11db);
    analogSetPinAttenuation(12, ADC_11db);

  // 初始化电流卡尔曼滤波器（参数可按需要调整）
  // 这里不重置x，以便启动后快速依赖测量收敛
}

// 读取电流值，单位：安培
float current_read(void)
{
    int raw14 = analogRead(14);
    int raw12 = analogRead(12);

  float I = ((raw14 - raw12) * 3.0f) / 4095.0f * 3.3f;

  // 使用静态卡尔曼滤波器平滑电流读数
  static KalmanFilter1D kf(1e-3f, 1e-2f, 0.0f, 1.0f);
  float If = kf.update(I);
  return If;
}

// 电流检测任务，每100ms打印一次电流值，测试用
void current_task(void *pvParameters)
{
  while (1)
  {
    float cur = current_read();
    Serial.print("Current = ");
    Serial.print(cur, 4); // 保留4位小数
    Serial.println(" A");

    vTaskDelay(pdMS_TO_TICKS(100)); // 100ms 一次
  }
}
