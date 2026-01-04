#include "HAL.h"

// 电流检测ADC引脚初始化
void current_init(void)
{
    // 设置 ADC 量程（≈3.3V）
    analogSetPinAttenuation(14, ADC_11db);
    analogSetPinAttenuation(12, ADC_11db);
}

// 读取电流值，单位：安培
float current_read(void)
{
    int raw14 = analogRead(14);
    int raw12 = analogRead(12);

    float I = ((raw14 - raw12) * 3.0) / 4095.0 * 3.3;
    return I;
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
