#include <Arduino.h>
#include "HAL.h"
#include "PIDManager.hpp"

PIDManager pidManager;

#define SYNC_GAIN 0.01f // 双电机同步控制增益

float Position_ref = 0.0f;
SemaphoreHandle_t xPositionMutex = NULL; // 互斥锁保护Position_ref

// 打印信息
void print_task(void *pvParameters)
{
  while (1)
  {
    float cur = current_read();
    Serial.print(cur, 4);
    Serial.print(",");
    Serial.print(angleA_read(), 2);
    Serial.print(",");
    Serial.println(angleB_read(), 2);

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void control_task(void *pvParameters)
{
  const float dt = 0.001f;

  while (1)
  {
    float a = angleA_read();
    float b = angleB_read();

    float pos_fb = (a + b) / 2.0f;  // 夹爪当前开度
    float curr_fb = current_read(); // 当前电流反馈

    // 读取目标位置时使用互斥锁保护
    float pos_ref_local = 0.0f;
    if (xSemaphoreTake(xPositionMutex, portMAX_DELAY) == pdTRUE)
    {
      pos_ref_local = Position_ref;
      xSemaphoreGive(xPositionMutex);
    }

    float base = Gripper::gripperStep(
        pos_fb,
        pos_ref_local,
        dt,
        curr_fb);

    // 同步修正
    float diff = a - b;
    float sync = -SYNC_GAIN * diff;

    float dutyA = constrain(base + sync, -1.0f, 1.0f);
    float dutyB = constrain(base - sync, -1.0f, 1.0f);

    motorA_set_pwm(dutyA);
    motorB_set_pwm(dutyB);

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// 串口命令处理任务
void serial_command_task(void *pvParameters)
{
  while (1)
  {
    if (Serial.available())
    {
      String msg = Serial.readStringUntil('\n'); // 读一行
      msg.trim();                                // 去掉换行和空格

      if (msg.startsWith("P:"))
      {
        String num = msg.substring(2); // 取冒号后面的部分
        float new_pos = num.toFloat(); // 转成 float

        // 更新目标位置时使用互斥锁保护
        if (xSemaphoreTake(xPositionMutex, portMAX_DELAY) == pdTRUE)
        {
          Position_ref = new_pos;
          xSemaphoreGive(xPositionMutex);
        }
      }
      else if (msg.indexOf(':') > 0)
      {
        int idx = msg.indexOf(':');
        String key = msg.substring(0, idx);
        String val = msg.substring(idx + 1);
        pidManager.setParam(key, val.toFloat());
        Serial.println("PID参数已更新");
      }
      else if (msg == "pid save")
      {
        pidManager.saveToNVS();
      }
      else if (msg == "pid show")
      {
        pidManager.show();
      }
      else
      {
        Serial.println("未知命令");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // 防止任务占用过多CPU
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 串口双向通信已启动~");

  xPositionMutex = xSemaphoreCreateMutex();

  pidManager.begin();

  motor_init();
  current_init();
  angle_init();

  xTaskCreatePinnedToCore(
      print_task,   // 任务函数
      "Print_task", // 名字
      4096,         // 栈大小
      NULL,         // 参数
      2,            // 优先级
      NULL,         // 任务句柄
      1             // 核心ID
  );
  xTaskCreatePinnedToCore(
      control_task,
      "ControlTask",
      4096,
      NULL,
      3, // 控制任务优先级最高
      NULL,
      1);
  xTaskCreatePinnedToCore(
      serial_command_task,
      "SerialTask",
      4096,
      NULL,
      1, // 串口任务优先级最低
      NULL,
      0);
}

void loop()
{
  // loop函数留空，所有处理在RTOS任务中进行
  vTaskDelay(portMAX_DELAY);
}
