#include <Arduino.h>
#include "HAL.h"


//打印信息，测试用
void print_task(void *pvParameters)
{
    while(1)
    {
        float cur = current_read();
        Serial.print(cur, 4); // 保留4位小数
        Serial.print(",");
        Serial.print(angleA_read(), 2);
        Serial.print(",");
        Serial.println(angleB_read(), 2);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 串口双向通信已启动~");

  motor_init();
  current_init();
  angle_init();

  xTaskCreate(
      print_task,   // 任务函数
      "Print_task", // 名字
      4096,         // 栈大小
      NULL,         // 参数
      1,            // 优先级
      NULL          // 任务句柄
  );
}

void loop()
{

  if (Serial.available())
  {
    String msg = Serial.readStringUntil('\n'); // 读一行
    msg.trim();                                // 去掉换行和空格

    if (msg == "A1")
    {
      motorA_set_pwm(0.8);
      delay(50);
      motor_stop();
      Serial.println("A1高电平50ms执行完毕");
    }
    else if (msg == "A2")
    {
      motorA_set_pwm(-0.8);
      delay(50);
      motor_stop();
      Serial.println("A2高电平50ms执行完毕");
    }
    else if (msg == "B1")
    {
      motorB_set_pwm(0.8);
      delay(50);
      motor_stop();
      Serial.println("B1高电平50ms执行完毕");
    }
    else if (msg == "B2")
    {
      motorB_set_pwm(-0.8);
      delay(50);
      motor_stop();
      Serial.println("B2高电平50ms执行完毕");
    }
    else
    {
      Serial.println("未知命令");
    }
  }
}
