#include <Arduino.h>
#include "HAL.h"

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 串口双向通信已启动~");

  motor_init();
  current_init();
  angle_init();

  xTaskCreate(
      angle_task,   // 任务函数
      "angle_task", // 名字
      4096,         // 栈大小
      NULL,         // 参数
      1,            // 优先级
      NULL          // 任务句柄
  );
  // xTaskCreate(
  //     current_task,  // 任务函数
  //     "CurrentTask", // 名字
  //     4096,          // 栈大小
  //     NULL,          // 参数
  //     1,             // 优先级
  //     NULL           // 任务句柄
  // );
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
