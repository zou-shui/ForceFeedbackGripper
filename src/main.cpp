#include <Arduino.h>
#include "HAL.h"
#include "control.hpp"

#define MOTOR_PWM_DUTY 0.9f // 电机PWM占空比
#define MOTOR_RUN_TIME 500  // 电机运行时间，单位ms

float Position_ref = 0.0f;

// 打印信息，测试用
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

    delay(50);
  }
}

void control_task(void *pvParameters)
{
  const float dt = 0.01f; // 10ms

  while (1)
  {
    float a = angleA_read();
    float b = angleB_read();

    float pos_fb = (a + b) / 2.0f;  // 夹爪当前开度
    float curr_fb = current_read(); // 当前电流反馈

    float base = Control::controlStep(
        pos_fb,
        Position_ref,
        dt,
        curr_fb);

    // 同步修正
    float diff = a - b;
    float sync = -0.01f * diff;

    float dutyA = constrain(base + sync, -1.0f, 1.0f);
    float dutyB = constrain(base - sync, -1.0f, 1.0f);

    motorA_set_pwm(dutyA);
    motorB_set_pwm(dutyB);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// 测试夹爪同步运行
void gripper_sync_run(float base_duty, int total_ms)
{
  unsigned long start = millis();

  while (millis() - start < total_ms)
  {
    float a = angleA_read();
    float b = angleB_read();

    float diff = a - b;
    float sync = -0.01 * diff;

    float dutyA = constrain(base_duty + sync, -1.0f, 1.0f);
    float dutyB = constrain(base_duty - sync, -1.0f, 1.0f);

    motorA_set_pwm(dutyA);
    motorB_set_pwm(dutyB);

    delay(1); // 这里必须有个延时不然运行时要报错，服了
  }

  motor_stop(); // 结束时停下
}

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 串口双向通信已启动~");

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
      0             // 核心ID
  );
  xTaskCreatePinnedToCore(
      control_task,
      "ControlTask",
      4096,
      NULL,
      1,
      NULL,
      0);
}

void loop()
{

  if (Serial.available())
  {
    String msg = Serial.readStringUntil('\n'); // 读一行
    msg.trim();                                // 去掉换行和空格

    if (msg.startsWith("P:"))
    {
      String num = msg.substring(2); // 取冒号后面的部分
      Position_ref = num.toFloat();  // 转成 float

      // Serial.print("接收到P值 = ");
      // Serial.println(Position_ref, 3);
    }
    else if (msg == "A1")
    {
      motorA_set_pwm(MOTOR_PWM_DUTY);
      delay(MOTOR_RUN_TIME);
      motor_stop();
      Serial.println("A1高电平执行完毕");
    }
    else if (msg == "A2")
    {
      motorA_set_pwm(-MOTOR_PWM_DUTY);
      delay(MOTOR_RUN_TIME);
      motor_stop();
      Serial.println("A2高电平执行完毕");
    }
    else if (msg == "B1")
    {
      motorB_set_pwm(MOTOR_PWM_DUTY);
      delay(MOTOR_RUN_TIME);
      motor_stop();
      Serial.println("B1高电平执行完毕");
    }
    else if (msg == "B2")
    {
      motorB_set_pwm(-MOTOR_PWM_DUTY);
      delay(MOTOR_RUN_TIME);
      motor_stop();
      Serial.println("B2高电平执行完毕");
    }
    else if (msg == "M1")
    {
      gripper_sync_run(MOTOR_PWM_DUTY, MOTOR_RUN_TIME);
      Serial.println("夹紧执行完毕");
    }
    else if (msg == "M2")
    {
      gripper_sync_run(-MOTOR_PWM_DUTY, MOTOR_RUN_TIME);
      Serial.println("放松执行完毕");
    }
    else
    {
      Serial.println("未知命令");
    }
  }
}
