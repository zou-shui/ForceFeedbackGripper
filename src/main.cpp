#include <Arduino.h>
#include "HAL.h"

#define MOTOR_PWM_DUTY 0.7f  // 电机PWM占空比
#define MOTOR_RUN_TIME 300   // 电机运行时间，单位ms

//打印信息，测试用
void print_task(void *pvParameters)
{
    while(1)
    {
        float cur = current_read();
        Serial.print(cur, 4); 
        Serial.print(",");
        Serial.print(angleA_read(), 2);
        Serial.print(",");
        Serial.println(angleB_read(), 2);

        delay(10);
    }
}

//测试夹爪同步运行
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

        delay(1); //这里必须有个延时不然运行时要报错，服了  
    }

    motor_stop();   // 结束时停下
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
      motorA_set_pwm(MOTOR_PWM_DUTY);
      delay(MOTOR_RUN_TIME);
      motor_stop();
      Serial.println("A1高电平50ms执行完毕");
    }
    else if (msg == "A2")
    {
      motorA_set_pwm(-MOTOR_PWM_DUTY);
      delay(MOTOR_RUN_TIME);
      motor_stop();
      Serial.println("A2高电平50ms执行完毕");
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
