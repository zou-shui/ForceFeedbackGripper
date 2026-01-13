#include <Arduino.h>
#include "HAL.h"
#include "control.hpp"
#include <Preferences.h>

Preferences prefs;

// PID 参数结构体
struct PIDParam
{
  float kp;
  float ki;
  float kd;
};

// 两个环的参数
PIDParam posParam;
PIDParam currParam;
// 互斥锁（防止串口任务和控制任务同时改 PID）
SemaphoreHandle_t xPIDMutex = NULL;

#define MOTOR_PWM_DUTY 0.9f // 电机PWM占空比
#define MOTOR_RUN_TIME 500  // 电机运行时间，单位ms
#define SYNC_GAIN 0.01f     // 双电机同步控制增益

float Position_ref = 0.0f;
SemaphoreHandle_t xPositionMutex = NULL; // 互斥锁保护Position_ref

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

    // 读取目标位置时使用互斥锁保护
    float pos_ref_local = 0.0f;
    if (xSemaphoreTake(xPositionMutex, portMAX_DELAY) == pdTRUE)
    {
      pos_ref_local = Position_ref;
      xSemaphoreGive(xPositionMutex);
    }

    float base = Control::controlStep(
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

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// 测试夹爪同步运行（使用vTaskDelay替代delay）
void gripper_sync_run(float base_duty, int total_ms)
{
  unsigned long start = millis();

  while (millis() - start < total_ms)
  {
    float a = angleA_read();
    float b = angleB_read();

    float diff = a - b;
    float sync = -SYNC_GAIN * diff;

    float dutyA = constrain(base_duty + sync, -1.0f, 1.0f);
    float dutyB = constrain(base_duty - sync, -1.0f, 1.0f);

    motorA_set_pwm(dutyA);
    motorB_set_pwm(dutyB);

    vTaskDelay(pdMS_TO_TICKS(1)); // 使用RTOS延时
  }

  motor_stop(); // 结束时停下
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

        float value = val.toFloat();

        if (xSemaphoreTake(xPIDMutex, portMAX_DELAY) == pdTRUE)
        {
          bool updated = true;

          // ---- 位置环 ----
          if (key == "posP")
            posParam.kp = value;
          else if (key == "posI")
            posParam.ki = value;
          else if (key == "posD")
            posParam.kd = value;

          // ---- 电流环 ----
          else if (key == "currP")
            currParam.kp = value;
          else if (key == "currI")
            currParam.ki = value;
          else if (key == "currD")
            currParam.kd = value;
          else
            updated = false;

          if (updated)
          {
            // 立即更新 PID 对象（关键）
            Control::posPID.setParam(
                posParam.kp,
                posParam.ki,
                posParam.kd);

            Control::currPID.setParam(
                currParam.kp,
                currParam.ki,
                currParam.kd);

            Serial.print("PID 已更新: ");
            Serial.print(key);
            Serial.print(" = ");
            Serial.println(value, 4);
          }

          xSemaphoreGive(xPIDMutex);
        }
      }
      else if (msg == "pid save")
      {
        prefs.begin("pid", false);

        prefs.putFloat("pos_kp", posParam.kp);
        prefs.putFloat("pos_ki", posParam.ki);
        prefs.putFloat("pos_kd", posParam.kd);

        prefs.putFloat("curr_kp", currParam.kp);
        prefs.putFloat("curr_ki", currParam.ki);
        prefs.putFloat("curr_kd", currParam.kd);

        prefs.end();

        Serial.println("PID 参数已保存到 NVS");
      }
      else if (msg == "pid show")
      {
        Serial.printf("POS: kp=%.3f ki=%.3f kd=%.3f\n",
                      posParam.kp, posParam.ki, posParam.kd);
        Serial.printf("CUR: kp=%.3f ki=%.3f kd=%.3f\n",
                      currParam.kp, currParam.ki, currParam.kd);
      }

      else if (msg == "A1")
      {
        motorA_set_pwm(MOTOR_PWM_DUTY);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_RUN_TIME));
        motor_stop();
        Serial.println("A1高电平执行完毕");
      }
      else if (msg == "A2")
      {
        motorA_set_pwm(-MOTOR_PWM_DUTY);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_RUN_TIME));
        motor_stop();
        Serial.println("A2高电平执行完毕");
      }
      else if (msg == "B1")
      {
        motorB_set_pwm(MOTOR_PWM_DUTY);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_RUN_TIME));
        motor_stop();
        Serial.println("B1高电平执行完毕");
      }
      else if (msg == "B2")
      {
        motorB_set_pwm(-MOTOR_PWM_DUTY);
        vTaskDelay(pdMS_TO_TICKS(MOTOR_RUN_TIME));
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

    vTaskDelay(pdMS_TO_TICKS(10)); // 防止任务占用过多CPU
  }
}

void loadPIDFromNVS()
{
  prefs.begin("pid", true); // 只读

  posParam.kp = prefs.getFloat("pos_kp", 0.5f);
  posParam.ki = prefs.getFloat("pos_ki", 0.5f);
  posParam.kd = prefs.getFloat("pos_kd", 0.01f);

  currParam.kp = prefs.getFloat("curr_kp", 0.5f);
  currParam.ki = prefs.getFloat("curr_ki", 0.05f);
  currParam.kd = prefs.getFloat("curr_kd", 0.02f);

  prefs.end();

  Serial.println("PID 参数已从 NVS 读取");
}

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 串口双向通信已启动~");

  xPositionMutex = xSemaphoreCreateMutex();
  xPIDMutex = xSemaphoreCreateMutex();

  loadPIDFromNVS();

  // 置 PID 参数
  Control::posPID.setParam(posParam.kp, posParam.ki, posParam.kd);
  Control::currPID.setParam(currParam.kp, currParam.ki, currParam.kd);

  // 创建互斥锁
  xPositionMutex = xSemaphoreCreateMutex();
  if (xPositionMutex == NULL)
  {
    Serial.println("互斥锁创建失败！");
  }

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
      3, // 提高控制任务优先级
      NULL,
      0);
  xTaskCreatePinnedToCore(
      serial_command_task,
      "SerialTask",
      4096,
      NULL,
      1, // 串口任务优先级最低
      NULL,
      1);
}

void loop()
{
  // loop函数留空，所有处理在RTOS任务中进行
  vTaskDelay(portMAX_DELAY);
}
