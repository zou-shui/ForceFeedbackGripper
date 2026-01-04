#include <Arduino.h>

#define M_A1 27   //置高这个脚，电机正转
#define M_A2 26

#define M_B1 33   //置高这个脚，电机正转
#define M_B2 25



void adcTask(void *pvParameters)
{
  // 设置 ADC 量程（≈3.3V）
  analogSetPinAttenuation(12, ADC_11db);
  analogSetPinAttenuation(14, ADC_11db);

  while (1)
  {
    int raw12 = analogRead(12);
    int raw14 = analogRead(14);

    float v12 = (float)raw12 / 4095.0 * 3.3;
    float v14 = (float)raw14 / 4095.0 * 3.3;

    Serial.printf("GPIO14 = %d -> %.3f V\n", raw14, v14);
    Serial.printf("GPIO12 = %d -> %.3f V\n", raw12, v12);  
    Serial.printf("-------------------------\n");

    vTaskDelay(pdMS_TO_TICKS(500));   // 必须要有延时！
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 串口双向通信已启动~");

  pinMode(M_A1, OUTPUT);
  pinMode(M_A2, OUTPUT);
  pinMode(M_B1, OUTPUT);
  pinMode(M_B2, OUTPUT);

  digitalWrite(M_A1, LOW);
  digitalWrite(M_A2, LOW);
  digitalWrite(M_B1, LOW);
  digitalWrite(M_B2, LOW);


}

void loop() {

  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n'); // 读一行
    msg.trim();  // 去掉换行和空格

    
    if (msg == "A1") {
      digitalWrite(M_A1, HIGH);
      delay(50);
      digitalWrite(M_A1, LOW);
      Serial.println("A1高电平50ms执行完毕");
    }
    else if (msg == "A2") {
      digitalWrite(M_A2, HIGH);
      delay(50);
      digitalWrite(M_A2, LOW);
      Serial.println("A2高电平50ms执行完毕");
    }
    else if (msg == "B1") {
      digitalWrite(M_B1, HIGH);
      delay(50);
      digitalWrite(M_B1, LOW);
      Serial.println("B1高电平50ms执行完毕");
    }
    else if (msg == "B2") {
      digitalWrite(M_B2, HIGH);
      delay(50);
      digitalWrite(M_B2, LOW);
      Serial.println("B2高电平50ms执行完毕");
    }
    else {
      Serial.println("未知命令");
    }
  }

}



