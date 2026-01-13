#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "control.hpp" // Control::posPID

class PIDManager
{
public:
    struct Param
    {
        float kp;
        float ki;
        float kd;
    };

    PIDManager()
    {
        xPIDMutex = xSemaphoreCreateMutex();
    }

    void begin()
    {
        loadFromNVS();
    }

    // 线程安全更新参数
    void setParam(const String &key, float value)
    {
        if (xSemaphoreTake(xPIDMutex, portMAX_DELAY) == pdTRUE)
        {
            bool updated = true;

            if (key == "posP")
                posParam.kp = value;
            else if (key == "posI")
                posParam.ki = value;
            else if (key == "posD")
                posParam.kd = value;
            else if (key == "currP")
                currParam.kp = value;
            else if (key == "currI")
                currParam.ki = value;
            else if (key == "currD")
                currParam.kd = value;
            else
                updated = false;

            if (updated)
                applyToPIDObjects();

            xSemaphoreGive(xPIDMutex);
        }
    }

    // 显示到串口
    void show()
    {
        Serial.printf("POS: kp=%.3f ki=%.3f kd=%.3f\n",
                      posParam.kp, posParam.ki, posParam.kd);
        Serial.printf("CUR: kp=%.3f ki=%.3f kd=%.3f\n",
                      currParam.kp, currParam.ki, currParam.kd);
    }

    // 保存到 NVS
    void saveToNVS()
    {
        prefs.begin("pid", false);
        prefs.putFloat("pos_kp", posParam.kp);
        prefs.putFloat("pos_ki", posParam.ki);
        prefs.putFloat("pos_kd", posParam.kd);
        prefs.putFloat("curr_kp", currParam.kp);
        prefs.putFloat("curr_ki", currParam.ki);
        prefs.putFloat("curr_kd", currParam.kd);
        prefs.end();
        Serial.println("PID 已保存到 NVS");
    }

private:
    Preferences prefs;
    SemaphoreHandle_t xPIDMutex;
    Param posParam{0.5f, 0.5f, 0.01f};
    Param currParam{0.5f, 0.05f, 0.02f};

    void loadFromNVS()
    {
        prefs.begin("pid", true);
        posParam.kp = prefs.getFloat("pos_kp", posParam.kp);
        posParam.ki = prefs.getFloat("pos_ki", posParam.ki);
        posParam.kd = prefs.getFloat("pos_kd", posParam.kd);
        currParam.kp = prefs.getFloat("curr_kp", currParam.kp);
        currParam.ki = prefs.getFloat("curr_ki", currParam.ki);
        currParam.kd = prefs.getFloat("curr_kd", currParam.kd);
        prefs.end();
        Serial.println("PID 参数已从 NVS 读取");
        applyToPIDObjects();
    }

    void applyToPIDObjects()
    {
        // 将参数应用到 Control 模块的 PID 对象
        Control::posPID.setParam(posParam.kp, posParam.ki, posParam.kd);
        Control::currPID.setParam(currParam.kp, currParam.ki, currParam.kd);
    }
};
