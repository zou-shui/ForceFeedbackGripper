#pragma once

#include "PID.hpp"

namespace Control
{
    static PID posPID(0.1f, 0.05f, 0.001f, -1.0f, 1.0f);
    static PID currPID(1.0f, 0.05f, 0.02f, -1.0f, 1.0f);

    float controlStep(float pos_fb, float pos_ref, float dt, float curr_fb)
    {
        float curr_ref = posPID.update(pos_ref, pos_fb, dt);
        float pwm_duty = currPID.update(curr_ref, curr_fb, dt);
        return pwm_duty;
    }
} // namespace Control
