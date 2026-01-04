#include "PID.hpp"

namespace Control {

PID* posPID = new PID(1.0f, 0.1f, 0.05f, -100.0f, 100.0f);
PID* currPID = new PID(0.5f, 0.05f, 0.02f, -50.0f, 50.0f);

float controlStep(float pos_fb, float pos_ref, float dt, float curr_fb) {
    float curr_ref = posPID->update(pos_ref, pos_fb, dt);
    float pwm_duty = currPID->update(curr_ref, curr_fb, dt);
    return pwm_duty;
}

} // namespace control
