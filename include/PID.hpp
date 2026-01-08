#pragma once

namespace Control {

class PID {
public:
    PID(float kp, float ki, float kd, float out_min, float out_max)
        : kp_(kp), ki_(ki), kd_(kd),
          out_min_(out_min), out_max_(out_max),
          prev_err_(0.0f), integral_(0.0f) {}

    float update(float ref, float fb, float dt) {
        float err = ref - fb;

        integral_ += err * dt;
        float derivative = (err - prev_err_) / dt;

        float out = kp_ * err
                  + ki_ * integral_
                  + kd_ * derivative;

        // 输出限幅
        if (out > out_max_) out = out_max_;
        if (out < out_min_) out = out_min_;

        prev_err_ = err;
        return out;
    }

    void reset() {
        integral_ = 0.0f;
        prev_err_ = 0.0f;
    }

private:
    float kp_, ki_, kd_;
    float out_min_, out_max_;
    float prev_err_;
    float integral_;
};

} // namespace control