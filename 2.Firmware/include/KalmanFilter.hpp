#pragma once

// Simple 1D Kalman filter (header-only)
// State: x (estimate), covariance: P
// Tunables: Q (process noise), R (measurement noise)
class KalmanFilter1D {
public:
    KalmanFilter1D(float q = 1e-3f, float r = 1e-2f, float x0 = 0.0f, float p0 = 1.0f)
        : Q(q), R(r), x(x0), P(p0) {}

    // Update with measurement z and return filtered estimate
    float update(float z) {
        // Predict
        P += Q;
        // Innovation
        float S = P + R;
        float K = (S != 0.0f) ? (P / S) : 0.0f;
        // Correct
        x += K * (z - x);
        P *= (1.0f - K);
        return x;
    }

    void reset(float x0, float p0 = 1.0f) {
        x = x0;
        P = p0;
    }

    void setQ(float q) { Q = q; }
    void setR(float r) { R = r; }

    float getX() const { return x; }
    float getP() const { return P; }
    float getQ() const { return Q; }
    float getR() const { return R; }

private:
    float Q; // process noise covariance
    float R; // measurement noise covariance
    float x; // state estimate
    float P; // estimate covariance
};
