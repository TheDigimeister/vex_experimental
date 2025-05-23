#include <vector>
#include <cmath>
#include <iostream>
#include "lemlib/pid.hpp"

// Simple 1D Kalman Filter class
typedef struct {
    double x;      // state estimate
    double p;      // estimate covariance
    double q;      // process noise covariance
    double r;      // measurement noise covariance
} KalmanFilter;

void kalman_init(KalmanFilter& kf, double q, double r, double p, double x) {
    kf.q = q;
    kf.r = r;
    kf.p = p;
    kf.x = x;
}

double kalman_update(KalmanFilter& kf, double measurement) {
    // Prediction update
    kf.p += kf.q;
    // Measurement update
    double k = kf.p / (kf.p + kf.r);
    kf.x += k * (measurement - kf.x);
    kf.p *= (1 - k);
    return kf.x;
}

// Test and tune functions for Kalman Filter
void test_kalman() {
    KalmanFilter kf;
    kalman_init(kf, 0.01, 1, 1, 0); // q, r, p, x
    std::vector<double> measurements = {1, 2, 1, 2, 1, 2, 1, 2};
    for (double m : measurements) {
        double estimate = kalman_update(kf, m);
        std::cout << "Measurement: " << m << ", Estimate: " << estimate << std::endl;
    }
}

// Use lemlib::PID for PID controller
// Example test and tune function for lemlib::PID
void test_pid() {
    lemlib::PID pid(1.0f, 0.1f, 0.05f, 0.0f, false); // kP, kI, kD, windupRange, signFlipReset
    float setpoint = 10.0f;
    float measurement = 0.0f;
    float dt = 0.1f;
    for (int i = 0; i < 50; ++i) {
        float error = setpoint - measurement;
        float output = pid.update(error);
        measurement += output * dt; // Simulate system response
        std::cout << "Step: " << i << ", Output: " << output << ", Measurement: " << measurement << std::endl;
    }
}
