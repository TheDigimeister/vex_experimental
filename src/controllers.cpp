#include <vector>
#include <cmath>
#include <iostream>
#include "lemlib/pid.hpp"
#include "pros/distance.hpp"
#include <random>

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

struct Particle {
    double x, y, theta, weight;
};

constexpr int NUM_PARTICLES = 100;
Particle particles[NUM_PARTICLES];

// Example sensor ports
constexpr int DIST_LEFT_PORT = 7;
constexpr int DIST_CENTER_PORT = 8;
constexpr int DIST_RIGHT_PORT = 9;
pros::Distance dist_left(DIST_LEFT_PORT);
pros::Distance dist_center(DIST_CENTER_PORT);
pros::Distance dist_right(DIST_RIGHT_PORT);

// Initialize particles randomly within a field (example: 0-144 inches, 0-144 inches, 0-2pi)
void init_particles(double field_x = 144, double field_y = 144) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_dist(0, field_x);
    std::uniform_real_distribution<> y_dist(0, field_y);
    std::uniform_real_distribution<> theta_dist(0, 2 * M_PI);
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].x = x_dist(gen);
        particles[i].y = y_dist(gen);
        particles[i].theta = theta_dist(gen);
        particles[i].weight = 1.0 / NUM_PARTICLES;
    }
}

// Simulate expected sensor readings for a particle (stub: user should implement based on field geometry)
void expected_sensor_readings(const Particle& p, double& left, double& center, double& right) {
    // For demo, just use x, y, theta (replace with real field model)
    left = p.x;
    center = p.y;
    right = p.theta;
}

// Update particle weights based on sensor readings
void update_particle_weights() {
    double left_meas = dist_left.get();
    double center_meas = dist_center.get();
    double right_meas = dist_right.get();
    double total_weight = 0;
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        double left_exp, center_exp, right_exp;
        expected_sensor_readings(particles[i], left_exp, center_exp, right_exp);
        // Gaussian likelihood (assume stddev=10 for all sensors)
        double w = exp(-pow(left_meas - left_exp, 2) / 200.0)
                 * exp(-pow(center_meas - center_exp, 2) / 200.0)
                 * exp(-pow(right_meas - right_exp, 2) / 200.0);
        particles[i].weight = w;
        total_weight += w;
    }
    // Normalize
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i].weight /= total_weight;
    }
}

// Resample particles based on weights
void resample_particles() {
    std::vector<Particle> new_particles;
    std::random_device rd;
    std::mt19937 gen(rd());
    // Build weights vector
    std::vector<double> weights(NUM_PARTICLES);
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        weights[i] = particles[i].weight;
    }
    std::discrete_distribution<> d(weights.begin(), weights.end());
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        new_particles.push_back(particles[d(gen)]);
    }
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        particles[i] = new_particles[i];
        particles[i].weight = 1.0 / NUM_PARTICLES;
    }
}

// Estimate robot position as weighted mean
void estimate_position(double& x, double& y, double& theta) {
    x = y = theta = 0;
    for (int i = 0; i < NUM_PARTICLES; ++i) {
        x += particles[i].x * particles[i].weight;
        y += particles[i].y * particles[i].weight;
        theta += particles[i].theta * particles[i].weight;
    }
}

// Example function to run the filter (call in a loop)
void run_particle_filter() {
    update_particle_weights();
    resample_particles();
    double x, y, theta;
    estimate_position(x, y, theta);
    std::cout << "Estimated position: x=" << x << ", y=" << y << ", theta=" << theta << std::endl;
}
