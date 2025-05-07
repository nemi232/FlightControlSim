#include "PIDController.hpp"

PIDController::PIDController(double kp, double ki, double kd, double dt)
    : kp_(kp), ki_(ki), kd_(kd), dt_(dt), integral_(0.0), prev_error_(0.0) {}

double PIDController::update(double setpoint, double measurement) {
    // Calculate error
    double error = setpoint - measurement;
    
    // Calculate proportional term
    double proportional = kp_ * error;
    
    // Calculate integral term (with simple anti-windup by limiting)
    integral_ += error * dt_;
    // Limit integral term to prevent windup (-100 to 100 as a reasonable limit)
    if (integral_ > 100.0) integral_ = 100.0;
    if (integral_ < -100.0) integral_ = -100.0;
    double integral = ki_ * integral_;
    
    // Calculate derivative term
    double derivative = kd_ * (error - prev_error_) / dt_;
    prev_error_ = error;
    
    // Calculate control output
    return proportional + integral + derivative;
}

void PIDController::reset() {
    integral_ = 0.0;
    prev_error_ = 0.0;
}