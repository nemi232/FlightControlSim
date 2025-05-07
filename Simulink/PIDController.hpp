#pragma once

class PIDController {
public:
    // Constructor with PID gains and sample time
    PIDController(double kp, double ki, double kd, double dt);
    
    // Main update function - returns control output
    double update(double setpoint, double measurement);
    
    // Reset internal state (integral and previous error)
    void reset();
    
    // Getters and setters for PID gains
    double getKp() const { return kp_; }
    double getKi() const { return ki_; }
    double getKd() const { return kd_; }
    
    void setKp(double kp) { kp_ = kp; }
    void setKi(double ki) { ki_ = ki; }
    void setKd(double kd) { kd_ = kd; }

private:
    // PID gains
    double kp_;  // Proportional gain
    double ki_;  // Integral gain
    double kd_;  // Derivative gain
    
    // Sample time
    double dt_;
    
    // Controller state
    double integral_;    // Accumulated error
    double prev_error_;  // Previous error (for derivative)
};