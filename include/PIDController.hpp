#pragma once

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double dt);
    double update(double setpoint, double measurement);
    void reset();

private:
    double kp_, ki_, kd_, dt_;
    double integral_;
    double prev_error_;
};
