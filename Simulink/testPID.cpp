#include <iostream>
#include "PIDController.hpp"

int main() {
    PIDController pid(1.0, 0.1, 0.05, 0.01);  // Kp, Ki, Kd, dt
    double measurement = 0.0;
    double setpoint = 10.0;

    for (int i = 0; i < 100; ++i) {
        double control = pid.update(setpoint, measurement);
        measurement += control * 0.01;  // basic physics: integrate control signal
        std::cout << "Time: " << i * 0.01
                  << "s | Output: " << measurement
                  << " | Control: " << control << std::endl;
    }

    return 0;
}
