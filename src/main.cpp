#include <iostream>
#include "PIDController.hpp"

int main() {
    double dt = 0.01; // time step = 10 ms
    PIDController pid(0.5, 0.1, 0.05, dt);

    double setpoint = 1.0; // target value
    double process_value = 0.0; // current system state

    for (int i = 0; i < 500; ++i) {
        double control = pid.update(setpoint, process_value);

        // Simulated system: first-order lag (for demo purposes)
        process_value += control * dt; // naive integration

        std::cout << "Time: " << i * dt 
                  << "s | Output: " << process_value 
                  << " | Control: " << control << std::endl;
    }

    return 0;
}
