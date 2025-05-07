# 3-Axis Flight Control Simulator

## Overview
A flight control system simulator built with C++ and MATLAB/Simulink that models aircraft dynamics and implements PID controllers for pitch, roll, and yaw axes.

## Features
- Real-time PID control of aircraft orientation
- Individual axis tuning for different moments of inertia
- Dynamic response visualization
- C++ controller implementation with Simulink integration

## Getting Started
1. Ensure MATLAB and Simulink are installed
2. Clone this repository
3. Compile the C++ files:
   ```
   mex PIDControllerS.cpp PIDController.cpp
   ```
4. Run the main script:
   ```
   buildSimulateAndPlot3AxisPID
   ```

## File Structure
- `buildSimulateAndPlot3AxisPID.m` - Main script that builds and runs the Simulink model
- `PIDController.hpp/cpp` - C++ implementation of the PID controller
- `PIDControllerS.cpp` - S-function interface for Simulink integration

## Results
The simulation shows dynamic responses of all three orientation axes with appropriate oscillation and convergence toward setpoints.

## Future Improvements
- Add disturbance modeling
- Implement more sophisticated control algorithms
- Add 3D visualization of aircraft motion
