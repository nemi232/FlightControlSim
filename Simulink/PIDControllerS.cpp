#define S_FUNCTION_NAME PIDControllerS
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <stdio.h>

// Debug version - extremely simplified to isolate the problem
// No external dependencies, just implements basic P controller directly

// Sample time
#define DT 0.01

// Simple controller gains (proportional only for debugging)
#define KP 3.0  

static void mdlInitializeSizes(SimStruct *S) {
    ssSetNumSFcnParams(S, 0);
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    // Two input ports: setpoint and measurement
    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 1);  // Setpoint
    ssSetInputPortWidth(S, 1, 1);  // Measurement
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);

    // One output port: control signal
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);

    ssSetNumSampleTimes(S, 1);
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, DT);  // Sample time
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct *S, int_T tid) {
    // Get inputs
    const real_T *setpoint = (const real_T*) ssGetInputPortSignal(S, 0);
    const real_T *measurement = (const real_T*) ssGetInputPortSignal(S, 1);
    
    // Get output
    real_T *control = (real_T*) ssGetOutputPortSignal(S, 0);
    
    // Super simple proportional control only - for debugging
    double error = setpoint[0] - measurement[0];
    control[0] = KP * error;
    
    // Add debug message to check if this function is being called
    // This will appear in the MATLAB command window during simulation
    mexPrintf("PID Debug: Setpoint=%.2f, Measurement=%.2f, Error=%.2f, Control=%.2f\n", 
              setpoint[0], measurement[0], error, control[0]);
}

static void mdlTerminate(SimStruct *S) {
    // Nothing to do
}

#include "simulink.c"