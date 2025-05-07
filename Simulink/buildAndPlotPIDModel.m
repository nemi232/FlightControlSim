% buildAndPlotPIDModel.m
% Fully self-contained script for building, simulating, and plotting a
% PID-controlled aircraft angle system using a C++ S-function.

%% === Step 1: Create and Build the Simulink Model ===
modelName = 'PIDControlAircraft';
new_system(modelName);
open_system(modelName);

% Layout positions
x = 30; y = 30; dx = 120; dy = 80;

% Add Blocks
add_block('simulink/Sources/Constant', [modelName '/Setpoint'], ...
    'Value', '10', ...
    'Position', [x y x+30 y+30]);

add_block('simulink/User-Defined Functions/S-Function', [modelName '/PIDControllerS'], ...
    'FunctionName', 'PIDControllerS', ...
    'Position', [x+2*dx y x+2*dx+80 y+60]);

add_block('simulink/Commonly Used Blocks/Gain', [modelName '/Gain_1_I'], ...
    'Gain', '1/5', ...
    'Position', [x+3*dx y x+3*dx+50 y+30]);

add_block('simulink/Continuous/Integrator', [modelName '/Integrator_omega'], ...
    'Position', [x+4*dx y x+4*dx+40 y+30]);

add_block('simulink/Continuous/Integrator', [modelName '/Integrator_theta'], ...
    'Position', [x+5*dx y x+5*dx+40 y+30]);

add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
    'Position', [x+6*dx y x+6*dx+50 y+30]);

% ✅ Add To Workspace block for guaranteed logging
add_block('simulink/Sinks/To Workspace', [modelName '/ToWorkspace'], ...
    'VariableName', 'thetaOut', ...
    'SaveFormat', 'StructureWithTime', ...
    'Position', [x+6*dx+70 y x+6*dx+140 y+30]);

% Connect Blocks
add_line(modelName, 'Setpoint/1', 'PIDControllerS/1');
add_line(modelName, 'Integrator_theta/1', 'PIDControllerS/2');
add_line(modelName, 'PIDControllerS/1', 'Gain_1_I/1');
% Add a Scope to monitor the PID controller output
add_block('simulink/Sinks/Scope', [modelName '/PID_Output_Scope'], ...
    'Position', [x+3*dx-30 y+100 x+3*dx+30 y+130]);

% Tap into the line between PIDControllerS and Gain
% To do that, delete the existing line and replace it with a branch

% Delete the original connection
delete_line(modelName, 'PIDControllerS/1', 'Gain_1_I/1');

% Reconnect with a branch
add_line(modelName, 'PIDControllerS/1', 'Gain_1_I/1');
add_line(modelName, 'PIDControllerS/1', 'PID_Output_Scope/1');

add_line(modelName, 'Gain_1_I/1', 'Integrator_omega/1');
add_line(modelName, 'Integrator_omega/1', 'Integrator_theta/1');
add_line(modelName, 'Integrator_theta/1', 'Scope/1');
add_line(modelName, 'Integrator_theta/1', 'ToWorkspace/1');

% Save the model
save_system(modelName);
disp('✅ Simulink model created with signal logging to "thetaOut".');

%% === Step 2: Simulate the Model ===
simOut = sim(modelName);
disp('✅ Simulation completed.');

%% === Step 3: Plot the Output ===
theta = simOut.get('thetaOut');

figure;
plot(theta.time, theta.signals.values, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Angle (deg)');
title('PID-Controlled Aircraft Angle Response');
grid on;
disp('✅ Plot generated successfully.');
