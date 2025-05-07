% testPIDModel.m
% Creates a simple test model to isolate and debug the PID controller

clear;
clc;

% Define model name
modelName = 'SimplePIDTest';

% Close any existing model with this name
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

% Create a new model
new_system(modelName);
open_system(modelName);

% Set simulation parameters
set_param(modelName, 'StopTime', '20');
set_param(modelName, 'Solver', 'ode45');

% Layout parameters
x = 100; y = 100; dx = 120;

% === Simple single PID control loop ===

% Add setpoint constant block (10 degrees)
add_block('simulink/Sources/Constant', [modelName '/Setpoint'], ...
    'Value', '10', ...
    'Position', [x, y, x+30, y+30]);

% Add summing junction for error calculation
add_block('simulink/Math Operations/Sum', [modelName '/Sum'], ...
    'Inputs', '+-', ...  % First input positive, second negative
    'Position', [x+dx, y, x+dx+30, y+30]);

% Add simple gain block instead of PID (for testing)
add_block('simulink/Math Operations/Gain', [modelName '/P_Controller'], ...
    'Gain', '5', ... % High proportional gain
    'Position', [x+2*dx, y, x+2*dx+30, y+30]);

% Add gain for 1/Inertia (use 1/5 as in the original code)
add_block('simulink/Math Operations/Gain', [modelName '/Inertia'], ...
    'Gain', '1/5', ...
    'Position', [x+3*dx, y, x+3*dx+30, y+30]);

% Add first integrator (angular velocity)
add_block('simulink/Continuous/Integrator', [modelName '/Integrator_w'], ...
    'Position', [x+4*dx, y, x+4*dx+40, y+30]);

% Add second integrator (angle)
add_block('simulink/Continuous/Integrator', [modelName '/Integrator_theta'], ...
    'Position', [x+5*dx, y, x+5*dx+40, y+30]);

% Add scope
add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
    'Position', [x+6*dx, y, x+6*dx+30, y+30]);

% Add to workspace block
add_block('simulink/Sinks/To Workspace', [modelName '/ToWorkspace'], ...
    'VariableName', 'theta_data', ...
    'SaveFormat', 'Structure With Time', ...
    'Position', [x+6*dx, y+50, x+6*dx+70, y+80]);

% Add debugging displays to view signals
add_block('simulink/Sinks/Display', [modelName '/Display_Error'], ...
    'Position', [x+dx, y+70, x+dx+50, y+100]);
    
add_block('simulink/Sinks/Display', [modelName '/Display_Control'], ...
    'Position', [x+2*dx, y+70, x+2*dx+50, y+100]);
    
add_block('simulink/Sinks/Display', [modelName '/Display_Angle'], ...
    'Position', [x+5*dx, y+70, x+5*dx+50, y+100]);

% Connect blocks
% Forward path
add_line(modelName, 'Setpoint/1', 'Sum/1');
add_line(modelName, 'Sum/1', 'P_Controller/1');
add_line(modelName, 'Sum/1', 'Display_Error/1');
add_line(modelName, 'P_Controller/1', 'Inertia/1');
add_line(modelName, 'P_Controller/1', 'Display_Control/1');
add_line(modelName, 'Inertia/1', 'Integrator_w/1');
add_line(modelName, 'Integrator_w/1', 'Integrator_theta/1');
add_line(modelName, 'Integrator_theta/1', 'Scope/1');
add_line(modelName, 'Integrator_theta/1', 'ToWorkspace/1');
add_line(modelName, 'Integrator_theta/1', 'Display_Angle/1');

% Feedback path - CRITICAL!
add_line(modelName, 'Integrator_theta/1', 'Sum/2', 'autorouting', 'on');

% Set scope to automatically start
set_param([modelName '/Scope'], 'Open', 'on');

% Save the model
save_system(modelName);
disp('✅ Test model created.');

%% Simulate and plot
disp('⏳ Running simulation...');
sim_out = sim(modelName);
disp('✅ Simulation completed.');

%% Plot results
figure('Name', 'Simple PID Test', 'Position', [100, 100, 800, 400]);
hold on;

% Plot the angle 
plot(sim_out.theta_data.time, sim_out.theta_data.signals.values, 'b', 'LineWidth', 1.5);
    
% Plot setpoint as dashed line
yline(10, 'r--', 'LineWidth', 1);

% Add labels
xlabel('Time (s)');
ylabel('Angle (degrees)');
title('Simple PID Test - Does Basic Feedback Work?');
grid on;
legend('Angle', 'Setpoint');

% Check if there's actual dynamics in the response
if max(abs(diff(sim_out.theta_data.signals.values))) < 0.001
    disp('❌ ERROR: Response is flat - no dynamics detected!');
    disp('This indicates a fundamental problem with the Simulink model.');
    disp('Possible causes:');
    disp('1. Wrong feedback connections');
    disp('2. Controller not working correctly');
    disp('3. Simulation settings incorrect');
else
    disp('✅ Response shows dynamics - basic feedback is working!');
end

disp('Check the plot to see the actual response.');