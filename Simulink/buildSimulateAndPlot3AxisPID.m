% buildSimulateAndPlot3AxisPID.m - FIXED VERSION
% Builds, simulates, and plots 3-axis PID aircraft control
% Using direct Simulink blocks instead of S-function

% Clear workspace
clear;
clc;

modelName = 'PIDControl3Axis';

% Close any existing model with this name
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

% Create a new model
new_system(modelName);
open_system(modelName);

% Define model parameters
axes = {'Pitch', 'Roll', 'Yaw'};
inertia = [5, 4, 6];         % Different inertia for each axis
setpoints = [10, 5, -15];    % Desired angles in degrees
colors = {'r', 'g', 'b'};

% PID gains - tuned for each axis
Kp = [2.0, 1.8, 1.5];   % Proportional gain
Ki = [0.3, 0.25, 0.2];  % Integral gain
Kd = [1.0, 0.8, 0.6];   % Derivative gain

% Set simulation parameters
set_param(modelName, 'StopTime', '20');  % Simulation duration
set_param(modelName, 'Solver', 'ode45'); % Differential equation solver

% Layout parameters
x = 100; y = 100; dx = 150; dy = 200;

%% === Build the model ===
for i = 1:3
    ax = axes{i};
    I = inertia(i);
    sp = setpoints(i);
    
    % Position offset for each axis
    yOffset = y + (i-1) * dy;
    
    % Add setpoint constant block
    add_block('simulink/Sources/Constant', [modelName '/Setpoint_' ax], ...
        'Value', num2str(sp), ...
        'Position', [x, yOffset, x+30, yOffset+30]);
    
    % Add summing junction for error calculation
    add_block('simulink/Math Operations/Sum', [modelName '/Sum_' ax], ...
        'Inputs', '+-', ...  % First input positive, second negative
        'Position', [x+100, yOffset, x+120, yOffset+20]);
    
    % === PID Controller using native Simulink blocks ===
    % P term
    add_block('simulink/Math Operations/Gain', [modelName '/P_' ax], ...
        'Gain', num2str(Kp(i)), ...
        'Position', [x+170, yOffset-40, x+200, yOffset-10]);
    
    % I term
    add_block('simulink/Continuous/Integrator', [modelName '/I_Integrator_' ax], ...
        'Position', [x+170, yOffset, x+200, yOffset+30]);
    
    add_block('simulink/Math Operations/Gain', [modelName '/I_' ax], ...
        'Gain', num2str(Ki(i)), ...
        'Position', [x+220, yOffset, x+250, yOffset+30]);
    
    % D term
    add_block('simulink/Continuous/Derivative', [modelName '/Derivative_' ax], ...
        'Position', [x+170, yOffset+40, x+200, yOffset+70]);
    
    add_block('simulink/Math Operations/Gain', [modelName '/D_' ax], ...
        'Gain', num2str(Kd(i)), ...
        'Position', [x+220, yOffset+40, x+250, yOffset+70]);
    
    % Sum up PID terms
    add_block('simulink/Math Operations/Sum', [modelName '/PID_Sum_' ax], ...
        'Inputs', '+++', ...
        'Position', [x+300, yOffset, x+320, yOffset+20]);
    
    % Add gain block for 1/I (inertia)
    add_block('simulink/Math Operations/Gain', [modelName '/Gain_' ax], ...
        'Gain', ['1/' num2str(I)], ...
        'Position', [x+370, yOffset, x+400, yOffset+30]);
    
    % Add angular acceleration to angular velocity integrator
    add_block('simulink/Continuous/Integrator', [modelName '/Integrator_w_' ax], ...
        'Position', [x+450, yOffset, x+490, yOffset+30]);
    
    % Add angular velocity to angle integrator
    add_block('simulink/Continuous/Integrator', [modelName '/Integrator_theta_' ax], ...
        'Position', [x+550, yOffset, x+590, yOffset+30]);
    
    % Add scope for visualization
    add_block('simulink/Sinks/Scope', [modelName '/Scope_' ax], ...
        'Position', [x+650, yOffset, x+680, yOffset+30]);
    
    % Add to workspace block for data export
    add_block('simulink/Sinks/To Workspace', [modelName '/ToWorkspace_' ax], ...
        'VariableName', ['theta_' lower(ax)], ...
        'SaveFormat', 'Structure With Time', ...
        'Position', [x+650, yOffset+50, x+700, yOffset+80]);
    
    % Connect blocks
    % Forward path - Error to PID
    add_line(modelName, ['Setpoint_' ax '/1'], ['Sum_' ax '/1']);
    add_line(modelName, ['Sum_' ax '/1'], ['P_' ax '/1']);
    add_line(modelName, ['Sum_' ax '/1'], ['I_Integrator_' ax '/1']);
    add_line(modelName, ['Sum_' ax '/1'], ['Derivative_' ax '/1']);
    
    % PID internal connections
    add_line(modelName, ['P_' ax '/1'], ['PID_Sum_' ax '/1']);
    add_line(modelName, ['I_Integrator_' ax '/1'], ['I_' ax '/1']);
    add_line(modelName, ['I_' ax '/1'], ['PID_Sum_' ax '/2']);
    add_line(modelName, ['Derivative_' ax '/1'], ['D_' ax '/1']);
    add_line(modelName, ['D_' ax '/1'], ['PID_Sum_' ax '/3']);
    
    % PID to Plant
    add_line(modelName, ['PID_Sum_' ax '/1'], ['Gain_' ax '/1']);
    add_line(modelName, ['Gain_' ax '/1'], ['Integrator_w_' ax '/1']);
    add_line(modelName, ['Integrator_w_' ax '/1'], ['Integrator_theta_' ax '/1']);
    add_line(modelName, ['Integrator_theta_' ax '/1'], ['Scope_' ax '/1']);
    add_line(modelName, ['Integrator_theta_' ax '/1'], ['ToWorkspace_' ax '/1']);
    
    % Feedback path - CRITICAL!
    add_line(modelName, ['Integrator_theta_' ax '/1'], ['Sum_' ax '/2'], 'autorouting', 'on');
    
    % Set scope to automatically open
    set_param([modelName '/Scope_' ax], 'Open', 'on');
end

% Save the model
save_system(modelName);
disp('✅ Model created and saved successfully.');

%% === Simulate ===
disp('⏳ Running simulation...');
sim_out = sim(modelName);
disp('✅ Simulation completed.');

%% === Plot Results ===
disp('📊 Generating plot...');

figure('Name', 'Aircraft Orientation Control', 'Position', [100, 100, 800, 600]);
hold on;

% Plot the response for each axis
for i = 1:3
    ax = axes{i};
    data = eval(['sim_out.theta_' lower(ax)]);
    
    % Plot the angle response with the appropriate color
    plot(data.time, data.signals.values, colors{i}, 'LineWidth', 1.5, ...
        'DisplayName', [ax ' (SP: ' num2str(setpoints(i)) '°)']);
    
    % Plot setpoint as dashed line with same color
    yline(setpoints(i), [colors{i} '--'], 'LineWidth', 1);
end

% Add labels and legend
xlabel('Time (s)');
ylabel('Angle (degrees)');
title('PID-Controlled Aircraft Orientation');
legend('Location', 'best');
grid on;

% Set axis limits to show all data clearly
axis_max = max(abs(setpoints)) * 1.5;
ylim([-axis_max, axis_max]);
xlim([0, str2double(get_param(modelName, 'StopTime'))]);

disp('✅ Plot generated successfully.');