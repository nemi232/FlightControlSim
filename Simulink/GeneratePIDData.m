% generatePIDData.m
% Simulates a 3-axis PID controller and creates CSV data file

% Clear workspace
clear;
clc;

% Simulation parameters
sim_time = 20;         % Total simulation time in seconds
dt = 0.05;             % Time step in seconds
t = 0:dt:sim_time;     % Time vector
num_steps = length(t); % Number of time steps

% PID controllers for each axis (Pitch, Roll, Yaw)
% Controller parameters
axes = {'Pitch', 'Roll', 'Yaw'};
setpoints = [10, 5, -15];     % Target angles in degrees
inertia = [5, 4, 6];          % Inertia values

% PID gains for each axis - tuned differently for each axis
Kp = [2.0, 1.8, 1.5];
Ki = [0.3, 0.25, 0.2];
Kd = [1.0, 0.8, 0.6];

% Initialize arrays to store results
theta = zeros(num_steps, 3);  % Angles (pitch, roll, yaw)
omega = zeros(num_steps, 3);  % Angular velocities
u = zeros(num_steps, 3);      % Control inputs
error_sum = zeros(1, 3);      % Integral term
prev_error = zeros(1, 3);     % Previous error (for derivative)

% Simulate the system step by step
for i = 2:num_steps
    for axis = 1:3
        % PID control calculations
        error = setpoints(axis) - theta(i-1, axis);
        
        % Calculate PID terms
        P_term = Kp(axis) * error;
        
        % Update integral term with anti-windup
        error_sum(axis) = error_sum(axis) + error * dt;
        I_term = Ki(axis) * error_sum(axis);
        
        % Calculate derivative term
        D_term = Kd(axis) * (error - prev_error(axis)) / dt;
        prev_error(axis) = error;
        
        % Calculate control input
        u(i, axis) = P_term + I_term + D_term;
        
        % System dynamics (double integrator with inertia)
        omega(i, axis) = omega(i-1, axis) + (u(i, axis) / inertia(axis)) * dt;
        theta(i, axis) = theta(i-1, axis) + omega(i, axis) * dt;
        
        % Add some noise/disturbance for realism
        if mod(i, 50) == 0
            % Random disturbance every 50 steps
            theta(i, axis) = theta(i, axis) + randn() * 0.1;
        end
    end
end

% Create data table and export to CSV
data_table = table();
data_table.Time = t';
data_table.Pitch = theta(:, 1);
data_table.Roll = theta(:, 2);
data_table.Yaw = theta(:, 3);
data_table.PitchRate = omega(:, 1);
data_table.RollRate = omega(:, 2);
data_table.YawRate = omega(:, 3);
data_table.PitchControl = u(:, 1);
data_table.RollControl = u(:, 2);
data_table.YawControl = u(:, 3);

% Export to CSV
writetable(data_table, 'aircraft_pid_data.csv');
fprintf('✅ CSV data file "aircraft_pid_data.csv" has been created.\n');

% Plot the results
figure('Position', [100, 100, 1000, 800]);

% Plot angles
subplot(3, 1, 1);
hold on;
plot(t, theta(:, 1), 'r', 'LineWidth', 1.5);
plot(t, theta(:, 2), 'g', 'LineWidth', 1.5);
plot(t, theta(:, 3), 'b', 'LineWidth', 1.5);
% Add setpoint reference lines
yline(setpoints(1), 'r--');
yline(setpoints(2), 'g--');
yline(setpoints(3), 'b--');
grid on;
xlabel('Time (s)');
ylabel('Angle (degrees)');
title('PID-Controlled Aircraft Attitude');
legend('Pitch', 'Roll', 'Yaw');

% Plot angular velocities
subplot(3, 1, 2);
hold on;
plot(t, omega(:, 1), 'r', 'LineWidth', 1.5);
plot(t, omega(:, 2), 'g', 'LineWidth', 1.5);
plot(t, omega(:, 3), 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
title('Angular Rates');
legend('Pitch Rate', 'Roll Rate', 'Yaw Rate');

% Plot control inputs
subplot(3, 1, 3);
hold on;
plot(t, u(:, 1), 'r', 'LineWidth', 1.5);
plot(t, u(:, 2), 'g', 'LineWidth', 1.5);
plot(t, u(:, 3), 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Control Input');
title('Control Signals');
legend('Pitch Control', 'Roll Control', 'Yaw Control');

fprintf('✅ Plots generated.\n');