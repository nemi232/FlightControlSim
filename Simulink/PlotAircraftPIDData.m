% plotAircraftPIDData.m
% Loads and plots the aircraft PID data from CSV file

% Clear workspace
clear;
clc;

fprintf('Loading PID data...\n');

% Check if the file exists
if ~exist('aircraft_pid_data.csv', 'file')
    error('Error: aircraft_pid_data.csv not found. Run generatePIDData.m first.');
end

% Load the data from CSV
data = readtable('aircraft_pid_data.csv');

% Extract the data columns
time = data.Time;
pitch = data.Pitch;
roll = data.Roll;
yaw = data.Yaw;
pitch_rate = data.PitchRate;
roll_rate = data.RollRate;
yaw_rate = data.YawRate;
pitch_control = data.PitchControl;
roll_control = data.RollControl;
yaw_control = data.YawControl;

% Set the target setpoints (these match what was used in the simulation)
setpoints = [10, 5, -15];  % Pitch, Roll, Yaw in degrees

% Create a multi-panel figure
figure('Position', [100, 100, 1000, 800]);

% 1. Plot angles
subplot(3, 1, 1);
hold on;
plot(time, pitch, 'r', 'LineWidth', 1.5);
plot(time, roll, 'g', 'LineWidth', 1.5);
plot(time, yaw, 'b', 'LineWidth', 1.5);
% Add setpoint reference lines
yline(setpoints(1), 'r--');
yline(setpoints(2), 'g--');
yline(setpoints(3), 'b--');
grid on;
xlabel('Time (s)');
ylabel('Angle (degrees)');
title('PID-Controlled Aircraft Orientation');
legend({'Pitch', 'Roll', 'Yaw'}, 'Location', 'best');

% 2. Plot angular velocities
subplot(3, 1, 2);
hold on;
plot(time, pitch_rate, 'r', 'LineWidth', 1.5);
plot(time, roll_rate, 'g', 'LineWidth', 1.5);
plot(time, yaw_rate, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
title('Angular Rates');
legend({'Pitch Rate', 'Roll Rate', 'Yaw Rate'}, 'Location', 'best');

% 3. Plot control inputs
subplot(3, 1, 3);
hold on;
plot(time, pitch_control, 'r', 'LineWidth', 1.5);
plot(time, roll_control, 'g', 'LineWidth', 1.5);
plot(time, yaw_control, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Control Input');
title('Control Signals');
legend({'Pitch Control', 'Roll Control', 'Yaw Control'}, 'Location', 'best');

% Create an overall title
sgtitle('Flight Control System - PID Response', 'FontSize', 14);

fprintf('✅ Plots generated.\n');

% Performance analysis
fprintf('\n----- Flight Control Performance Analysis -----\n');

% Calculate settling time (time to reach and stay within 2% of setpoint)
settling_threshold = 0.02;  % 2%
settling_times = zeros(1, 3);
labels = {'Pitch', 'Roll', 'Yaw'};

for i = 1:3
    if i == 1
        signal = pitch;
        target = setpoints(1);
    elseif i == 2
        signal = roll;
        target = setpoints(2);
    else
        signal = yaw;
        target = setpoints(3);
    end
    
    % Calculate error band
    error_band = abs(target) * settling_threshold;
    
    % Find where signal stays within error band
    within_band = abs(signal - target) <= error_band;
    
    % Find the first point where it stays within band
    for j = 2:length(within_band)
        if all(within_band(j:min(j+20, end)))  % Must stay in band for at least 20 samples
            settling_times(i) = time(j);
            break;
        end
    end
    
    fprintf('%s settling time (±2%%): %.2f seconds\n', labels{i}, settling_times(i));
end

% Calculate overshoot
overshoot = zeros(1, 3);
for i = 1:3
    if i == 1
        signal = pitch;
        target = setpoints(1);
    elseif i == 2
        signal = roll;
        target = setpoints(2);
    else
        signal = yaw;
        target = setpoints(3);
    end
    
    if target > 0
        max_val = max(signal);
        overshoot(i) = max(0, (max_val - target) / target * 100);
    else
        min_val = min(signal);
        overshoot(i) = max(0, (target - min_val) / abs(target) * 100);
    end
    
    fprintf('%s maximum overshoot: %.1f%%\n', labels{i}, overshoot(i));
end

fprintf('\nNote: The CSV file contains all simulation data for further analysis.\n');