% Create a new Simulink model
modelName = 'PIDControlAircraft';
new_system(modelName);
open_system(modelName);

% Set block positions and spacing
x = 30; y = 30; dx = 120; dy = 80;

% Add blocks
add_block('simulink/Sources/Constant', [modelName '/Setpoint'], ...
    'Position', [x y x+30 y+30]);

add_block('simulink/User-Defined Functions/S-Function', [modelName '/PIDControllerS'], ...
    'FunctionName', 'PIDControllerS', ...
    'Position', [x+2*dx y x+2*dx+80 y+60]);

add_block('simulink/Commonly Used Blocks/Gain', [modelName '/Gain_1_I'], ...
    'Gain', '1/5', 'Position', [x+3*dx y x+3*dx+50 y+30]);

add_block('simulink/Continuous/Integrator', [modelName '/Integrator_omega'], ...
    'Position', [x+4*dx y x+4*dx+40 y+30]);

add_block('simulink/Continuous/Integrator', [modelName '/Integrator_theta'], ...
    'Position', [x+5*dx y x+5*dx+40 y+30]);

add_block('simulink/Sinks/Scope', [modelName '/Scope'], ...
    'Position', [x+6*dx y x+6*dx+50 y+30]);

% Connect blocks
add_line(modelName, 'Setpoint/1', 'PIDControllerS/1');
add_line(modelName, 'Integrator_theta/1', 'PIDControllerS/2');
add_line(modelName, 'PIDControllerS/1', 'Gain_1_I/1');
add_line(modelName, 'Gain_1_I/1', 'Integrator_omega/1');
add_line(modelName, 'Integrator_omega/1', 'Integrator_theta/1');
add_line(modelName, 'Integrator_theta/1', 'Scope/1');

% Save the model
save_system(modelName);
disp(['Model "' modelName '" created and saved as .slx']);
