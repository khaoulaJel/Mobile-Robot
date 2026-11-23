% REPLAY_CHECK  Does our Python model of the professor's robot equal his Simulink blocks?
%
% Feeds the command stream logged by make_replay.py through
%   * the professor's `input_filter` MATLAB Function (code copied verbatim from robot_mimo_identification.slx), and
%   * a Discrete State-Space block with the identified A, B, C, D (tools/data/identified_plant.json),
% and compares the output velocities with the Python plant. Run from MATLAB:
%   run('<...>/tools/simulink_replay/replay_check.m')     (after: python3 make_replay.py)
% Expected: differences of the order of 1e-16.
here = fileparts(mfilename('fullpath'));
T = readtable(fullfile(here, 'replay.csv'));
P = jsondecode(fileread(fullfile(here, '..', 'data', 'identified_plant.json')));
A = P.A; B = P.B; C = P.C; D = P.D; Ts = P.Ts;
N = height(T);
% exact k*Ts time stamps: an accumulated t = t + Ts drifts by 1e-16 and From Workspace then returns the
% previous sample (a one-sample lag that looks like a model mismatch but is not)
simin = [(0:N-1)' * Ts, T.cmd_v, T.cmd_w];

mdl = 'replay_prof_plant';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
cleanup = onCleanup(@() close_system(mdl, 0));
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', num2str(Ts, '%.15g'), 'StopTime', num2str((N-1)*Ts, '%.15g'));
add_block('simulink/Sources/From Workspace', [mdl '/Cmd'], 'VariableName', 'simin', ...
    'SampleTime', num2str(Ts, '%.15g'), 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value');
add_block('simulink/Signal Routing/Demux', [mdl '/Split'], 'Outputs', '2');
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/input_filter']);
chart = sfroot().find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/input_filter']);
chart.Script = sprintf(['function [v_out, w_out] = input_filter(v_cmd, w_cmd)\n' ...
    'Vmax = 0.3;\nWmax = 2.5;\nVdead = 0.04;\nWdead = 0.2;\n' ...
    'feasible = (abs(v_cmd)/Vmax + abs(w_cmd)/Wmax) <= 1;\n' ...
    'deadband = (abs(v_cmd) <= Vdead) && (abs(w_cmd) <= Wdead);\n' ...
    'if feasible && ~deadband\n    v_out = v_cmd;\n    w_out = w_cmd;\nelse\n    v_out = 0.0;\n    w_out = 0.0;\nend\nend\n']);
add_block('simulink/Signal Routing/Mux', [mdl '/Join'], 'Inputs', '2');
assignin('base', 'A_p', A); assignin('base', 'B_p', B); assignin('base', 'C_p', C); assignin('base', 'D_p', D);
add_block('simulink/Discrete/Discrete State-Space', [mdl '/Plant'], 'A', 'A_p', 'B', 'B_p', 'C', 'C_p', ...
    'D', 'D_p', 'InitialCondition', 'zeros(4,1)', 'SampleTime', num2str(Ts, '%.15g'));
add_block('simulink/Sinks/To Workspace', [mdl '/Out'], 'VariableName', 'yout_sl', 'SaveFormat', 'Array');
add_block('simulink/Sinks/To Workspace', [mdl '/Filtered'], 'VariableName', 'filt_sl', 'SaveFormat', 'Array');
add_line(mdl, 'Cmd/1', 'Split/1');
add_line(mdl, 'Split/1', 'input_filter/1'); add_line(mdl, 'Split/2', 'input_filter/2');
add_line(mdl, 'input_filter/1', 'Join/1'); add_line(mdl, 'input_filter/2', 'Join/2');
add_line(mdl, 'Join/1', 'Plant/1'); add_line(mdl, 'Plant/1', 'Out/1'); add_line(mdl, 'Join/1', 'Filtered/1');

simOut = sim(mdl);
ysl = simOut.get('yout_sl');
fsl = simOut.get('filt_sl');
n = min(size(ysl, 1), N);
dv = max(abs(ysl(1:n, 1) - T.v_out(1:n)));
dw = max(abs(ysl(1:n, 2) - T.w_out(1:n)));
changed = sum(any(abs(fsl(1:n, :) - [T.cmd_v(1:n) T.cmd_w(1:n)]) > 0, 2));
fprintf('samples compared: %d\n', n);
fprintf('max |v_simulink - v_python| = %.3e m/s   (peak v %.4f)\n', dv, max(abs(T.v_out)));
fprintf('max |w_simulink - w_python| = %.3e rad/s (peak w %.4f)\n', dw, max(abs(T.w_out)));
fprintf('commands altered by the professor''s filter: %d of %d samples\n', changed, n);
if n == N && dv < 1e-9 && dw < 1e-9
    disp('RESULT: PASS - the Python plant + filter equal the professor''s Simulink blocks');
else
    error('RESULT: FAIL - Python model and Simulink blocks differ');
end
