clear all;
clc;
close all;

% Global variables
global P_now P_future P_offset Kb dt_preview i_num;
global a_error b_error a_command b_command b_clamp b_zero a_threshold;
global v_reference v_actual v_preview a_reference b_reference dt1 dt2;
    
% Misc settings
b_clamp = 15;   % brake value to keep car stopped
                % the exact value doesn't matter, was chosen to match data   
b_zero = 3;     % recorded data has nonzero value when brake is not pressed
                % this offset to the brake value was approximated from data

% Initial variable values
i_num = 0;
P_now = 0.3986;
P_future = 2.2262;
P_offset = 0.3576;
dt_preview = 2.9616;
Kb = 0.8566;
a_threshold = 3.2292;

% Import data
SHEET1 = xlsread('data1.xlsx','sheet1','A2:C34042');
SHEET2 = xlsread('data1.xlsx','sheet2','A2:D141398');

t1 = SHEET1(:,1);
dt1 = t1(2) - t1(1);
v1_ref = SHEET1(:,2);
v1_dyno = SHEET1(:,3);

t2 = SHEET2(:,1);
dt2 = t2(2) - t2(1);
v2_dyno = SHEET2(:,2);
a2_ref = SHEET2(:,3);
b2_ref = SHEET2(:,4);

% Make signals for workspace
a_reference = [t2, a2_ref];
b_reference = [t2, b2_ref];
v_reference = [t1 + 17.0, v1_ref];
v_actual = [t2, v2_dyno];

% Make preview signal
n_prevframes = ceil(dt_preview/dt1);
v_preview = v_reference;
v_preview(1:end-n_prevframes,2) = v_reference(n_prevframes:end-1,2);
    
% Run simulation once
sim('Version14_2019.slx');

% Initial state vector
x0 = [P_now, P_future, P_offset, Kb, dt_preview, a_threshold];

% Linear equality constraints (constraint on linear combo of states)
Aeq = [];
Beq = [];

% Linear inequality constraints (set boundaries of parameter values)
% Ax <= B

A = [-1, 0, 0, 0, 0, 0; ...   % P_now lower
      1, 0, 0, 0, 0, 0; ...   % P_now upper
      0,-1, 0, 0, 0, 0; ...   % P_future lower limit
      0, 1, 0, 0, 0, 0; ...   % P_future upper limit
      0, 0,-1, 0, 0, 0; ...   % P_offset lower limit
      0, 0, 1, 0, 0, 0; ...   % P_offset upper limit
      0, 0, 0,-1, 0, 0; ...   % Kb lower limit
      0, 0, 0, 1, 0, 0; ...   % Kb upper limit
      0, 0, 0, 0,-1, 0; ...   % dt_preview lower limit
      0, 0, 0, 0, 1, 0; ...   % dt_preview upper limit
      0, 0, 0, 0, 0,-1; ...   % a_threshold lower limit
      0, 0, 0, 0, 0, 1]; ...  % a_threshold upper limit

% "Small" and "large" parameter values for optimization boundaries
d = 0.001;
D = 5;

B = [-d; D; ...
     -d; D; ...
     -d; D; ...
     -d; D; ...
     -d; D; ...
     -d; D];

% Set options and call optimizer
options = optimset('Algorithm', 'active-set', 'MaxIter', 500, ...
                   'Display', 'off', 'MaxFunEvals', 500, 'TolFun', 1e-11);
[x, fVal, exitFlag] = fmincon(@ComputeRMSE, ...
                              x0, A, B, Aeq, Beq, [], [], [], options);

disp('Final values');
fprintf('\tP_now\tP_future\tP_offset\tKb\tdt_preview\ta_threshold\n')
disp(x);


function [RMSerror] = ComputeRMSE(x)
    global P_now P_future P_offset Kb dt_preview i_num;
    global a_error b_error a_command b_command b_clamp b_zero a_threshold
    global v_reference v_actual v_preview a_reference b_reference dt1 dt2;

    % Increment # of function evals
    i_num = i_num + 1;

    % Unpack states
    P_now = x(1);
    P_future = x(2);
    P_offset = x(3);
    Kb = x(4);
    dt_preview = x(5);
    a_threshold = x(6);

    % Make previewed reference speed
    n_prevframes = ceil(dt_preview/dt1);
    v_preview = v_reference;
    v_preview(1:end-n_prevframes,2) = v_reference(n_prevframes:end-1,2);

    % Run simulation
    res = sim('Version14_2019');
    
    % Calculate error
    sim_accel_diff = res.a_error.Data;
    sim_brake_diff = res.b_error.Data;
    RMSerror = sqrt(mean(sim_accel_diff.^2)) ...
               + sqrt(mean(sim_brake_diff.^2));         
    
    fprintf('\t i_num \t RMSE \n');
    disp([i_num, RMSerror]);
    fprintf('\tP_now\tP_future\tP_offset\tKb\tdt_preview\ta_threshold\n');
    disp(x);
end


