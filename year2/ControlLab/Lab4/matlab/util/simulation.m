% SIMULATION.m
%
% Can be used for tuning and validation of attitude angle PID.
%
% Close the inner loop on angular rate and the outer loop on the attitude
% angle, simulate (step attitude setpoints).
% 

clc
clear
close all

%% load model

models_dir = fullfile(getAttCtrlMainDir(), 'models');

md = load(fullfile(models_dir, 'ANTX_2DOF_DRONE_pitch_model_1dof_doublet.mat'));

S = md.S;

%% open-loop attitude dynamics: discretize model
Sc = S;
dt = 1/250; % the controller runs at 250Hz

Sd = c2d(Sc, dt, 'tustin');
Sd.InputName = 'M';
Sd.OutputName = 'q';

% pitch rate kinematics
int_d = tf(Sd.Ts, [1 -1], Sd.Ts);
G_q_to_theta = int_d;
G_q_to_theta.InputName = 'q';
G_q_to_theta.OutputName = 'theta';

%% close attitude loop
Kd = 0.00054;
Ki = 0.251;
Kp = 0.0854;
Kp_o = 12;
K_FF_attitude = 0;
N = 100;

Ci_attitude = pitchrate_controller_1DOFPID_tf_BE(K_FF_attitude, Kp, Ki, Kd, Sd.Ts, N);

Co_attitude = pitchangle_controller(Kp_o, Sd.Ts);
Co_attitude.InputName = {'theta_0', 'theta'};

% close the loop
T_att = connect(Sd, G_q_to_theta, Ci_attitude, Co_attitude, ...
    {'theta_0'}, {'theta', 'q', 'M'});

% inner loop (angular rate)
Li_c = -Ci_attitude(2) * Sd;
figure
margin(Li_c)
% title('Inner loop transfer function')
grid
xlim([1,100])

figure
bode(T_att('theta','theta_0'))
grid
title('Outer closed-loop response')

figure
step(T_att('theta','theta_0'))
grid
title('Outer closed-loop response')

%% Simulation
% response to step attitude setpoints

rval = load(fullfile(getAttCtrlMainDir, 'data', 'X0124_2020-12-21-10-50-57.mat'));

% plot setpoint
h = figure;
stairs(rval.theta0.timestamp, rval.theta0.value/pi*180)
xlim([5 13])
ylim([-12 12])
grid
title('Attitude control: sequence of step setpoints')
ylabel('[deg]')
xlabel('Time [s]')

% interpolate reference over uniform time grid
tvec = rval.theta0.timestamp(1):dt:rval.theta0.timestamp(end);
theta0i = interp1(rval.theta0.timestamp, rval.theta0.value, tvec, 'previous');

% simulate with closed-loop model
ysim = lsim(T_att, theta0i);

thetasim = ysim(:,1);
qsim = ysim(:,2);
Msim = ysim(:,3);

% compare
h = figure;
stairs(rval.theta0.timestamp, rval.theta0.value/pi*180, 'k--');
hold on
plot(tvec, thetasim/pi*180, 'r')
legend('setpoint', 'simulated', 'Location', 'best')
title('Pitch model simulation')
grid
xlabel('time[s]')
ylabel('theta [deg]')

xlim([5 13])

h = figure;
plot(tvec, qsim/pi*180)
legend('simulated', 'Location', 'best')
title('Pitch rate')
grid
xlabel('time[s]')
ylabel('q [deg/s]')

xlim([5 13])

h = figure;
plot(tvec, Msim)
legend('simulated', 'Location', 'best')
title('Pitch moment')
grid
xlabel('time[s]')
ylabel('[]')

xlim([5 13])