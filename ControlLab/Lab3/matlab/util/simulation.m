% SIMULATION.m
% 
% Can be used for tuning and validation of 1DOF PID.
% 1DOF PID implementation (input variable: tracking error; derivative
% action is computed on the error, rather than on the measured output)
% The integrator is implemented with the Backward Euler approach.
% 

clc
clear
close all

%% manually tuned
Kd = 0.000268;
Ki = 0.446;
Kp = 0.113;
% Kp_o = 12;
K_FF_attitude = 0;
N=100;

% load log data
% setpoint response
% r_s = load(fullfile(getAngRateCtrlMainDir(), 'data', 'antx_2023-03-13-14-30-04.mat'));
% r_s = load(fullfile(getAngRateCtrlMainDir(), 'data', 'antx_2023-03-13-14-25-43.mat'));
r_s = load(fullfile(getAngRateCtrlMainDir(), 'data', 'antx_2023-03-13-14-22-10.mat'));
T_FRAME_s = [4.5 14];

%% load model
models_dir = fullfile(getAngRateCtrlMainDir(), 'models');

md = load(fullfile(models_dir, 'ANTX_2DOF_DRONE_pitch_model_1dof_doublet.mat'));

S = md.S;

%% open-loop attitude dynamics: discretize model
Sc = S;
dt = 1/250; % the controller runs at 250Hz

Sd = c2d(Sc, dt, 'tustin');
Sd.InputName = 'M';
Sd.OutputName = 'q';

% pitch rate kinematics
int_d = tf([Sd.Ts 0], [1 -1], Sd.Ts);
G_q_to_theta = int_d;
G_q_to_theta.InputName = 'q';
G_q_to_theta.OutputName = 'theta';

%% close the loop
C_rate_BE = pitchrate_controller_1DOFPID_tf_BE(K_FF_attitude, Kp, Ki, Kd, Sd.Ts, N);

% augment with pitch moment disturbance input
msum = sumblk('M = Mc + Md');
C_rate_BE.OutputName = 'Mc';

% close the loop
T_rate_BE = connect(Sd, G_q_to_theta, C_rate_BE, msum, ...
    {'q_0', 'Md'}, {'q', 'theta', 'M'});

% inner loop (angular rate)
Li_c_BE = -C_rate_BE(2) * Sd; % the controller tf already contains a minus sign (negative feedback)

figure
margin(Li_c_BE)
grid
xlim([1,500])

figure
bode(T_rate_BE('q', 'q_0'))
xlim([1,500])
grid

%%
T_rate = T_rate_BE

%% Simulation
% response to angular rate setpoint
r = r_s;
T_FRAME = T_FRAME_s;

% interpolate

tvec = T_FRAME(1):dt:T_FRAME(2);

% q = r.q.value;
% q0 = r.q0.value;
q0i = interp1(r.q0.timestamp, r.q0.value, tvec, 'previous');

figure
stairs(tvec, q0i)

% simulate
% qsim = lsim(T_rate_q, q0i);

u = [q0i', zeros(size(q0i'))];
ysim = lsim(T_rate, u);
qsim = ysim(:,1);
thetasim = ysim(:,2);
Msim = ysim(:,3);

% compare
figure
stairs(r.q0.timestamp, r.q0.value/pi*180, 'r--')
hold on
grid
plot(tvec, qsim/pi*180, 'b', r.q.timestamp, r.q.value/pi*180, 'r')
%plot(tvec, qsim/pi*180)
legend('setpoint', 'simulated', 'experiment')
title('Pitch rate model simulation')
xlabel('time [s]')
ylabel('q [deg/s]')

xlim(T_FRAME)


figure
plot(tvec, thetasim/pi*180, r.theta.timestamp, r.theta.value/pi*180)
%plot(tvec, thetasim/pi*180)
grid
legend('simulated', 'experiment')
xlabel('Time [s]')
ylabel('theta [deg]')
title('Attitude angle')
xlim(T_FRAME)

figure
plot(tvec, Msim, r.M.timestamp, r.M.value)
grid
legend('simulated', 'experiment')
xlabel('Time [s]')
ylabel('M []')
title('Pitch moment')
xlim(T_FRAME)
