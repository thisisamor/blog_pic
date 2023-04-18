%SIMULATION.M
% Given the models of angular rate dynamics and velocity dynamics, and the
% controller gains, build the closed-loop system including
% position+velocity control and attitude+angular rate control, simulate and
% evaluate performance (complementary sensitivity bandwidth) of each loop.
% 

clc
clear
close all

%% ANGULAR DYNAMICS
%% mid-bandwidth angular rate controller (wc_q = 30rad/s)
Kd = 5.4e-4;
Ki = 0.251;
Kp = 0.0854;
K_FF_attitude = 0;
N=100;
Kp_theta = 11;

%% load model
models_dir = fullfile(getVelPosCtrlMainDir(), 'models');

md = load(fullfile(models_dir, 'ANTX_2DOF_DRONE_pitch_model_1dof_doublet.mat'));

S = md.S;

%% open-loop attitude dynamics: discretize model
Sc = S;
dt = 1/250; % the controller runs at 250Hz

Sd = c2d(Sc, dt, 'tustin');
Sd.InputName = 'M';
Sd.OutputName = 'q';

int_d = tf([Sd.Ts, 0], [1 -1], Sd.Ts);
G_q_to_theta = int_d;
G_q_to_theta.InputName = 'q';
G_q_to_theta.OutputName = 'theta';

%% inner loop
R_i = pitchrate_controller_1DOFPID_tf_BE(K_FF_attitude, Kp, Ki, Kd, Sd.Ts, N);

% augment with pitch moment disturbance input
msum = sumblk('M = Mc + Md');
R_i.OutputName = 'Mc';

% close the loop
T_i = connect(Sd, G_q_to_theta, R_i, msum, ...
    {'q_0', 'Md'}, {'q', 'theta', 'M'});

% inner loop (angular rate)
L_i = -R_i(2) * Sd; % the controller tf already contains a minus sign (negative feedback)

figure
margin(L_i)
grid
xlim([1,500])

figure
bode(T_i('q', 'q_0'))
xlim([1,500])
grid

figure
step(T_i('q', 'q_0'))
grid

%% verify bandwidth of inner loop
fprintf('bandwidth of angular rate loop (complementary sensitivity) %2.2f [rad/s]\n', bandwidth(T_i('q', 'q_0')))

% NOTE: the bandwidth of T is much higher than wc_q
% This is due to:
% _the bandwidth of the complementary sensitivity is different from
% critical frequency
% _ 1DOF PID implementation (the derivative action is computed on the
% tracking error)

%% outer loop
R_o = tf(Kp_theta);
R_o.InputName = 'e_theta';
R_o.OutputName = 'q_0';

msum_etheta = sumblk('e_theta = theta_0 - theta');

T_o = connect(msum_etheta, R_o, T_i, ...
    {'theta_0', 'Md'}, {'theta', 'q', 'q_0', 'M'});

L_o = R_o * T_i('q', 'q_0') * G_q_to_theta;

figure
bode(T_o('theta', 'theta_0'))
xlim([1,50])
grid
title('Outer complementary sensitivity')

figure
step(T_o('theta', 'theta_0'))
grid

figure
margin(L_o)
xlim([1,50])
grid
% title('Outer loop transfer function')

%% verify bandwidth of outer loop
fprintf('bandwidth of attitude loop (complementary sensitivity) %2.2f [rad/s]\n', bandwidth(T_o('theta', 'theta_0')))

%%
T_att = T_o('theta', 'theta_0');

%% TRANSLATIONAL DYNAMICS
%% load velocity dynamics model
load(fullfile(models_dir, 'ANTX_2DOF_DRONE_theta_to_v_model.mat'))

%% discretize
dt = 1/250; % the controller runs at 250Hz

G_vel_theta_to_vx = c2d(G, dt, 'tustin');
G_vel_theta_to_vx.InputName = 'theta';
G_vel_theta_to_vx.OutputName = 'vx';

%% discrete-time integrator
int_d = tf([dt 0], [1 -1], dt);
G_vx_to_x = int_d;
G_vx_to_x.InputName = 'vx';
G_vx_to_x.OutputName = 'x';

%% attitude reference generator
k_bar = 4.75;
T_bar = 0.4;
mass = 0.374 + 0.160; % drone + cart

eta_t = k_bar * mass / T_bar;

f_dz = -T_bar * eta_t; % [N] desired force in the Z direction

A_r = tf(1/f_dz, 1, dt);
A_r.InputName = 'f_dx';
A_r.OutputName = 'theta_0';

%% velocity control loop
Kd = 0;
Ki = 2.81;
Kp = 2.4;
K_FF_vel = 0;
N=10;

Ci_vel = velocity_controller_tf(K_FF_vel, Kp, Ki, Kd, dt, N);

%% close velocity loop
T_vel = connect(Ci_vel, A_r, T_att, G_vel_theta_to_vx, ...
    {'vx_0'}, {'vx', 'theta', 'theta_0'});

figure
bode(T_vel({'vx'}, {'vx_0'}))
grid
xlim([.1,30])

fprintf('bandwidth of velocity loop (complementary sensitivity) %2.2f [rad/s]\n', bandwidth(T_vel({'vx'}, {'vx_0'})))

Li_vel = -Ci_vel(2) * A_r * T_att * G_vel_theta_to_vx;

Li_vel_approx = -Ci_vel(2) * A_r * G_vel_theta_to_vx;

figure
margin(Li_vel)
grid
xlim([.1,30])

h = figure;
bode(Li_vel_approx)
hold on
bode(Li_vel)
xlim([.1,30])
legend('ideal', 'full order', 'Location', 'best')
grid
title('Loop transfer function (velocity loop)')

h = figure;
step(T_vel('vx', 'vx_0'))
grid

figure
step(T_vel('theta', 'vx_0'))
hold on
grid
step(T_vel('theta_0', 'vx_0'))

%% position control loop
Kp_o = 1.5;

% Co_pos.InputName = {'x_0', 'x'};
% Co_pos.OutputName = 'vx_0';
Co_pos = position_controller(Kp_o, dt);

T_pos = connect(Co_pos, T_vel, G_vx_to_x, ...
    {'x_0'}, {'x', 'vx', 'theta', 'theta_0'});

figure
bode(T_pos('x', 'x_0'))
grid
xlim([.1,30])

fprintf('bandwidth of position loop (complementary sensitivity) %2.2f [rad/s]\n', bandwidth(T_pos('x', 'x_0')))

Lo_pos = -Co_pos(2) * T_vel('vx', 'vx_0') * G_vx_to_x;
Lo_pos_approx = -Co_pos(2) * G_vx_to_x;

figure
margin(Lo_pos)
grid
xlim([.1,30])

h = figure;
bode(Lo_pos_approx)
hold on
bode(Lo_pos)
xlim([.1,30])
legend('ideal', 'full order', 'Location', 'best')
grid
title('Loop transfer function (position loop)')

%% simulation: velocity control
rval = load(fullfile(getVelPosCtrlMainDir, 'data', 'antx_2023-03-16-14-01-14.mat'));

%% extract velocity setpoint
figure
stairs(rval.v0.timestamp, rval.v0.value)
grid

%%
TIME_FRAME = [22 33];

%%
h = figure;
stairs(rval.v0.timestamp, rval.v0.value)
xlim(TIME_FRAME)
ylim([-.3 .3])
grid
title('Velocity control: sequence of doublets')
ylabel('[m/s]')
xlabel('Time [s]')

%% interpolate reference over uniform time grid
tvec = rval.v0.timestamp(1):dt:rval.v0.timestamp(end);
v0i = interp1(rval.v0.timestamp, rval.v0.value, tvec, 'previous');

%% simulate
ysim = lsim(T_vel, v0i);

vx_sim = ysim(:,1);
theta_sim = ysim(:,2);
%theta0_sim = ysim(:,3);

%% plot
h = figure;
hold on
stairs(rval.v0.timestamp, rval.v0.value, 'k--')
plot(tvec, vx_sim, 'r', rval.v.timestamp, rval.v.value, 'b')
grid
xlim(TIME_FRAME)
legend('simulated', 'experiment')
title('Simulation: velocity controller')
ylabel('vx [m/s]')
xlabel('Time [s]')

h = figure;
hold on
plot(tvec, theta_sim/pi*180, rval.theta.timestamp, rval.theta.value/pi*180)
grid
xlim(TIME_FRAME)
legend('simulated', 'experiment')
title('Simulation: velocity controller')
ylabel('theta [deg]')
xlabel('Time [s]')

%% experimental validation: position control
rval = load(fullfile(getVelPosCtrlMainDir, 'data', 'antx_2023-03-16-14-01-14.mat'));

%% extract position estimate and position setpoint

figure
stairs(rval.x0.timestamp, rval.x0.value)
grid

%%
TIME_FRAME = [5 18];

%%
h = figure;
stairs(rval.x0.timestamp, rval.x0.value)
grid
ylim([-.35 .35])
xlim(TIME_FRAME)
ylabel('[m]')
xlabel('Time [s]')
title('Position control: sequence of step setpoints')

%% interpolate reference over uniform time grid
tvec = rval.x0.timestamp(1):dt:rval.x0.timestamp(end);
x0i = interp1(rval.x0.timestamp, rval.x0.value, tvec, 'previous');

%% simulate
ysim = lsim(T_pos, x0i);

x_sim = ysim(:,1);
vx_sim = ysim(:,2);
theta_sim = ysim(:,3);
% theta0_sim = ysim(:,4);

%%
h = figure;
hold on
stairs(rval.x0.timestamp, rval.x0.value, 'k--')
grid
plot(tvec, x_sim, 'r', rval.x.timestamp, rval.x.value, 'b')
xlim(TIME_FRAME)
legend('setpoint', 'simulated', 'Location', 'best')
title('Simulation: position controller')
ylabel('x [m]')
xlabel('Time [s]')

h = figure;
plot(tvec, vx_sim, rval.v.timestamp, rval.v.value)
grid
xlim(TIME_FRAME)
legend('simulated', 'experiment')
title('Experimental validation: position controller')
ylabel('vx [m/s]')
xlabel('Time [s]')

h = figure;
plot(tvec, theta_sim/pi*180, rval.theta.timestamp, rval.theta.value/pi*180)
grid
xlim(TIME_FRAME)
legend('simulated', 'experiment')
title('Simulation: position controller')
ylabel('theta [deg]')
xlabel('Time [s]')
