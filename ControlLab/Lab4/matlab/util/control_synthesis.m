%CONTROL_SYNTHESIS
%
% Controller synthesis with loop shaping + verification
%

clc
clear
close all

%% modeling
% angular rate closed-loop dynamics (see lecture 3)
wc_q = 32.6;
phi_m = 57.8;

% option#1: neglect angular rate dynamics (easier)
T_i_id = tf(1);

% % option#2: use 2nd order approx model of angular rate (closed-loop)
% dynamics (more accurate)
% wn = wc_q;
% xi = phi_m/100;
% Ti_c = tf(wn^2, [1, 2*xi*wn, wn^2]);

figure
bode(T_i_id)

% tf q to theta is modeled as an integrator
Gq_to_theta_id = tf(1, [1 0]);

%% requirements
wc_theta = 1/3 * wc_q

%% tuning
Kp_theta = 11;

L_o_id = Kp_theta * T_i_id * Gq_to_theta_id;

T_o_id= feedback(L_o_id,1);

figure
margin(L_o_id)
grid

figure
step(T_o_id)
grid

%% verification against model of closed-loop angular rate dynamics
%% mid-bandwidth angular rate controller (wc_q = 30rad/s)
Kd = 5.4e-4;
Ki = 0.251;
Kp = 0.0854;
K_FF_attitude = 0;
N=100;

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
bandwidth(T_i('q', 'q_0'))

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
hold on
bode(T_o_id)
xlim([1,50])
grid
legend('full order','ideal')
title('Outer complementary sensitivity')

h = figure;
bode(T_o('theta', 'theta_0'))
xlim([1,50])
grid
title('Attitude: complementary sensitivity (outer loop)')

figure
step(T_o('theta', 'theta_0'))
hold on
step(T_o_id)
grid
legend('full order','ideal')

h = figure;
margin(L_o)
xlim([1,50])
grid
title('Attitude loop transfer function (outer loop)')

figure
margin(L_o)
hold on
margin(L_o_id)
xlim([1,50])
grid
legend('full order','ideal')
title('Outer loop transfer function')

figure
margin(L_o)
grid

figure
step(T_o('q', 'theta_0'))
hold on
step(T_o('q_0', 'theta_0'))
legend('output', 'setpoint')
grid
title('Angular rate response to an attitude step setpoint')

figure
step(T_o('M', 'theta_0'))
grid
title('Control action response to an attitude step setpoint')
