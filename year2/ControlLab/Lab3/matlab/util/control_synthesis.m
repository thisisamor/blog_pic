% CONTROL_SYNTHESIS.m
% 
% Controller synthesis with loop shaping on reduced order model +
% verification on full order continuous and discrete models.
%

clc
clear
close all

%% load model
models_dir = fullfile(getAngRateCtrlMainDir(), 'models');

md = load(fullfile(models_dir, 'ANTX_2DOF_DRONE_pitch_model_1dof_doublet.mat'));

S = md.S;
Gq = tf(S);

s=tf('s');

%% reduce model
tau = 0.02;

xi = 0.0836;
mu = 358.5171;
omega_n = 3.4370;

Gq_r = tf(mu, [1 0]);
Gq_r.InputDelay = tau;

figure
bode(Gq, Gq_r, {1,200})
grid
legend('full-order', 'reduced order')
title('Open-loop model')

%% requirements
% omega_d = 10; % rad/s
 omega_d = 30; % rad/s
% omega_d = 40; % rad/s

% phi_M = 60; %deg

% parameters for loop shaping
tau_f = 0.01;
tau_1 = 1/(omega_d * 0.1)
tau_2 = 1/(omega_d * 2)

% % low bandwidth
% bar_mu=50;

% % mid-bandwidth
% bar_mu=100;

% high bandwidth
% bar_mu=150;

bar_mu = omega_d/tau_1

%% loop shaping on reduced order model
%reduced order loop tf
Lq_r= tf([tau_1,1], 1) * tf([tau_2, 1],1) * tf(1, [tau_f, 1]) * tf(1,[1 0 0]) * bar_mu * exp(-tau*s)

h = figure;
margin(Lq_r)
grid
xlim([1 200])

% recover PID gains
a=tau_1+tau_2;
b=tau_1*tau_2;

Ki=bar_mu/mu
Kp=a*Ki-Ki*tau_f
Kd=b*Ki-Kp*tau_f

Rq = pid(Kp,Ki,Kd,tau_f)

%% verification on full order model

Lq = Rq*Gq;

h = figure; 
margin(Lq)
grid
xlim([1 200])

h=figure; 
margin(Lq)
grid
xlim([1 200])
title('Angular rate loop transfer function (inner loop)')

figure
bode(Lq, Lq_r, {1,200})
grid
legend('full-order', 'reduced order')
title('Loop transfer function')

%% closed-loop model

Gq.u = 'M';
Gq.y = 'q';

msum = sumblk('M = Mc + Md');
Rq.y = 'Mc';
error_sum_q = sumblk('e_q = q_0 - q');
Rq.u = 'e_q';

% close the loop
T_rate = connect(Gq, Rq, msum, error_sum_q, {'q_0', 'Md'}, {'q', 'M'});

figure
step(T_rate('q', 'q_0'))
grid

h = figure;
bode(T_rate('q', 'q_0'), {1 200})
grid
title('Angular rate: complementary sensitivity (inner loop)')

%% verification in discrete time
%% discretize model
Sc = S;
dt = 1/250; % the controller runs at 250Hz

Sd = c2d(Sc, dt, 'tustin');
Sd.InputName = 'M';
Sd.OutputName = 'q';

int_d = tf([Sd.Ts 0], [1 -1], Sd.Ts);
G_q_to_theta = int_d;
G_q_to_theta.InputName = 'q';
G_q_to_theta.OutputName = 'theta';

%% discretized controller
K_FF_attitude = 0;
N=1/tau_f;

C_rate = pitchrate_controller_1DOFPID_tf_BE(K_FF_attitude, Kp, Ki, Kd, Sd.Ts, N);

% augment with pitch moment disturbance input
msum = sumblk('M = Mc + Md');
C_rate.OutputName = 'Mc';

%% close the loop (discrete time)
T_rate_d = connect(Sd, G_q_to_theta, C_rate, msum, ...
    {'q_0', 'Md'}, {'q', 'theta', 'M'});


% inner loop (angular rate)
Li_c = -C_rate(2) * Sd; % the controller tf already contains a minus sign (negative feedback)
%Li_c = Ci_attitude*Sd;
figure
margin(Li_c)
grid
xlim([1,200])

figure
bode(T_rate_d('q', 'q_0'))
xlim([1,200])
grid
title('Complementary sensitivity-discrete time')

figure
step(T_rate('q', 'q_0'))
hold on
step(T_rate_d('q', 'q_0'))
grid
legend('continuous', 'discrete')


figure
bode(T_rate('q', 'q_0'), T_rate_d('q', 'q_0'), {1, 200})
grid
legend('continuous', 'discrete')
