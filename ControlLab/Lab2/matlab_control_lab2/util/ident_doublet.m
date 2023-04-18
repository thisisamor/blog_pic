% IDENT_DOUBLET.m
% 
% Identify model of the pitch angular rate dynamics based on experimental
% data collected on test-bed (1DOF mode) in open-loop conditions.
% Carry out verification and validation.
% 
% Algorithm: three-steps procedure for 2nd order model (see doc): forced
% response
% Model structure: LTI, grey-box
% Experiment: open-loop, on test-bed in 1DOF mode.
% Signal: doublet
% Validation data:
% _ different ident experiments (repetitions of same signal)

clc
clear
close all

%% use ROS logged data
rr = load(fullfile(getIdentMainDir, 'data', 'antx_2023-03-10-14-08-45.mat'));

%%
figure
stairs(rr.M.timestamp, rr.M.value)
grid
xlabel('time [s]')
ylabel('pitch moment []')

dt = 1/100; % [s] telemetry sampling rate

%% extract data
TIME_FRAME = [25.25 26.50];

t_vec = TIME_FRAME(1):dt:TIME_FRAME(2);

u = interp1(rr.M.timestamp, rr.M.value, t_vec, 'previous');
y = interp1(rr.q.timestamp, rr.q.value, t_vec);

ind_t_init = find(abs(diff(u)) > eps, 1);

% remove bias
du = u - mean(u(ind_t_init));
dy = y - mean(y(ind_t_init));

dataset_1.t = t_vec;
dataset_1.u = u;
dataset_1.y = y;
dataset_1.du = du;
dataset_1.dy = dy;

%%
TIME_FRAME = [22.4 23.8];

t_vec = TIME_FRAME(1):dt:TIME_FRAME(2);

u = interp1(rr.M.timestamp, rr.M.value, t_vec, 'previous');
y = interp1(rr.q.timestamp, rr.q.value, t_vec);

ind_t_init = find(abs(diff(u)) > eps, 1);

% remove bias
du = u - mean(u(ind_t_init));
dy = y - mean(y(ind_t_init));

dataset_2.t = t_vec;
dataset_2.u = u;
dataset_2.y = y;
dataset_2.du = du;
dataset_2.dy = dy;


%%
TIME_FRAME = [36.9 38.2];

t_vec = TIME_FRAME(1):dt:TIME_FRAME(2);

u = interp1(rr.M.timestamp, rr.M.value, t_vec, 'previous');
y = interp1(rr.q.timestamp, rr.q.value, t_vec);

ind_t_init = find(abs(diff(u)) > eps, 1);

% remove bias
du = u - mean(u(ind_t_init));
dy = y - mean(y(ind_t_init));

dataset_3.t = t_vec;
dataset_3.u = u;
dataset_3.y = y;
dataset_3.du = du;
dataset_3.dy = dy;

%%
TIME_FRAME = [40 41.3];

t_vec = TIME_FRAME(1):dt:TIME_FRAME(2);

u = interp1(rr.M.timestamp, rr.M.value, t_vec, 'previous');
y = interp1(rr.q.timestamp, rr.q.value, t_vec);

ind_t_init = find(abs(diff(u)) > eps, 1);

% remove bias
du = u - mean(u(ind_t_init));
dy = y - mean(y(ind_t_init));

dataset_4.t = t_vec;
dataset_4.u = u;
dataset_4.y = y;
dataset_4.du = du;
dataset_4.dy = dy;

%%
TIME_FRAME = [43.4 44.7];

t_vec = TIME_FRAME(1):dt:TIME_FRAME(2);

u = interp1(rr.M.timestamp, rr.M.value, t_vec, 'previous');
y = interp1(rr.q.timestamp, rr.q.value, t_vec);

ind_t_init = find(abs(diff(u)) > eps, 1);

% remove bias
du = u - mean(u(ind_t_init));
dy = y - mean(y(ind_t_init));

dataset_5.t = t_vec;
dataset_5.u = u;
dataset_5.y = y;
dataset_5.du = du;
dataset_5.dy = dy;

%% plot doublet time history
h = figure;
stairs(dataset_1.t - dataset_1.t(1), dataset_1.du)
ylim([-.04, .04])
grid
xlim([0 1])
title('Doublet signal')
xlabel('time [s]')
ylabel('pitch moment M []')

%% choose dataset for ID/verification and validation
dataset_id = dataset_1;
dataset_val = dataset_2;

%% plot I/O
h= figure;

subplot 211
stairs(dataset_id.t, dataset_id.du)
grid
ylim([-.05 0.05])
title('Open-loop doublet experiment, 1DOF')
ylabel('M []')

subplot 212
plot(dataset_id.t, dataset_id.dy)
grid
ylabel('q [rad/s]')
xlabel('Time [s]')

%% build 2nd order model
% data from free response params identification
wn = 3.437;
xi = 0.0836;

mu = 1; % use an arbitrary value of mu for the time being

A = [0 1; -wn^2, - 2*xi*wn];
B = [0;mu];
C = [0 1];
D = 0;

S = ss(A,B,C,D);

G = tf(S); % tf from M to q

figure
bode(G)
grid
xlim([1e-1, 1e2])

figure
impulse(G)
grid

%% simulate
ysim = lsim(G, dataset_id.du, dataset_id.t);

h = figure;
plot(dataset_id.t, ysim)
grid
xlabel('time [s]')
ylabel('pitch rate [rad/s]')
title('Simulated response [mu=1]')

%% estimate gain
% determine peak value (as a response to the first part of the doublet)

dd = diff(dataset_id.du);
i = find(abs(dd) > eps, 1);

if dd(i) > 0 % positive doublet
    ypk = max(dataset_id.y); % [rad/s] peak of logged q
    ysim_pk = max(ysim); % [rad/s] peak of simulated q
else % negative doublet
    ypk = min(dataset_id.y); % [rad/s] peak of logged q
    ysim_pk = min(ysim); % [rad/s] peak of simulated q
end

fprintf('peak of measured response y_PK = %2.2f[rad/s]\n', ypk);
fprintf('peak of simulated response y_PK^SIM = %2.5f[rad/s]\n', ysim_pk);

mu = ypk / ysim_pk

%% re-build 2nd order model with correct mu
A = [0 1; -wn^2, - 2*xi*wn];
B = [0;mu];
C = [0 1];
D = 0;

S = ss(A,B,C,D);

G = tf(S); % tf from M to q

h = figure;
bode(G)
grid
xlim([1e-1, 1e2])

figure
impulse(G)
grid

%% I/O delay
% value estimated from experimental data (logged onboard the FCU)
Td = 20e-3; % s

G0 = G;
S.InputDelay = Td;
G.InputDelay = Td;

h = figure;
bode(G0, G)
grid
xlim([1e-1, 1e2])
legend('no delay', 'with delay')

%% verification
ysim = lsim(G, dataset_id.du, dataset_id.t);

h = figure;
plot(dataset_id.t, dataset_id.y, ...
    dataset_id.t, ysim)
grid
xlabel('time [s]')
ylabel('q [rad/s]')
title('Verification: doublet response')
legend('measured', 'simulated')

%% validation
ysim_val = lsim(G, dataset_val.du, dataset_val.t);

h = figure;
plot(dataset_val.t, dataset_val.y, ...
    dataset_val.t, ysim_val)
grid
xlabel('time [s]')
ylabel('q [rad/s]')
title('Validation: doublet response')
legend('measured', 'simulated')

%% save model
models_dir = fullfile(getIdentMainDir(), 'models');
save(fullfile(models_dir, 'ANTX_2DOF_DRONE_pitch_model_1dof_doublet.mat'), 'S')
