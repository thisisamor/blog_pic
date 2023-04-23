% IDENT_FREE_RESPONSE.m
% 
% Identify free-response parameters of the pitch angular rate dynamics
% based on experimental data collected on test-bed (1DOF mode) in open-loop
% conditions.
% 
% Algorithm: three-steps procedure for 2nd order model (see doc):
% impulse-like response
% Model structure: LTI, grey-box
% Experiment: open-loop, on test-bed in 1DOF mode.
% Signal: impulse-like

clc
clear 
close all

%%
r = load(fullfile(getIdentMainDir, 'data', 'antx_2023-02-23-15-16-41.mat'));

%% plot pitch moment input
figure
stairs(r.M.timestamp, r.M.value)
grid
xlabel('time [s]')
ylabel('pitch moment []')

dt = 1/100; % [s] telemetry sampling rate

%% identify the moment when the "impulse" is applied (descent step)
thresh_m = -0.05;
ii = find(diff(r.M.value) < thresh_m, 1);
T0 = r.M.timestamp(ii)

%%
tin = 10.96;

h = figure;
stairs(r.M.timestamp - tin, r.M.value)
xlim([0, 2])
grid
xlabel('time [s]')
ylabel('pitch moment []')
title('Impulse-like excitation signal')

%%
h = figure;
plot(r.q.timestamp - T0, r.q.value)
grid
xlabel('time [s]')
ylabel('pitch rate [rad/s]')
title('Open-loop response to impulse-like signal')

%% find peak points
t_vec = r.q.timestamp - T0;

% find peak between 1 and 2.8s
i1 = t_vec > 1 & t_vec < 2.8;
t_vec1 = t_vec(i1);
[m1,im1] = max(r.q.value(i1));
t1 = t_vec1(im1)

% find peak between 12 and 13.6s
i2 = t_vec > 12 & t_vec < 13.6;
t_vec2 = t_vec(i2);
[m2,im2] = max(r.q.value(i2));
t2 = t_vec2(im2)

hold on
plot(t1,m1, 'r*')
plot(t2,m2, 'r*')

%% compute period of oscillation
N = 6; % number of periods of oscillations between t1 and t2
T = (t2-t1)/N

%% compute settling time
Ts = 16 % rough estimate of settling time

plot(0*[1 1], [2 -2], 'k--')
plot(Ts*[1 1], [2 -2], 'k--')
xlim([0-3, Ts+3])

%% estimate of natural frequency
% approximation: neglect damping ratio contribution because it is very
% small
wn = 2*pi/T

%% estimate of damping ratio
xi = 4.6/Ts/wn

%% build 2nd order model

mu = 1; % use an arbitrary value of mu for the time being

A = [0 1; -wn^2, - 2*xi*wn];
B = [0;mu];
C = [0 1];
D = 0;

S = ss(A,B,C,D);

G = tf(S); % tf from M to q

figure
impulse(G)
grid
