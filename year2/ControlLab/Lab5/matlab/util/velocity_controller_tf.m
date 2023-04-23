function [inner_ctr,Cff_i,Cp_i,Ci_i,Cd_i] = velocity_controller_tf( Kff, Kp, Ki, Kd, dt, N )
% This function receives controller parameters and returns the pitch angular velocity controller 
% transfer functions with the input and output signal to use with the connect function.
% PID implementation with derivative on the output (setpoint and measured
% output are separately fed to the controller; derivative action is
% computed on the measured output)
% TODO update doc
%
%   INPUTS:
%   Kff: the feed-forward gain.
%   Kp: the proportianal gain.
%   Ki: the integral gain.
%   Kd: the derivative gain.
%   dt: sample time.
%   N: derivative filter time constant
%
%   OUTPUTS:
%   inner_ctr: the transfer function of the pitch angular rate controller
%
%   Cff_i: tf object of the feed-forward term.
%   Cp_i: tf object of the proportional term.
%   Ci_i: tf object of the integral term.
%   Cd_i: tf object of the derivative term.

%% Pitch angular velocity controller
feedForward_term_i = tf(Kff, 1, dt);
feedForward_term_i.u = 'vx_0'; 
feedForward_term_i.y = 'Fx_ff';
Cff_i = feedForward_term_i;

proportional_term_i = tf(Kp, 1, dt);
proportional_term_i.u = 'e_vx'; 
proportional_term_i.y = 'Fx_p';
Cp_i = proportional_term_i;

integrator_tf = tf(dt, [1 -1], dt);
integral_term_i = Ki * integrator_tf;
integral_term_i.u = 'e_vx'; 
integral_term_i.y = 'Fx_i';
Ci_i = integral_term_i;

derivative_tf = tf([1 -1], [(1/N+dt), -1/N], dt);
derivative_term_i = Kd * derivative_tf;
derivative_term_i.u = 'vx'; 
derivative_term_i.y = 'Fx_d';
Cd_i = derivative_term_i;

%% Sum block
error_sum_q = sumblk('e_vx = vx_0 - vx');
moment_sum = sumblk('f_dx = Fx_ff + Fx_p + Fx_i - Fx_d');

%% Inner controller
inner_ctr = connect(feedForward_term_i, ...
                   proportional_term_i, integral_term_i, derivative_term_i, ...
                   error_sum_q, moment_sum, ...
                   {'vx_0', 'vx'},{'f_dx'});
end