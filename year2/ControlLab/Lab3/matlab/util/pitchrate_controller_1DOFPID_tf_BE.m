function [inner_ctr,Cff_i,Cp_i,Ci_i,Cd_i] = pitchrate_controller_1DOFPID_tf_BE( Kff, Kp, Ki, Kd, dt, N )
% This function receives controller parameters and returns the pitch angular velocity controller 
% transfer functions with the input and output signal to use with the connect function.
% 1DOF PID implementation (input variable: tracking error; derivative
% action is computed on the error, rather than on the measured output)
% The integrator is implemented with the Backward Euler approach.
%
%   INPUTS:
%   Kff: the feed-forward gain.
%   Kp: the proportional gain.
%   Ki: the integral gain.
%   Kd: the derivative gain.
%   dt: sample time.
%   N: derivative filter parameter
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
feedForward_term_i.u = 'q_0'; 
feedForward_term_i.y = 'M_ff';
Cff_i = feedForward_term_i;

proportional_term_i = tf(Kp, 1, dt);
proportional_term_i.u = 'e_q'; 
proportional_term_i.y = 'M_p';
Cp_i = proportional_term_i;

integrator_tf = tf([dt 0], [1 -1], dt);
integral_term_i = Ki * integrator_tf;
integral_term_i.u = 'e_q'; 
integral_term_i.y = 'M_i';
Ci_i = integral_term_i;

derivative_tf = tf([1 -1], [(1/N+dt), -1/N], dt);
derivative_term_i = Kd * derivative_tf;
derivative_term_i.u = 'e_q'; 
derivative_term_i.y = 'M_d';
Cd_i = derivative_term_i;

%% Sum block
error_sum_q = sumblk('e_q = q_0 - q');
moment_sum = sumblk('M = M_ff + M_p + M_i + M_d');

%% Inner controller
inner_ctr = connect(feedForward_term_i, ...
                   proportional_term_i, integral_term_i, derivative_term_i, ...
                   error_sum_q, moment_sum, ...
                   {'q_0', 'q'},{'M'});
end