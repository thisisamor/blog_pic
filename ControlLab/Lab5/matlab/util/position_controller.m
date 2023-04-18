function [outer_ctr,Cp_o] = position_controller(Kp, dt)
% This function receives controller parameters and returns the pitch angle controller 
% transfer functions with the input and output signal to use with the connect function.
%
%   INPUTS:
%   Kp: the proportianal gain.
%   dt: sample time.
%
%   OUTPUTS:
%   outer_ctr: the transfer function of the attutide pitch outer controller 
%       q_0 = Kp * (\theta_0 - \theta).
%   Cp_o: tf object of the proportional term.
%

%% Pitch angle controller
proportional_term_o = tf(Kp, 1, dt);
proportional_term_o.u = 'e_x';
proportional_term_o.y = 'vx_0';
Cp_o = proportional_term_o;

%% Sum block
error_sum_theta = sumblk('e_x = x_0 - x');

%% Outer controller
outer_ctr = connect(proportional_term_o, ...
                       error_sum_theta, ...
                       {'x_0', 'x'},{'vx_0'});   
end