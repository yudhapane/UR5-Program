function [u, err] = pid_control(ref, xm, last_error, sum_err, par)
%PID_CONTROL PID controller for UR5 robot
% 
% pid_control(ref, xm, par) calculates the PID controller signal for the UR5 robot:
% INPUTS:
%       1. ref = reference position at current time instance
%       2. xm  = measured state/output
%       3. par = parameters struct
%       4. last_error = the last error signal needed for derivative
%       calculation. For no derivative term, simplify input ''
%       5. sum_error  = the accumulated error so far. For no integral
%       action, simply input ''
% OUTPUT:
%       1. u   = the PID control signal 
%       2. err = the tracking error
% 
% Yudha Prawira Pane (c)
% created on      : Jun-01-2015
% last updated on : Jun-01-2015

% Load the controller variables
Kp = par.Kp;
Ki = par.Ki;
Kd = par.Kd;

err       = ref-xm;
sum_err   = sum_err + err;
u         = Kp*err + Kd*(err-last_error) + Ki*sum_err;