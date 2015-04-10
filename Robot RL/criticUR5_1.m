function V = criticUR5_1(x, params)
%criticUR5_1 evaluate the value function at a certain state of UR5 robot
% 
%   V = criticUR5_1(state, params) evaluates the value function of UR5
%   robot at state x. The necessary parameters is defined in params
% 
% Copyright 2015 Yudha Pane
% created on      : Mar-23-2015
% last updated on : Mar-23-2015

    V = params.theta'*rbfUR5_1(x, params);          % calculate value