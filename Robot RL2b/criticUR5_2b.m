function V = criticUR5_2b(x, params)
%criticUR5_2b evaluate the value function at a certain state of UR5 robot
% 
%   V = criticUR5_2b(state, params) evaluates the value function of UR5
%   robot at state x. The necessary parameters is defined in params. The 
% 	actor does not have to have the same rbf parameters as the critic
% 
% Copyright 2015 Yudha Pane
% created on      : Apr-20-2015
% last updated on : Apr-20-2015

    V = params.theta'*rbfUR5_2b(x, params, 'critic');          % calculate value