function r = costUR5_1(y, r, params)
%costUR5_1 calculate immediate cost for UR5 
%
%   r = costUR5_1(state, input, params) calculates the immediate cost/reward
%       at a the next state/output y. The necessary parameters are defined in
%       params. Please note y should be the next state i.e. x_{k+1} since
%       the input is not taken into consideration while calculating the cost   
% 
% Copyright 2015 Yudha Pane
% created on      : Mar-23-2015
% last updated on : Mar-23-2015

    r = -(y-r)*params.q*(y-r);