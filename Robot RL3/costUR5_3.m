function [r, r1, r2] = costUR5_3(w, wref, params)
%costUR5_3 calculate immediate cost for UR5 for two-axis trajectory
%
%   r = costUR5_3(state, input, params) calculates the immediate cost/reward
%       at a the next state/output y. The necessary parameters are defined in
%       params. Please note y should be the next state i.e. x_{k+1} if
%       the input is not taken into consideration while calculating the cost   
% 
% Copyright 2015 Yudha Pane
% created on      : May-13-2015
% last updated on : May-13-2015
    r1    	= -(wref(1)-w(1))*params.Q(1,1)*(wref(1)-w(1));
    r2  	= -(wref(2)-w(2))*params.Q(1,1)*(wref(2)-w(2));
    r       =   r1 + r2;
