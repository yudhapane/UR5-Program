function [r, r1, r2] = costUR5_4(w, wref, par)
%costUR5_4 calculate immediate cost for UR5 for RL-based reference modifier
%
%   r = costUR5_4(state, input, par) calculates the immediate cost/reward
%       at a the next state/output y. The necessary parameters are defined in
%       par. Please note y should be the next state i.e. x_{k+1} if
%       the input is not taken into consideration while calculating the cost   
% 
% Copyright 2015 Yudha Pane
% created on      : June-01-2015
% last updated on : June-01-2015
    r1    	= -(wref(1)-w(1))*par.Q(1,1)*(wref(1)-w(1));
    r2      = -(wref(2)-w(2))*par.Q(2,2)*(wref(2)-w(2));
%     r2  	= -(wref(2)-w(2))*par.Q(1,1)*(wref(2)-w(2));
    r       =   r1 + r2;
