function r = costUR5_2(z, zdot, input, old_input, zref, params)
%costUR5_2 calculate immediate cost for UR5 
%
%   r = costUR5_2(state, input, params) calculates the immediate cost/reward
%       at a the next state/output y. The necessary parameters are defined in
%       params. Please note y should be the next state i.e. x_{k+1} if
%       the input is not taken into consideration while calculating the cost   
% 
% Copyright 2015 Yudha Pane
% created on      : Mar-23-2015
% last updated on : Apr-08-2015
%     r = -(zref-z)'*params.q*(zref-z) + sign(zdot)*((zref-z)'*params.r*(zref-z))*sign(zref-z);
    r = -[zref-z; zdot]'*params.Q*[zref-z; zdot] - input*params.R*input - params.S*(input-old_input)^2;
%     if abs(z) > 0.336
%         b = cot(0.99*(z-zref))-1506;
%         r = min([b r]);
%     end
%     if abs(z) < 0.333
%         b = -2.5e3 + 0.8e2*tan(4.8*zref + 32*(z-zref));
%         r = min([b r]);
%     end
        
