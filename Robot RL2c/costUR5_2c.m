function [r, rz, rzdot] = costUR5_2c(z, zdot, zref, params)
%costUR5_2c calculate immediate cost for UR5 
%
%   r = costUR5_2c(state, input, params) calculates the immediate cost/reward
%       at a the next state/output y. The necessary parameters are defined in
%       params. Please note y should be the next state i.e. x_{k+1} if
%       the input is not taken into consideration while calculating the cost   
% 
% Copyright 2015 Yudha Pane
% created on      : Apr-30-2015
% last updated on : Apr-30-2015
    rz      = -(zref-z)^2*params.Q(1,1)*(zref-z)^2;
    rzdot   = -zdot*params.Q(2,2)*zdot;
    r       = rz + rzdot;
%     r       = -[zref-z; zdot]'*params.Q*[zref-z; zdot];
    
%     r = -(zref-z)'*params.q*(zref-z) + sign(zdot)*((zref-z)'*params.r*(zref-z))*sign(zref-z);
%     r = -[zref-z; zdot]'*params.Q*[zref-z; zdot] - input*params.R*input - params.S*(input-old_input)^2;
%     if abs(z) > 0.336
%         b = cot(0.99*(z-zref))-1506;
%         r = min([b r]);
%     end
%     if abs(z) < 0.333
%         b = -2.5e3 + 0.8e2*tan(4.8*zref + 32*(z-zref));
%         r = min([b r]);
%     end
        
