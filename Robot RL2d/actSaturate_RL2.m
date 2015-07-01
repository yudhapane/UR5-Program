function [usat, uSel] = actSaturate_RL2(u, params)
%actSaturate_RL2 calculate the saturated input value
%
%   usat =  actSaturate_RL2(u, params) calculate the saturated value for input
%           u. If u is within saturation region, usat is the same as u. If not, then 
%           usat is equal to the saturation value defined in params.
% 
% Copyright 2015 Yudha Pane
% created on      : Apr-08-2015
% last updated on : Apr-08-2015

if isscalar(u)
    if u > params.uSat
        usat = params.uSat;
        uSel = 0;
    elseif u < -params.uSat
        usat = -params.uSat;
        uSel = 0;
    else
        usat = u;
        uSel = 1;
    end
else
    usat = u;
    idxg = u>params.uSat;
    usat(idxg) = params.uSat;
    idxl = u<-params.uSat;
    usat(idxl) = -params.uSat;
end