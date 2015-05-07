function y = plotfourUR5_2c(par, opt, opt2)
%PLOTFOURUR5_2C plot the approximated function using fourier basis
% 
% Y = PLOTFOURUR5_2C(PAR, OPT, OPT2) plot the approximated function of the
% policy (actor) and value function (critic) using fourier basis functions. 
%
% INPUTS:   PAR : the struct variable which contains the necessary parameters
%           OPT : choose either 'actor' or 'critic'
%           OPT2: plot view option - choose either '2d' or '3d'            
%
% OUTPUTS:  Y: just a flag that the plot is succesful
% 
% Copyright 2015 Yudha Pane
% created on      : Apr-30-2015
% last updated on : Apr-30-2015

N       = 30;
theta   = par.theta;
phi     = par.phi;

% generates coordinate for evaluation
x1      = linspace(par.cBF.r(1,1), par.cBF.r(1,2), N);
x2      = linspace(par.cBF.r(2,1), par.cBF.r(2,2), N);

if strcmp(opt, 'actor')
    for i = 1:length(x1)
        for j = 1:length(x2)
            [PhiA, ~]	= fourUR5_2c([x1(i); x2(j)], par, 'actor');
            V(i,j)    	= phi'*PhiA;
        end
    end
    if strcmp(opt2, '2d')
        imagesc(x1,x2, V);
    elseif strcmp(opt2, '3d')
        surf(x1, x2, V);
    else
        error('third parameter is not recognized. Input only "3d" or "2d"');
    end
    y = 1;
elseif strcmp(opt, 'critic')
    for i = 1:length(x1)
        for j = 1:length(x2)
            [PhiC, ~]	= fourUR5_2c([x1(i); x2(j)], par, 'critic');
            V(i,j)    	= theta'*PhiC;
        end
    end
    if strcmp(opt2, '2d')
        imagesc(x1,x2, V);
    elseif strcmp(opt2, '3d')
        surf(x1, x2, V);
    else
        error('third parameter is not recognized. Input only "3d" or "2d"');
    end
    y = 1;
else
    error('second parameter is not recognized. Input only "actor" or "critic"');
end