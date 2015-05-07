function [phi, dphidx] = fourUR5_2c(x, par, opt)
%FOUR  Generate feature values for use in RL approximation
%
% [PHI DPHIDX] = fourUR5_2c(X, BF, opt) calculates the feature value of vector x, using
% (N+1)^n Fourier Basis functions (if sine or cosine) or 2(N+1)^n when 
% using both.
% 
% INPUTS:   X: n-dimensional input vector
%           
%           BF.r: [n x 2] matrix containing domain of x-values
%           
%           BF.pb: matrix containing combinations of frequencies
%              See also fourgen
%            
%           BF.rn: matrix containing scaling factors
%             rn = [x(1)_min x(1)_max; x(2)_min x(2)_max; ...; x(n)_min
%             x(n)_max]
%           
%           BF.T: Scalar period (equal for sine & cosine)
%           
%           BF.f: Use sine (0), cosine (1) or both (2) for function
%           approximation
%             
%           opt: select whether 'actor' or 'critic'
%            
% OUTPUTS:  PHI:    feature vector
%           DPHIDX: derivative to x of feature vector
% 
% Original code is written by: Subramanya P. Nageshrao
% first modified on	: Apr-30-2015
% last updated on   : Apr-30-2015	


if strcmp(opt, 'critic')
    % Define feature vectors
    phi     = zeros(length(par.cBF.pb),1);
    dphidx  = zeros(length(par.cBF.pb),length(x));

    % Scale state matrices to arbitrary domain BF.rn
    for i=1:length(x)
        x(i)  = ((x(i)-par.cBF.r(i,1))/(par.cBF.r(i,2)-par.cBF.r(i,1)))*(par.cBF.rn(i,2)-par.cBF.rn(i,1)) + par.cBF.rn(i,1);
    end

    % Calculate feature vector.
    for i=1:length(par.cBF.pb)
        phi(i)      	= cos(2*(pi/par.cBF.T)*par.cBF.pb(i,:)*(x - [par.zphase; 0]));
        dphidx(i,:) 	= -par.cBF.pb(i,:)*sin(2*(pi/par.cBF.T)*par.cBF.pb(i,:)*x);
    end

elseif strcmp(opt, 'actor')
    % Define feature vectors
%     phi     = zeros(length(par.aBF.pb)*2,1);
%     dphidx  = zeros(length(par.aBF.pb)*2,length(x));
    phi     = zeros(length(par.aBF.pb),1);
    dphidx  = zeros(length(par.aBF.pb),length(x));

    % Scale state matrices to arbitrary domain BF.rn
    for i=1:length(x)
        x(i)  = ((x(i)-par.aBF.r(i,1))/(par.aBF.r(i,2)-par.aBF.r(i,1)))*(par.aBF.rn(i,2)-par.aBF.rn(i,1)) + par.aBF.rn(i,1);
    end

    % Calculate feature vector.
    for i=1:length(par.aBF.pb)
        phi(i)                      = cos(2*(pi/par.aBF.T)*par.aBF.pb(i,:)*(x - [par.zphase; 0]));
%         phi(i+length(par.aBF.pb))	= sin(2*(pi/par.aBF.T)*par.aBF.pb(i,:)*(x - [par.zphase; 0]));
        dphidx(i,:)                 = -2*(pi/par.aBF.T)*par.aBF.pb(i,:)*sin(2*(pi/par.aBF.T)*par.aBF.pb(i,:)*x);
%         dphidx(i+length(par.aBF.pb),:)   = 2*(pi/par.aBF.T)*par.aBF.pb(i,:)*cos(2*(pi/par.aBF.T)*par.aBF.pb(i,:)*x);
    end
else
    error('Choose the option to be either "actor" or "critic"');   
end