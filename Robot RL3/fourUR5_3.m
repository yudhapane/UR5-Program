function [phi, dphidx] = fourUR5_3(x, par, opt)
%FOUR  Generate feature values for use in RL approximation
%
% [PHI DPHIDX] = fourUR5_3(X, BF, opt) calculates the feature value of vector x, using
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
%           opt: select whether 'actor1', 'actor2' or 'critic'
%            
% OUTPUTS:  PHI:    feature vector
%           DPHIDX: derivative to x of feature vector
% 
% Original code is written by: Subramanya P. Nageshrao
% first modified on	: May-13-2015
% last updated on   : May-13-2015	


if strcmp(opt, 'critic')
    % Define feature vectors
%     phi     = zeros(length(par.cBF.pb)*2,1);    	% for cosine and sine
%     dphidx  = zeros(length(par.cBF.pb)*2,length(x));
    phi     = zeros(length(par.cBF.pb),1);     	% for cosine or sine only
    dphidx  = zeros(length(par.cBF.pb),length(x));

    % Scale state matrices to arbitrary domain BF.rn
    for i=1:length(x)
        x(i)  = ((x(i)-par.cBF.r(i,1))/(par.cBF.r(i,2)-par.cBF.r(i,1)))*(par.cBF.rn(i,2)-par.cBF.rn(i,1)) + par.cBF.rn(i,1);
    end

    % Calculate feature vector.
%     for i=1:length(par.cBF.pb)
%         phi(i)      	= cos(2*(pi/par.cBF.T)*par.cBF.pb(i,:)*(x - [par.zphase; 0]));
%         dphidx(i,:) 	= -par.cBF.pb(i,:)*sin(2*(pi/par.cBF.T)*par.cBF.pb(i,:)*x);
%     end
    for i=1:length(par.cBF.pb)
        phi(i)      	= cos(2*(pi./par.cBF.T).*par.cBF.pb(i,:)*x);
        dphidx(i,:) 	= -par.cBF.pb(i,:)*sin(2*(pi./par.cBF.T).*par.cBF.pb(i,:)*x);
%         phi(i)                      = cos(2*(pi./par.cBF.T).*par.cBF.pb(i,:)*x);
%         phi(i+length(par.cBF.pb))	= sin(2*(pi./par.cBF.T).*par.cBF.pb(i,:)*x);
%         dphidx(i,:)                 = -2*(pi./par.cBF.T).*par.cBF.pb(i,:)*sin(2*(pi./par.cBF.T).*par.cBF.pb(i,:)*x);
%         dphidx(i+length(par.cBF.pb),:)   = 2*(pi./par.cBF.T).*par.cBF.pb(i,:)*cos(2*(pi./par.cBF.T).*par.cBF.pb(i,:)*x);
    end
elseif strcmp(opt, 'actor1')
    % Define feature vectors
    phi     = zeros(length(par.aBF1.pb)*2,1);    	% for cosine and sine
    dphidx  = zeros(length(par.aBF1.pb)*2,length(x));
%     phi     = zeros(length(par.aBF1.pb),1);     	% for cosine or sine only
%     dphidx  = zeros(length(par.aBF1.pb),length(x));

    % Scale state matrices to arbitrary domain BF.rn
    for i=1:length(x)
        x(i)  = ((x(i)-par.aBF1.r(i,1))/(par.aBF1.r(i,2)-par.aBF1.r(i,1)))*(par.aBF1.rn(i,2)-par.aBF1.rn(i,1)) + par.aBF1.rn(i,1);
    end

%     % Calculate feature vector.
%     for i=1:length(par.aBF1.pb)
%         phi(i)                      = cos(2*(pi/par.aBF1.T)*par.aBF1.pb(i,:)*(x - [par.zphase; 0]));
% %         phi(i+length(par.aBF1.pb))	= sin(2*(pi/par.aBF1.T)*par.aBF1.pb(i,:)*(x - [par.zphase; 0]));
%         dphidx(i,:)                 = -2*(pi/par.aBF1.T)*par.aBF1.pb(i,:)*sin(2*(pi/par.aBF1.T)*par.aBF1.pb(i,:)*x);
% %         dphidx(i+length(par.aBF1.pb),:)   = 2*(pi/par.aBF1.T)*par.aBF1.pb(i,:)*cos(2*(pi/par.aBF1.T)*par.aBF1.pb(i,:)*x);
%     end
    % Calculate feature vector.
    for i=1:length(par.aBF1.pb)
        phi(i)                      = cos(2*(pi./par.aBF1.T).*par.aBF1.pb(i,:)*x);
        phi(i+length(par.aBF1.pb))	= sin(2*(pi./par.aBF1.T).*par.aBF1.pb(i,:)*x);
        dphidx(i,:)                 = -2*(pi./par.aBF1.T).*par.aBF1.pb(i,:)*sin(2*(pi./par.aBF1.T).*par.aBF1.pb(i,:)*x);
        dphidx(i+length(par.aBF1.pb),:)   = 2*(pi./par.aBF1.T).*par.aBF1.pb(i,:)*cos(2*(pi./par.aBF1.T).*par.aBF1.pb(i,:)*x);
    end
elseif strcmp(opt, 'actor2')
    % Define feature vectors
    phi     = zeros(length(par.aBF2.pb)*2,1);    	% for cosine and sine
    dphidx  = zeros(length(par.aBF2.pb)*2,length(x));
%     phi     = zeros(length(par.aBF2.pb),1);     	% for cosine or sine only
%     dphidx  = zeros(length(par.aBF2.pb),length(x));

    % Scale state matrices to arbitrary domain BF.rn
    for i=1:length(x)
        x(i)  = ((x(i)-par.aBF2.r(i,1))/(par.aBF2.r(i,2)-par.aBF2.r(i,1)))*(par.aBF2.rn(i,2)-par.aBF2.rn(i,1)) + par.aBF2.rn(i,1);
    end

%     % Calculate feature vector.
%     for i=1:length(par.aBF2.pb)
%         phi(i)                      = cos(2*(pi/par.aBF2.T)*par.aBF2.pb(i,:)*(x - [par.zphase; 0]));
% %         phi(i+length(par.aBF2.pb))	= sin(2*(pi/par.aBF2.T)*par.aBF2.pb(i,:)*(x - [par.zphase; 0]));
%         dphidx(i,:)                 = -2*(pi/par.aBF2.T)*par.aBF2.pb(i,:)*sin(2*(pi/par.aBF2.T)*par.aBF2.pb(i,:)*x);
% %         dphidx(i+length(par.aBF2.pb),:)   = 2*(pi/par.aBF2.T)*par.aBF2.pb(i,:)*cos(2*(pi/par.aBF2.T)*par.aBF2.pb(i,:)*x);
%     end
    % Calculate feature vector.
    for i=1:length(par.aBF2.pb)
        phi(i)                      = cos(2*(pi./par.aBF2.T).*par.aBF2.pb(i,:)*x);
        phi(i+length(par.aBF2.pb))	= sin(2*(pi./par.aBF2.T).*par.aBF2.pb(i,:)*x);
        dphidx(i,:)                 = -2*(pi./par.aBF2.T).*par.aBF2.pb(i,:)*sin(2*(pi./par.aBF2.T).*par.aBF2.pb(i,:)*x);
        dphidx(i+length(par.aBF2.pb),:)   = 2*(pi./par.aBF2.T).*par.aBF2.pb(i,:)*cos(2*(pi./par.aBF2.T).*par.aBF2.pb(i,:)*x);
    end
else
    error('Choose the option to be either "actor1", "actor2" or "critic"');   
end