function y = plotrbfUR5_1(params, opt)
%plotrbfUR5_1 plot the radial basis functions-based approximated actor or 
%             critic function for the UR5 robot
%
%   y = plotrbf(params, opt) plot a rbf-based approximated function
%       with options as follows:
%       opt = 'actor' --> plot actor function
%       opt = 'critic' --> plot critic function
%
% Copyright 2015 Yudha Pane
% created on      : Mar-23-2015
% last updated on : Mar-23-2015

    N       = 500;
    theta   = params.theta;
    phi     = params.phi;
    c       = params.c;
    B       = params.B;
    
    % generate coordinates for evaluation
    x       = linspace(params.llim, params.ulim, N);   

    % for plotting purpose
    Phi         = zeros(N,length(c));
    if strcmp(opt, 'actor')
        for k = 1: length(c)        % for each rbf
            center = repmat(c(k),[1,N]);    

            temp = (x-center)/B;
            temp2 = temp.*(x-center);
            Phi(:,k) = exp(-0.5*temp2);                             
        end
        PhiSum = sum(Phi,2);
        Phi    = Phi*phi;
        Phi    = Phi./PhiSum;
        plot(x, Phi);
    elseif strcmp(opt, 'critic')
        for k = 1: length(c)        % for each rbf
            center = repmat(c(k),[1,N]);    

            temp = (x-center)/B;
            temp2 = temp.*(x-center);
            Phi(:,k) = exp(-0.5*temp2);                             
        end
        PhiSum = sum(Phi,2);
        Phi    = Phi*theta;
        Phi    = Phi./PhiSum;
        plot(x, Phi);
    else
        error('not a valid option');
    end
    y = 1;
