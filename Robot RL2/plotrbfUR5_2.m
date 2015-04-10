function y = plotrbfUR5_2(params, opt, opt2)
%plotrbfUR5_2 plot the radial basis functions-based approximated actor or 
%             critic function for the UR5 robot
%
%   y = plotrbfUR5_2(params, opt) plot a rbf-based approximated function
%       with options as follows:
%       opt = 'actor' --> plot actor function
%       opt = 'critic' --> plot critic function
%
% Copyright 2015 Yudha Pane
% created on      : Mar-23-2015
% last updated on : Apr-08-2015


    N       = 50;
    B       = params.B;
    theta   = params.theta;
    phi     = params.phi;
    c       = params.c;

    % generate coordinates for evaluation
    x1  = linspace(params.zllim, params.zulim, N);   
    x2  = linspace(params.zdotllim, params.zdotulim, N);

    X1  = repmat(x1, [N,1]);
    X1v = reshape(X1, [1, N*N]);
    X2v = repmat(x2, [1, N]);
    X   = [X1v; X2v];
    
    % for plotting purpose
    Phi         = zeros(N^2,size(c,2));
    if strcmp(opt, 'actor')
        for k = 1: size(c,2)        % for each rbf
            center = repmat(c(:,k),[1,N*N]);    

            temp = (X-center)'/B;
            temp2 = temp.*transpose(X-center);
            Phi(:,k) = exp(-0.5*sum(temp2,2));                             
        end
        PhiSum = sum(Phi,2);
        Phi    = Phi*phi;
        Phi    = Phi./PhiSum;
        Phi = actSaturate_RL2(Phi, params);
        Phi = reshape(Phi, N, N);
        
        if (strcmp(opt2, '3d'))
            surf(x1,x2,Phi);  shading interp;  
        else
            imagesc(x1, x2, Phi);
        end
    elseif strcmp(opt, 'critic')
        for k = 1: size(c,2)        % for each rbf
            center = repmat(c(:,k),[1,N*N]);    
            temp = (X-center)'/B;
            temp2 = temp.*transpose(X-center);
            Phi(:,k) = exp(-0.5*sum(temp2,2));                             
        end
        PhiSum  = sum(Phi,2);
        Phi     = Phi*theta;
        Phi     = Phi./PhiSum;
        Phi     = reshape(Phi, N, N);
        
        if (strcmp(opt2, '3d'))
            surf(x1,x2,Phi);    shading interp;   
        else
            imagesc(x1, x2, Phi);
        end
    else
        error('not a valid option');
    end
    y = 1;
