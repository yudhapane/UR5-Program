function out = plotrbfUR5_3(par, opt, opt2, opt3)
%plotrbfUR5_3 plot the radial basis functions-based approximated actor or 
%             critic function for the UR5 robot
%
%   y = plotrbfUR5_3(par, opt) plot a rbf-based approximated function
%       with inputs as follows:
%       par  = parameters struct
%       opt  = 'actor' or 'critic'
%       opt2 = if opt equals 'actor', then choose either 'x' or 'y'. Otherwise,
%              ignore or simply put ''
%       opt3 = '3d' or '2d' (affecting critic plot only)
% Copyright 2015 Yudha Pane
% created on      : May-13-2015
% last updated on : May-13-2015

    if strcmp(opt, 'actor')
        % generate coordinates for evaluation
        N       = 200;
        x       = linspace(par.xmin, par.xmax, N);   
        y       = linspace(par.xdmin, par.xdmax, N);
  
        if strcmp(opt2, 'x')            
            phi     = par.phi1;
            c       = par.cx;
            B       = par.Bx;
            % for plotting purpose
            Phi  	= zeros(N,length(c));                  
            for k = 1: length(c)        % for each rbf
                center = repmat(c(k),[1,N]);    

                temp = (x-center)/B;
                temp2 = temp.*(x-center);
                Phi(:,k) = exp(-0.5*temp2);                             
            end
            PhiSum = sum(Phi,2);
            Phi    = Phi*phi;
            Phi    = Phi./PhiSum;
            plot(1000*x, Phi);          % plot in mm
        elseif strcmp(opt2, 'y')
            phi     = par.phi2;
            c       = par.cy;
            B       = par.By;     
            % for plotting purpose
            Phi  	= zeros(N,length(c));            
            for k = 1: length(c)        % for each rbf
                center = repmat(c(k),[1,N]);    

                temp = (y-center)/B;
                temp2 = temp.*(y-center);
                Phi(:,k) = exp(-0.5*temp2);                             
            end
            PhiSum = sum(Phi,2);
            Phi    = Phi*phi;
            Phi    = Phi./PhiSum;
            plot(1000*y, Phi);          % plot in mm
        end
    elseif strcmp(opt, 'critic')
        N       = 40;
        theta   = par.theta;        
        x       = linspace(par.xmin, par.xmax, N);   
        y       = linspace(par.xdmin, par.xdmax, N);
        X1  = repmat(x, [N,1]);
        X1v = reshape(X1, [1, N*N]);
        X2v = repmat(y, [1, N]);
        X   = [X1v; X2v];
		
        % load critic rbf parameters		
		B       = par.Bc;
		c       = par.cc;		
        
		% for plotting purpose
		Phi         = zeros(N^2,size(c,2));
		
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
		
		if (strcmp(opt3, '3d'))
			surf(x,y,Phi);    shading interp;   
		else
			imagesc(x, y, Phi);
		end
	else
		error('not a valid option');
    end
    out = 1;
