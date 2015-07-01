function out = plotrbfUR5_4anim(par, opt, opt2, counter)
%plotrbfUR5_4 plot the radial basis functions-based approximated actor or 
%             critic function for the UR5 robot
%
%   y = plotrbfUR5_4(par, opt) plot a rbf-based approximated function
%       with inputs as follows:
%       par  = parameters struct
%       opt  = 'actor' or 'critic'
%       opt2 = '3d' or '2d' 
% Copyright 2015 Yudha Pane
% created on      : June-01-2015
% last updated on : June-04-2015

	N       = 100;
	theta   = par.Theta{counter};
	phi     = par.Phi{counter};
	
	% generate coordinates for evaluation
	x1  = linspace(par.xmin, par.xmax, N);   
	x2  = linspace(par.xdmin, par.xdmax, N);

	X1  = repmat(x1, [N,1]);
	X1v = reshape(X1, [1, N*N]);
	X2v = repmat(x2, [1, N]);
	X   = [X1v; X2v];
		
	if strcmp(opt, 'actor')
		% load actor rbf parameters		
		B  	= par.Ba;
		c 	= par.ca;
		
		% for plotting purpose
		Phi	= zeros(N^2,size(c,2));
        
        %% Calculating sum of rbf and normalizing the rbf
		for k = 1: size(c,2)        % for each rbf
			center = repmat(c(:,k),[1,N*N]);    

			temp = (X-center)'/B;
			temp2 = temp.*transpose(X-center);
			Phi(:,k) = exp(-0.5*sum(temp2,2));                             
		end
		PhiSum = sum(Phi,2);
		Phi    = Phi*phi;
		Phi    = Phi./PhiSum;
		Phi = reshape(Phi, N, N);
		
		if (strcmp(opt2, '3d'))
			surf(x1*1000,x2*1000,Phi);  shading interp;  
		else
			imagesc(x1*1000, x2*1000, Phi);
		end
	

	elseif strcmp(opt, 'critic')
		
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
		
		if (strcmp(opt2, '3d'))
			surf(x1*1000,x2*1000,Phi);    shading interp;   
		else
			imagesc(x1*1000, x2*1000, Phi);
		end
	else
		error('not a valid option');
	end
	out = 1;