function y = plotrbfUR5_4i(par, opt, opt2)
%plotrbfUR5_4i plot the radial basis functions-based approximated actor or 
%             critic function for the UR5 robot individually. Useful for
%             tuning the rbf parameters
%
%   y = plotrbfUR5_4i(par, opt) plot a rbf-based approximated function
%       with options as follows:
%       opt = 'actor' --> plot actor function
%       opt = 'critic' --> plot critic function
%       opt2= '2d' or '3d'
% 	The actor does not have to have the same rbf parameters as the critic
% 
% Copyright 2015 Yudha Pane
% created on      : Jun-05-2015
% last updated on : Jun-09-2015	

	N       = 200;
	theta   = par.theta;
	phi     = par.phi;
	
	% generate coordinates for evaluation
	x1  = linspace(par.xmin, par.xmax, N);   
	x2  = linspace(par.xdmin, par.xdmax, N);

	X1  = repmat(x1, [N,1]);
	X1v = reshape(X1, [1, N*N]);
	X2v = repmat(x2, [1, N]);
	X   = [X1v; X2v];
		
	if strcmp(opt, 'actor')
		% load actor rbf parameters		
		B       = par.Ba;
		c       = par.ca;
		
		% for plotting purpose
		Phi         = zeros(N^2,size(c,2));
        
        %% Calculating sum of rbf and normalizing the rbf
		for k = 1: size(c,2)        % for each rbf
			center = repmat(c(:,k),[1,N*N]);    

			temp = (X-center)'/B;
			temp2 = temp.*transpose(X-center);
			Phi(:,k) = exp(-0.5*sum(temp2,2));                             
		end
		PhiSum = sum(Phi,2);
        PhiSum = repmat(PhiSum, [1 size(c,2)]);
        Phi    = Phi./PhiSum;
        
        %% Plotting the desired the rbf individually        
		for k = 85: 87% :size(c,2)        % for each rbf
%         for k = 1: size(c,2)        % for each rbf
            tempPhi = Phi(:,k);
            tempPhi = reshape(tempPhi, N, N);
            if (strcmp(opt2, '3d'))
                surf(x1,x2,tempPhi);  hold on; %shading interp; 
            else
                imagesc(x1, x2, tempPhi); hold on;
            end            
        end
% 		for k = 105: 107% size(c,2)        % for each rbf
        for k = 1: size(c,2)        % for each rbf
            tempPhi = Phi(:,k);
            tempPhi = reshape(tempPhi, N, N);
            if (strcmp(opt2, '3d'))
                surf(x1,x2,tempPhi);  hold on; %shading interp; 
            else
                imagesc(x1, x2, tempPhi); hold on;
            end            
        end
% 		Phi = actSaturate_RL2(Phi, par);
		

	elseif strcmp(opt, 'critic')
		
		% load critic rbf parameters		
		B       = par.Bc;
		c       = par.cc;
		
		% for plotting purpose
		Theta         = zeros(N^2,size(c,2));

		for k = 1: size(c,2)        % for each rbf
			center = repmat(c(:,k),[1,N*N]);    

			temp = (X-center)'/B;
			temp2 = temp.*transpose(X-center);
			Theta(:,k) = exp(-0.5*sum(temp2,2));                             
		end
		ThetaSum = sum(Theta,2);
        ThetaSum = repmat(ThetaSum, [1 size(c,2)]);
        Theta    = Theta./ThetaSum;
        
        
		for k = 250: 252 %size(c,2)        % for each rbf
            tempTheta = Theta(:,k);
            tempTheta = reshape(tempTheta, N, N);
            if (strcmp(opt2, '3d'))
                surf(x1,x2,tempTheta);  hold on; %shading interp; 
            else
                imagesc(x1, x2, tempTheta); hold on;
            end            
        end
        for k = 280: 282 %size(c,2)        % for each rbf
            tempTheta = Theta(:,k);
            tempTheta = reshape(tempTheta, N, N);
            if (strcmp(opt2, '3d'))
                surf(x1,x2,tempTheta);  hold on; %shading interp; 
            else
                imagesc(x1, x2, tempTheta); hold on;
            end            
		end
	else
		error('not a valid option');
	end
	y = 1;
