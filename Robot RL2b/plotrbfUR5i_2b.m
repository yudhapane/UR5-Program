function y = plotrbfUR5i_2b(params, opt, opt2)
%plotrbfUR5i_2b plot the radial basis functions-based approximated actor or 
%             critic function for the UR5 robot individually. Useful for
%             tuning the rbf parameters
%
%   y = plotrbfUR5_2b(params, opt) plot a rbf-based approximated function
%       with options as follows:
%       opt = 'actor' --> plot actor function
%       opt = 'critic' --> plot critic function
% 	The actor does not have to have the same rbf parameters as the critic
% 
% Copyright 2015 Yudha Pane
% created on      : Apr-20-2015
% last updated on : Apr-22-2015	

	N       = 100;
	theta   = params.theta;
	phi     = params.phi;
	
	% generate coordinates for evaluation
	x1  = linspace(params.zllim, params.zulim, N);   
	x2  = linspace(params.zdotllim, params.zdotulim, N);

	X1  = repmat(x1, [N,1]);
	X1v = reshape(X1, [1, N*N]);
	X2v = repmat(x2, [1, N]);
	X   = [X1v; X2v];
		
	if strcmp(opt, 'actor')
		% load actor rbf parameters		
		B       = params.Ba;
		c       = params.ca;
		
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
		for k = 150: 152% size(c,2)        % for each rbf
%         for k = 1: size(c,2)        % for each rbf
            tempPhi = Phi(:,k);
            tempPhi = reshape(tempPhi, N, N);
            if (strcmp(opt2, '3d'))
                surf(x1,x2,tempPhi);  hold on; %shading interp; 
            else
                imagesc(x1, x2, tempPhi); hold on;
            end            
        end
		for k = 190: 192% size(c,2)        % for each rbf
%         for k = 1: size(c,2)        % for each rbf
            tempPhi = Phi(:,k);
            tempPhi = reshape(tempPhi, N, N);
            if (strcmp(opt2, '3d'))
                surf(x1,x2,tempPhi);  hold on; %shading interp; 
            else
                imagesc(x1, x2, tempPhi); hold on;
            end            
        end
% 		Phi = actSaturate_RL2(Phi, params);
		

	elseif strcmp(opt, 'critic')
		
		% load critic rbf parameters		
		B       = params.Bc;
		c       = params.cc;
		
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
        for k = 290: 292 %size(c,2)        % for each rbf
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
