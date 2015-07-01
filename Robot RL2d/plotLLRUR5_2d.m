function y = plotLLRUR5_2d(par, opt, opt2, kd)
%PLOTLLRUR5_2D plot the LLR functions-based approximated actor or 
%             critic function for the UR5 robot
%
%   y = plotLLRUR5_2d(par, opt) plot a LLR approximated function
%       with options as follows:
%       opt  = 'actor'   --> plot actor function
%       opt  = 'critic'  --> plot critic function
%       opt2 = '3d'      --> plot function in 3D
%       opt2 = otherwise --> plot function in 2D
%       
% Copyright 2015 Yudha Pane
% created on      : May-19-2015
% last updated on : May-19-2015	
    global LLR_memory_actor;
    global LLR_memory_critic;
	N       = 30;
	
	% generate coordinates for evaluation
	x1  = linspace(par.zmin, par.zmax, N);   
	x2  = linspace(par.zdmin, par.zdmax, N);

	if strcmp(opt, 'actor')
        for i = 1: length(x1)
            for j = 1: length(x2)
                Phi(i,j) = Locallinearmodel_actor([x1(i); x2(j)], par.cBF.K, 0, [], kd);
            end
        end
		if (strcmp(opt2, '3d'))
			surf(x1,x2,Phi);  shading interp;  
		else
			imagesc(x1, x2, Phi);
		end
	elseif strcmp(opt, 'critic')		
        for i = 1: length(x1)
            for j = 1: length(x2)
                Phi(i,j) = Locallinearmodel_critic([x1(i); x2(j)], par.cBF.K, 0, [], kd);
            end
        end
		if (strcmp(opt2, '3D'))
			surf(x1,x2,Phi);  shading interp;  
		else
			imagesc(x1, x2, Phi);
		end
	else
		error('not a valid option');
	end
	y = 1;
