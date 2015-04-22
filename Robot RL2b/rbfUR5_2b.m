function y = rbfUR5_2b(x, params, opt)
%rbfUR5_2 evaluate a radial basis function for the UR5 robot
%
%   y = rbfUR5_2b(x, params) evaluate the rbf value of the UR5 robot at a 
%       state x for with paramaters defined in params. The actor does not
%		have to have the same rbf parameters as the critic
% 
% Copyright 2015 Yudha Pane
% created on      : Apr-20-2015
% last updated on : Apr-20-2015

    if ~isvector(x)
        error('the input must be a vector');
    end
    if isrow(x)
        x = x';
    end
    
	if strcmp(opt, 'actor')
		c = params.ca;                       % mean
		B = params.Ba;                       % variance matrix
		N = params.NrbfXa*params.NrbfYa;      % no of rbfs
		
		% pre-process input and parameter
		x = repmat(x, [1, N]);
		
		% efficient matrix multiplication of (x-c)'inv(B)(x-c)
		temp = (x-c)'/B;
		temp2 = temp.*transpose(x-c);
		y = exp(-0.5*sum(temp2,2));
		sumY = sum(y);
		y = y/sumY;
	elseif strcmp(opt, 'critic')
		c = params.cc;                       % mean
		B = params.Bc;                       % variance matrix
		N = params.NrbfXc*params.NrbfYc;      % no of rbfs
		
		% pre-process input and parameter
		x = repmat(x, [1, N]);
		
		% efficient matrix multiplication of (x-c)'inv(B)(x-c)
		temp = (x-c)'/B;
		temp2 = temp.*transpose(x-c);
		y = exp(-0.5*sum(temp2,2));
		sumY = sum(y);
		y = y/sumY;
	else 
		error ('Option opt is not recognized. Only feed "actor" or "critic"');
	end