function y = rbfUR5_4(x, par, opt)
%rbfUR5_4 evaluate a radial basis function for the UR5 robot
%
%   y = rbfUR5_4(x, par) evaluate the rbf value of the UR5 robot at a 
%       state x with paramaters defined in par. The actor does not
%		have to have the same rbf parameters as the critic
%       par  = parameters struct
%       opt  = 'actor' or 'critic'
% Copyright 2015 Yudha Pane
% created on      : June-01-2015
% last updated on : June-04-2015

    if ~isvector(x)
        error('the input must be a vector');
    end
    if isrow(x)
        x = x';
    end
    
	if strcmp(opt, 'actor')
		c = par.ca;             	% rbfs center
		B = par.Ba;                 % rbfs (co)-variance matrix
		N = par.NrbfXa*par.NrbfYa; 	% no of rbfs
		
		% pre-process input and parameter
		x = repmat(x, [1, N]);
		
		% efficient matrix multiplication of (x-c)'inv(B)(x-c)
		temp    = (x-c)'/B;
		temp2   = temp.*transpose(x-c);
		y       = exp(-0.5*sum(temp2,2));
		sumY    = sum(y);
		y       = y/sumY;
	elseif strcmp(opt, 'critic')
		c = par.cc;              	% rbfs center
		B = par.Bc;                 % rbfs (co)-variance matrix
		N = par.NrbfXc*par.NrbfYc;  % no of rbfs
		
		% pre-process input and parameter
		x = repmat(x, [1, N]);
		
		% efficient matrix multiplication of (x-c)'inv(B)(x-c)
		temp    = (x-c)'/B;
		temp2   = temp.*transpose(x-c);
		y       = exp(-0.5*sum(temp2,2));
		sumY    = sum(y);
		y       = y/sumY;
	else 
		error ('Option opt is not recognized. Only feed "actor" or "critic"');
	end