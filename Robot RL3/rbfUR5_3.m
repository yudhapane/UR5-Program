function y = rbfUR5_3(x, par, opt, opt2)
%rbfUR5_3 evaluate a radial basis function for the UR5 robot 
%
%   y = rbfUR5_3(x, params,opt) evaluate the rbf value of the UR5 robot at a 
%       state x for with paramaters defined in params
%   
%     INPUT:
%           x    = robot state
%           par  = parameters struct 
%           opt  = selection parameter. Choose either 'actor' or 'critic'
%           opt2 = selection parameter. Choose either 'x' or 'y'
% Copyright 2015 Yudha Pane
% created on      : May-13-2015
% last updated on : May-13-2015

if strcmp(opt, 'actor')
    if strcmp(opt2, 'x')
        % get parameters
        c = par.cx;   	% mean
        B = par.Bx;     	% variance matrix
        N = par.Nrbfx;	% no of rbfs

        % pre-process input and parameter
        x = repmat(x(1), [1, N]);

        % efficient matrix multiplication of (x-c)'inv(B)(x-c)
        temp = (x-c)/B;
        temp2 = temp.*(x-c);
        y = exp(-0.5*temp2);
        sumY = sum(y);
        y = transpose(y/sumY);

    elseif strcmp(opt2, 'y')
        % get parameters
        c = par.cy;   	% mean
        B = par.By;     	% variance matrix
        N = par.Nrbfy;	% no of rbfs

        % pre-process input and parameter
        x = repmat(x(2), [1, N]);

        % efficient matrix multiplication of (x-c)'inv(B)(x-c)
        temp = (x-c)/B;
        temp2 = temp.*(x-c);
        y = exp(-0.5*temp2);
        sumY = sum(y);
        y = transpose(y/sumY); 

    else
        error('Unrecognized option opt2. Choose either "x" or "y"');
    end
elseif strcmp(opt, 'critic')
    	c = par.cc;                	% mean
		B = par.Bc;                	% variance matrix
		N = par.Nrbfx*par.Nrbfy;   	% no of rbfs
		
		% pre-process input and parameter
		x = repmat(x, [1, N]);
		
		% efficient matrix multiplication of (x-c)'inv(B)(x-c)
		temp    = (x-c)'/B;
		temp2   = temp.*transpose(x-c);
		y       = exp(-0.5*sum(temp2,2));
		sumY    = sum(y);
		y       = y/sumY;
else 
    error('Unrecognized option opt. Choose either "actor" or "critic"');
end