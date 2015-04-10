function y = rbfUR5_2(x, params)
%rbfUR5_2 evaluate a radial basis function for the UR5 robot
%
%   y = rbfUR5_2(x, params) evaluate the rbf value of the UR5 robot at a 
%       state x for with paramaters defined in params
% 
% Copyright 2015 Yudha Pane
% created on      : Mar-23-2015
% last updated on : Apr-08-2015

    if ~isvector(x)
        error('the input must be a vector');
    end
    if isrow(x)
        x = x';
    end
    
    c = params.c;                       % mean
    B = params.B;                       % variance matrix
    N = params.NrbfX*params.NrbfY;      % no of rbfs
    
    % pre-process input and parameter
    x = repmat(x, [1, N]);
    
    % efficient matrix multiplication of (x-c)'inv(B)(x-c)
    temp = (x-c)'/B;
    temp2 = temp.*transpose(x-c);
    y = exp(-0.5*sum(temp2,2));
    sumY = sum(y);
    y = y/sumY;
