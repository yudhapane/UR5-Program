function y = rbfUR5_1(x, params)
%rbfUR5_1 evaluate a radial basis function for the UR5 robot
%
%   y = rbfUR5_1(x, params) evaluate the rbf value of the UR5 robot at a 
%       state x for with paramaters defined in params
% 
% Copyright 2015 Yudha Pane
% created on      : Mar-23-2015
% last updated on : Mar-23-2015

    
    % get parameters
    c = params.c;   	% mean
    B = params.B;       % variance matrix
    N = params.Nrbf;	% no of rbfs
    
    % pre-process input and parameter
    x = repmat(x, [1, N]);
    
    % efficient matrix multiplication of (x-c)'inv(B)(x-c)
    temp = (x-c)/B;
    temp2 = temp.*(x-c);
    y = exp(-0.5*temp2);
    sumY = sum(y);
    y = transpose(y/sumY);
   