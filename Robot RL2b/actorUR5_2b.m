function action = actorUR5_2b(x, params)
%actorUR5_2b calculates the actor output at a certain state of the UR5 robot
%
%   V = actorUR5_2b(state, params) calculates the additive joint velocity
%   compensator for the UR5 robot at state x. The joint is chosen to be 
%   joint-3. The necessary parameters are defined in params. The actor 
%   does not have to have the same rbf parameters as the critic
% 
% Copyright 2015 Yudha Pane
% created on      : Apr-20-2015
% last updated on : Apr-20-2015

    action = params.phi'*rbfUR5_2b(x,params, 'actor');     % calculate action