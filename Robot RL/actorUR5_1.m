function action = actorUR5_1(x, params)
%actorUR5_1 calculates the actor output at a certain state of the UR5 robot
%
%   V = actorUR5_1(state, params) calculates the additive joint velocity
%   compensator for the UR5 robot at state x. The joint is chosen to be 
%   joint-3. The necessary parameters are defined in params.
% 
% Copyright 2015 Yudha Pane
% created on      : Mar-23-2015
% last updated on : Mar-23-2015

    action = params.phi'*rbf(x,params);     % calculate action