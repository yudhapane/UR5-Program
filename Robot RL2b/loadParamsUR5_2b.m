% Yudha Prawira Pane (c)
% last updated on : Apr-22-2015	

% Simulation parameters
params.ts       = 1/125;
params.t_end    = 5;
params.Ntrial   = 500;                  	% no of trials
params.NrbfXc    = 40;                       % no of rbfs in q dimension for the critic
params.NrbfYc    = 40;                    	% no of rbfs in qdot dimension for the critic
params.NrbfXa    = 20;                       % no of rbfs in q dimension for the actor
params.NrbfYa    = 20;                    	% no of rbfs in qdot dimension for the actor

params.phi      = zeros(params.NrbfXa*params.NrbfYa,1);  	% actor parameters
params.theta    = zeros(params.NrbfXc*params.NrbfYc,1); 	% critic parameters -1e4*ones
params.zllim        = 0.32;                     % lower limit of the state space (position)
params.zulim        = 0.34;                     % upper limit of the state space
params.zdotllim 	=  -4.253e-3;             	% lower limit of the state space (velocity)
params.zdotulim     =  4e-3;                % upper limit of the state space

% generate center of rbf (2xN matrix) for the critic
zcc          = linspace(params.zllim, params.zulim, params.NrbfXc);   
zcc          = repmat(zcc, [params.NrbfYc,1]);
zcc          = reshape(zcc, [1, params.NrbfXc*params.NrbfYc]);
zdotcc       = linspace(params.zdotllim, params.zdotulim, params.NrbfYc);
zdotcc       = repmat(zdotcc, [1, params.NrbfXc]);
params.cc    = [zcc; zdotcc];                    	% center coordinate of all rbfs  
params.Bc    = [1.4e-07 0; 0 3e-08];           	% the rbf's variance

% generate center of rbf (2xN matrix) for the actor
zca          = linspace(params.zllim, params.zulim, params.NrbfXa);   
zca          = repmat(zca, [params.NrbfYa,1]);
zca          = reshape(zca, [1, params.NrbfXa*params.NrbfYa]);
zdotca       = linspace(params.zdotllim, params.zdotulim, params.NrbfYa);
zdotca       = repmat(zdotca, [1, params.NrbfXa]);
params.ca    = [zca; zdotca];                    	% center coordinate of all rbfs  
params.Ba    = [7e-06 0; 0 6e-07];           	% the rbf's variance


% cost function parameters
params.Q        = [8e10 0; 0 2e5];          	% the error cost function penalty
params.R        = 1e-1;                     % the actor input cost function penalty
params.S        = 5e2;						% increment input cost function penalty

% Learning parameters
params.alpha_a1 = 2e-06;                    % actor learning rate
params.alpha_a2 = 0.09;                     % actor learning rate
params.alpha_c1 = 0.1;                      % critic learning rate
params.alpha_c2 = 0.2;                      % critic learning rate
params.gamma    = 0.99;                     % discount term
params.lambda   = 0.65;                     % eligibility trace rate

% Saturation parameters
params.uSat     = 0.007;         % additive compensator saturation [rad/s]
params.max      = params.uSat;  % saturation maximum value
params.min      = -params.uSat; % saturation minimum value
params.sattype  = 'plain';
params.skew     = 0.500;

% Various paramaters
params.varRand          = 1e-4; 	% exploration variance
params.ampRand          = 4;        % exploration multiplier
params.idxRand          = 300;       
params.expSteps         = 1;      	% exploration steps
params.varInitInput     = 1;        % initial input variance for each new trial
params.expStepsRedIter  = 400;      % the exploration steps is reduced at this iteration    
params.expVarRedIter    = 680;      % the exploration variance is reduced at this iteration
params.expStops         = 720;    	% the exploration is stopped at this iteration
params.plotSteps        = 9;    	% the actor, critic, td & return plot steps
params.qHome            = [-0.1921 -1.8577 2.0274 -0.1697 1.3787 3.1416]; 
params.acc              = 10;       % default robot joint acceleration [rad/s^2]
params.plotopt          = '2d';     % the option of the function approximators plot
params.rlPause          = 10;
params.actorSelect      = 1;
params.osLimit          = 0.3354;