% Simulation parameters
params.ts       = 1/125;
params.t_end    = 5;
params.Ntrial   = 300;                  	% no of trials
params.NrbfX    = 40;                       % no of rbfs in q dimension
params.NrbfY    = 40;                    	% no of rbfs in qdot dimension
params.phi      = zeros(params.NrbfX*params.NrbfY,1);  	% actor parameters
params.theta    = zeros(params.NrbfX*params.NrbfY,1); 	% critic parameters
params.zllim        = 0.32;                     % lower limit of the state space (position)
params.zulim        = 0.34;                     % upper limit of the state space
params.zdotllim 	=  -4.253e-3;             	% lower limit of the state space (velocity)
params.zdotulim     =  -6.989e-5;                % upper limit of the state space

% generate center of rbf (2xN matrix)
zc          = linspace(params.zllim, params.zulim, params.NrbfX);   
zc          = repmat(zc, [params.NrbfY,1]);
zc          = reshape(zc, [1, params.NrbfX*params.NrbfY]);
zdotc       = linspace(params.zdotllim, params.zdotulim, params.NrbfY);
zdotc       = repmat(zdotc, [1, params.NrbfX]);
params.c    = [zc; zdotc];                    	% center coordinate of all rbfs  
params.B    = [7e-06 0; 0 5e-07];           	% the rbf's variance

% Other RL parameters
params.Q        = [1e7 0; 0 1e7];          	% the error cost function penalty
params.R        = 1e-1;                     % the actor input cost function penalty
params.alpha_a  = 0.05;                     % actor learning rate
params.alpha_c  = 0.5;                      % critic learning rate
params.gamma    = 0.97;                     % discount term
params.lambda   = 0.65;                     % eligibility trace rate

% Saturation parameters
params.uSat     = 0.005;         % additive compensator saturation [rad/s]
params.max      = params.uSat;  % saturation maximum value
params.min      = -params.uSat; % saturation minimum value
params.sattype  = 'plain';
params.skew     = 0.500;

% Various paramaters
params.varRand          = 3e-4; 	% exploration variance
params.expSteps         = 3;      	% exploration steps
params.varInitInput     = 1;        % initial input variance for each new trial
params.expStepsRedIter  = 400;      % the exploration steps is reduced at this iteration    
params.expVarRedIter    = 680;      % the exploration variance is reduced at this iteration
params.expStops         = 720;    	% the exploration is stopped at this iteration
params.plotSteps        = 1;    	% the actor, critic, td & return plot steps
params.qHome            = [-0.1921 -1.8577 2.0274 -0.1697 1.3787 3.1416]; 
params.acc              = 10;       % default robot joint acceleration [rad/s^2]
params.plotopt          = '2d';     % the option of the function approximators plot