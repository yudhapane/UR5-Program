% Simulation parameters
params.ts       = 1/125;
params.t_end    = 5;
params.Ntrial   = 40;                  	% no of trials
params.Nrbf     = 100;                      % no of rbfs 
params.phi      = zeros(params.Nrbf,1);      % actor parameters
params.theta    = zeros(params.Nrbf,1);      % critic parameters
params.llim     = 0.32;                     % lower limit of the state space
params.ulim     = 0.35;                     % upper limit of the state space

% radial basis functions
q1          = linspace(params.llim, params.ulim, params.Nrbf);   
params.c    = q1;                           % center coordinate of all rbfs
params.B    = 7e-8;                         % the rbf's variance

% Other RL parameters
params.q        = 5e6;                      % the error cost function penalty
params.alpha_a  = 0.05;                  	% actor learning rate
params.alpha_c  = 0.1;                      % critic learning rate
params.gamma    = 0.97;                     % discount term
params.lambda   = 0.65;                     % eligibility trace rate

% Saturation parameters
params.uSat     = 0.005;         % additive compensator saturation [rad/s]
params.max      = params.uSat;  % saturation maximum value
params.min      = -params.uSat; % saturation minimum value
params.sattype  = 'plain';
params.skew     = 0.500;

% Various paramaters
params.meanRand         = 0.0;      % exploration mean
params.varRand          = 3e-4; 	% exploration variance
params.expSteps         = 2;      	% exploration steps
params.varInitInput     = 1;        % initial input variance for each new trial
params.expStepsRedIter  = 400;      % the exploration steps is reduced at this iteration    
params.expVarRedIter    = 680;      % the exploration variance is reduced at this iteration
params.expStops         = 720;    	% the exploration is stopped at this iteration
params.plotSteps        = 1;    	% the actor, critic, td & return plot steps
params.qHome            = [-0.1921 -1.8577 2.0274 -0.1697 1.3787 3.1416]; 
params.acc              = 10;       % default robot joint acceleration [rad/s^2]
