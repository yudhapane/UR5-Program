% Yudha Prawira Pane (c)
% created on      : Apr-29-2015
% last updated on : May-06-2015	

% Simulation parameters
par.ts    	= 1/125;
par.t_end    = 5;
par.Ntrial   = 500;                  	% no of trials

% critic fourier basis function 
par.cBF.type                = 'four';
par.cBF.N                   = 4;                                                % Order of Fourier Basis
par.cBF.r                   = [par.zmin par.zmax; par.zdmin par.zdmax]; 	% State-space bounds
par.cBF.a                   = 1e-2;                                             % Learning rate
par.cBF.rn                  = [-1 1;-1 1];                                      % Projection x -> \bar{x}
par.cBF.T                   = 1;                                                % Period of Fourier Basis
[par.cBF.pb, par.cBF.alpha] = fourgenUR52_c(par.cBF);                           % Generate frequency matrix and learning rate vector

% actor fourier basis function 
par.aBF.type                    = 'four'; 
par.aBF.N                       = 4;                                                % Order of Fourier Basis
par.aBF.r                       = [par.zmin par.zmax; par.zdmin par.zdmax];     % State-space bounds
par.aBF.a                       = 1e-4;                                             % Learning rate
par.aBF.rn                      = [-1 1;-1 1];                                      % Projection x -> \bar{x}
par.aBF.f                       = 2;                                                % Which type of Fourier approximation. Choose '0' (sine), '1' (cosine) or 2 (sine+cosine). Note: '2' will generate twice as much parameters
par.aBF.T                       = 2;                                                % Period of Fourier Basis
[par.aBF.pb, par.aBF.alpha] 	= fourgenUR52_c(par.aBF);                           % Generate frequency matrix and learning rate vector
par.aBF.alpha                   = [par.aBF.alpha;par.aBF.alpha];


% Generate parameter vectors
par.theta   = zeros(length(par.cBF.alpha),1); 	% Initial critic parameter vector based on value function initialization
par.phi     = zeros(length(par.aBF.alpha),1);  	% Initial energy-shaping parameter vector

% cost function parameters
par.Q        = [1e8 0; 0 10];    	% the error cost function penalty
par.R        = 1e-1;            	% the actor input cost function penalty
par.S        = 5e2;					% increment input cost function penalty

% Learning parameters
par.gamma    = 0.9999;                    % discount term
par.lambda   = 0.65;                     % eligibility trace rate

% Saturation parameters
par.uSat     = 0.007;         % additive compensator saturation [rad/s]
par.max      = par.uSat;  % saturation maximum value
par.min      = -par.uSat; % saturation minimum value
par.sattype  = 'plain';
par.skew     = 0.500;

% Various paramaters
par.varRand          = 1e-4; 	% exploration variance
par.ampRand          = 4;        % exploration multiplier
par.idxRand          = 300;       
par.expSteps         = 1;      	% exploration steps
par.varInitInput     = 1;        % initial input variance for each new trial
par.expStepsRedIter  = 400;      % the exploration steps is reduced at this iteration    
par.expVarRedIter    = 680;      % the exploration variance is reduced at this iteration
par.expStops         = 720;    	% the exploration is stopped at this iteration
par.plotSteps        = 1;    	% the actor, critic, td & return plot steps
par.acc              = 10;       % default robot joint acceleration [rad/s^2]
par.plotopt          = '2d';     % the option of the function approximators plot
par.rlPause          = 10;
par.actorSelect      = 1;
par.osLimit          = 0.3355;