% Yudha Prawira Pane (c)
% created on      : Apr-29-2015
% last updated on : May-06-2015	

% Simulation parameters
par.ts    	 = 1/125;
par.t_end    = 5;
par.Ntrial   = 500;                  	% no of trials

% define boundaries
par.xmin    = -0.47;
par.xmax    = -0.33;
par.ymin    = -0.16;
par.ymax    = -0.04;

% critic fourier basis function 
par.cBF.type                = 'four';
par.cBF.N                   = 6;                                                % Order of Fourier Basis
par.cBF.r                   = [par.xmin par.xmax; par.ymin par.ymax];           % State-space bounds
par.cBF.a                   = 1e-5; % increase this (subbu's meeting 08-May-2015)                                            % Learning rate
par.cBF.rn                  = [-1 1;-1 1];                                      % Projection x -> \bar{x}
par.cBF.T                   = [2 10];                                                % Period of Fourier Basis
[par.cBF.pb, par.cBF.alpha] = fourgenUR52_c(par.cBF);                           % Generate frequency matrix and learning rate vector
% par.cBF.alpha                   = [par.cBF.alpha;par.cBF.alpha];

% actor1 fourier basis function 
par.aBF1.type                    = 'four'; 
par.aBF1.N                       = 2;                                                % Order of Fourier Basis
par.aBF1.r                       = [par.xmin par.xmax; par.ymin par.ymax];           % State-space bounds
par.aBF1.a                       = 5e-6;                                             % Learning rate
par.aBF1.rn                      = [-1 1;-1 1];                                      % Projection x -> \bar{x}
par.aBF1.f                       = 2;                                                % Which type of Fourier approximation. Choose '0' (sine), '1' (cosine) or 2 (sine+cosine). Note: '2' will generate twice as much parameters
par.aBF1.T                       = [2 10];                                       	 % Period of Fourier Basis
[par.aBF1.pb, par.aBF1.alpha] 	 = fourgenUR52_c(par.aBF1);                           % Generate frequency matrix and learning rate vector
par.aBF1.alpha                   = [par.aBF1.alpha;par.aBF1.alpha];

% actor2 fourier basis function 
par.aBF2.type                    = 'four'; 
par.aBF2.N                       = 2;                                                % Order of Fourier Basis
par.aBF2.r                       = [par.xmin par.xmax; par.ymin par.ymax];           % State-space bounds
par.aBF2.a                       = 5e-6;                                             % Learning rate
par.aBF2.rn                      = [-1 1;-1 1];                                      % Projection x -> \bar{x}
par.aBF2.f                       = 2;                                                % Which type of Fourier approximation. Choose '0' (sine), '1' (cosine) or 2 (sine+cosine). Note: '2' will generate twice as much parameters
par.aBF2.T                       = [2 10];                                         	 % Period of Fourier Basis
[par.aBF2.pb, par.aBF2.alpha] 	 = fourgenUR52_c(par.aBF2);                           % Generate frequency matrix and learning rate vector
par.aBF2.alpha                   = [par.aBF2.alpha;par.aBF2.alpha];


% cost function parameters
par.Q        = [1e3 0; 0 1e3];    	% the error cost function penalty
par.R        = 1e-1;            	% the actor input cost function penalty
par.S        = 5e2;					% increment input cost function penalty

% Learning parameters
par.gamma    = 0.99;              	% discount term
par.lambda   = 0.65;              	% eligibility trace rate

% Saturation parameters
par.uSat     = 0.001;     	% additive compensator saturation [rad/s]
par.max      = par.uSat;    % saturation maximum value
par.min      = -par.uSat;   % saturation minimum value
par.sattype  = 'plain';
par.skew     = 0.500;

% Various paramaters
par.varRand          = 1e-5; 	% exploration variance
par.ampRand          = 4;        % exploration multiplier
par.idxRand          = 300;       
par.expSteps         = 1;      	% exploration steps
par.varInitInput     = 1;        % initial input variance for each new trial
par.expStepsRedIter  = 400;      % the exploration steps is reduced at this iteration    
par.expVarRedIter    = 680;      % the exploration variance is reduced at this iteration
par.expStops         = 720;    	% the exploration is stopped at this iteration
par.plotSteps        = 10;    	% the actor, critic, td & return plot steps
par.acc              = 10;       % default robot joint acceleration [rad/s^2]
par.plotopt          = '2d';     % the option of the function approximators plot
par.plotSelect    	 = '1';
par.rlPause          = 20;
par.actorSelect      = 1;
par.osLimit          = 0.3355;

% Generate parameter vectors
par.theta   = zeros(length(par.cBF.alpha),1); 	% Initial critic parameter vector based on value function initialization
par.phi1    = zeros(length(par.aBF1.alpha),1);  	% Initial actor1 parameter vector 
par.phi2    = zeros(length(par.aBF2.alpha),1);  	% Initial actor2 parameter vector 
