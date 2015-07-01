% Yudha Prawira Pane (c)
% created on      : May-19-2015
% last updated on : May-21-2015	

% Simulation parameters
par.ts    	 = 1/125;
par.t_end    = 5;
par.Ntrial   = 500;  	% no of trials

% critic parameters
par.cBF.rows = 4;
par.cBF.N    = 300;     % maximum number of samples in critic LLR memory                                            
par.cBF.r    = [par.zmin par.zmax; par.zdmin par.zdmax];         
par.cBF.a    = 1e-7;    % Learning rate
par.cBF.K    = 15;

% actor parameters
par.aBF.rows = 3; 
par.aBF.N    = 300;     % maximum number of samples in critic LLR memory                           
par.aBF.r    = [par.zmin par.zmax; par.zdmin par.zdmax]; 
par.aBF.a1   = 1e-7;    % Learning rate for actor update with exploration term
par.aBF.a2   = 1e-13;    % Learning rate for actor update without exploration term
par.aBF.K    = 15;

% cost function parameters
% par.Q        = [5e6 0; 0 10];    	% the error cost function penalty
par.Q        = [1e3 0; 0 1e0];    	% the error cost function penalty
par.R        = 1e-1;            	% the actor input cost function penalty
par.S        = 5e2;					% increment input cost function penalty

% Learning parameters
par.gamma    = 0.97;                    % discount term
par.lambda   = 0.65;                     % eligibility trace rate

% Saturation parameters
par.uSat     = 0.003;         % additive compensator saturation [rad/s]
par.max      = par.uSat;  % saturation maximum value
par.min      = -par.uSat; % saturation minimum value
par.sattype  = 'plain';
par.skew     = 0.500;

% Various paramaters
par.varRand          = 1e-4;        % exploration variance
par.ampRand          = 4;           % exploration multiplier
par.idxRand          = 300;       
par.expSteps         = 3;           % exploration steps
par.varInitInput     = 1;           % initial input variance for each new trial
par.expStepsRedIter  = 400;         % the exploration steps is reduced at this iteration    
par.expVarRedIter    = 680;         % the exploration variance is reduced at this iteration
par.expStops         = 720;         % the exploration is stopped at this iteration
par.plotSteps        = 1;           % the actor, critic, td & return plot steps
par.acc              = 10;          % default robot joint acceleration [rad/s^2]
par.plotopt          = '2D';        % the option of the function approximators plot
par.rlPause          = 30;
par.actorSelect      = 1;
par.osLimit          = 0.3355;
par.memUpdate        = 1;           % memory
par.memUpdateCount   = 3;