% Yudha Prawira Pane (c)
% created on      : May-12-2015
% last updated on : May-13-2015	

% Simulation parameters
par.ts     	= 1/125;
par.t_end  	= 5;
par.Ntrial 	= 40;                     	% no of trials
par.Nrbfx 	= 40;                       % no of actor 1 rbfs 
par.Nrbfy	= 40;                       % no of actor 2 rbfs 
par.phi1	= zeros(par.Nrbfx,1);       % actor 1 parameters
par.phi2	= zeros(par.Nrbfy,1);       % actor 2 parameters
par.theta	= zeros(par.Nrbfx*par.Nrbfy,1);     % critic parameters

% define boundaries
par.xmin    = -0.47;
par.xmax    = -0.33;
par.ymin    = -0.16;
par.ymax    = -0.04;

% actor 1 rbf
xc          = linspace(par.xmin, par.xmax, par.Nrbfx);   
par.cx    	= xc;                           % center coordinate of actor1 rbf
par.Bx     	= 7e-8;                         % the rbf's variance

% actor 2 rbf
yc          = linspace(par.ymin, par.ymax, par.Nrbfy);   
par.cy      = yc;                           % center coordinate of actor2 rbf
par.By      = 7e-8;                         % the rbf's variance

% critic rbf
par.cc      = [xc; yc];                  	% center coordinate of critic rbf 
par.Bc      = [7e-8 0; 0 7e-8];        	% the rbf's variance
xc          = repmat(xc, [par.Nrbfx,1]);
xc          = reshape(xc, [1, par.Nrbfx*par.Nrbfy]);
yc          = repmat(yc, [1, par.Nrbfy]);
par.cc  	= [xc; yc];                    	% center coordinate of all rbfs  
par.Bc  	= [1.4e-07 0; 0 8e-08];           	% the rbf's variance

% RL parameters
par.aa1  = 0.01;          	% actor1 learning rate
par.aa2  = 0.01;          	% actor2 learning rate
par.ac   = 0.05;          	% critic learning rate
par.gamma    = 0.97;       	% discount term
par.lambda   = 0.65;      	% eligibility trace rate

% cost function parameters
par.Q        = [1e3 0; 0 1e3];    	% the error cost function penalty
par.R        = 1e-1;            	% the actor input cost function penalty
par.S        = 5e2;					% increment input cost function penalty

% Learning parameters
par.gamma    = 0.97;              	% discount term
par.lambda   = 0.65;              	% eligibility trace rate

% Saturation parameters
par.uSat     = 0.01;     	% additive compensator saturation [rad/s]
par.max      = par.uSat;    % saturation maximum value
par.min      = -par.uSat;   % saturation minimum value
par.sattype  = 'plain';
par.skew     = 0.500;

% Various paramaters
par.varRand          = 1e-5; 	% exploration variance
par.ampRand          = 4;    	% exploration multiplier
par.idxRand          = 300;       
par.expSteps         = 1;      	% exploration steps
par.varInitInput     = 1;      	% initial input variance for each new trial
par.expStepsRedIter  = 400;    	% the exploration steps is reduced at this iteration    
par.expVarRedIter    = 680;    	% the exploration variance is reduced at this iteration
par.expStops         = 720;    	% the exploration is stopped at this iteration
par.plotSteps        = 1;    	% the actor, critic, td & return plot steps
par.acc              = 10;    	% default robot joint acceleration [rad/s^2]
par.plotopt          = '2d';   	% the option of the function approximators plot
par.plotSelect    	 = '1';
par.rlPause          = 20;
par.actorSelect      = 1;
par.osLimit          = 0.3355;

% Generate parameter vectors
par.theta   = zeros(par.Nrbfx*par.Nrbfy,1); 	% Initial critic parameter vector based on value function initialization
par.phi1    = zeros(par.Nrbfx,1);  	% Initial actor1 parameter vector 
par.phi2    = zeros(par.Nrbfy,1);  	% Initial actor2 parameter vector 
