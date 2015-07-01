% Yudha Prawira Pane (c)
% created on      : June-10-2015
% last updated on : June-18-2015	

% Simulation parameters
par.ts     	= 1/125;
par.t_end  	= 5;
par.Ntrial 	= 400;    	% no of trials
par.NrbfXc  = 20;    	% no of rbfs in x dimension for the critic
par.NrbfYc  = 10;      	% no of rbfs in xdot dimension for the critic
par.NrbfXa  = 20;    	% no of rbfs in x dimension for the actor
par.NrbfYa  = 10;     	% no of rbfs in xdot dimension for the actor

par.phi     = zeros(par.NrbfXa*par.NrbfYa,1);  	% actor parameters
par.theta   = zeros(par.NrbfXc*par.NrbfYc,1); 	% critic parameters, optimistic
% par.theta   = -1e2*ones(par.NrbfXc*par.NrbfYc,1); 	% critic parameters, pessimistic

% define boundaries
% par.xmin    = -0.0059;      % x = error on axis of choice
% par.xmax    =  0.0059;
% par.xdmin   = -2.253e-3;
% par.xdmax   =  2.253e-3;
par.xmin    = -0.03;      % x = error on axis of choice
par.xmax    =  0.03;
% par.xdmin   = -1e-2;
% par.xdmax   =  1e-2;
par.xdmin   = -2e-2;
par.xdmax   =  2e-2;

% generate center of rbf (2xN matrix) for the actor
xca       	= linspace(par.xmin, par.xmax, par.NrbfXa);   
xca       	= repmat(xca, [par.NrbfYa,1]);
xca       	= reshape(xca, [1, par.NrbfXa*par.NrbfYa]);
xdotca    	= linspace(par.xdmin, par.xdmax, par.NrbfYa);
xdotca   	= repmat(xdotca, [1, par.NrbfXa]);
par.ca      = [xca; xdotca];                    	% center coordinate of actor rbfs  
% par.Ba      = [5e-7 0; 0 5e-7];                   % the rbf's variance
par.Ba      = [1e-5 0; 0 3e-6];                     % the rbf's variance


% generate center of rbf (2xN matrix) for the critic
xcc      	= linspace(par.xmin, par.xmax, par.NrbfXc);   
xcc      	= repmat(xcc, [par.NrbfYc,1]);
xcc      	= reshape(xcc, [1, par.NrbfXc*par.NrbfYc]);
xdotcc    	= linspace(par.xdmin, par.xdmax, par.NrbfYc);
xdotcc    	= repmat(xdotcc, [1, par.NrbfXc]);
par.cc      = [xcc; xdotcc];                    	% center coordinate of critic rbfs  
% par.Bc      = [5e-7 0; 0 5e-7];                 % the rbf's variance
par.Bc      = [2e-5 0; 0 6e-6];                 % the rbf's variance

% RL parameters
par.alpha_a1= 1e-1;       	% actor learning rate 
par.alpha_a2= 3e-5;      	% actor learning rate (alternative)
par.alpha_c = 5e-2;      	% critic learning rate 

% par.alpha_a1= 1e-1;       	% actor learning rate 
% par.alpha_a2= 1e-4;      	% actor learning rate (alternative)
% par.alpha_c = 1e-2;      	% critic learning rate 

% par.alpha_a1= 1e-6;       	% actor learning rate (pessimistic setting)
% par.alpha_a2= 1e-2;      	% actor learning rate (alternative)
% par.alpha_c = 1e-1;      	% critic learning rate (when par.actorSelect==0)
% par.alpha_c = 1e-5;      	% critic learning rate (pessimistic setting)

par.gamma   = 0.97;       	% discount term
par.lambda  = 0.65;      	% eligibility trace rate

% PID controller parameters
Kp          = 1;
Ki          = 0;
Kd          = 1e-6;
par.Kp      = Kp-Ki*par.ts/2;
par.Ki      = Ki*par.ts;
par.Kd      = Kd/par.ts;

% cost function parameters
% par.Q        = [1e8 0; 0 1e1];    	% the error cost function penalty (sinusoid random exp)
par.Q        = [1e9 0; 0 1e4];    	% the error cost function penalty (ZMWN random exp)
% par.Q        = [1e2 0; 0 1e-1];    	% the error cost function penalty (pessimistic setting)
par.R        = 1e-1;            	% the actor input cost function penalty
par.S        = 5e2;					% increment input cost function penalty

% Saturation parameters
par.sat     = 2e-3;     	% additive modifier saturation [m]
par.max      = par.sat;    % saturation maximum value
par.min      = -par.sat;   % saturation minimum value
par.sattype  = 'plain';
par.skew     = 0.500;

% Various paramaters
par.plotSteps        = 1;    	% the actor, critic, td & return plot steps
par.acc              = 10;    	% default robot joint acceleration [rad/s^2]
par.plotopt          = '2d';   	% the option of the function approximators plot
par.plotSelect    	 = '1';
par.rlPause          = 30;
par.actorSelect      = 0;
par.osLimit          = 0.3355;

% Exploration parameters
par.varRand          = 5e-5; 	% exploration variance
par.ampRand          = 4;    	% exploration multiplier
par.idxRand          = 300;       
par.expSteps         = 5;      	% exploration steps
par.varInitInput     = 1;      	% initial input variance for each new trial
par.expStepsRedIter  = 400;    	% the exploration steps is reduced at this iteration    
par.expVarRedIter    = 680;    	% the exploration variance is reduced at this iteration
par.expStops         = 720;    	% the exploration is stopped at this iteration
par.sinfreq          = 1/10;
par.cosfreq          = 1/6;
par.expSelect        = 'zwmn';     

% PID control limit (important for abrupt reference signal e.g. step)
par.pid1.min = 4* 0.000219001549625;
par.pid2.min = 4* 0.000480391620402;
par.pid3.min = 4*-0.091047262777211;
par.pid4.min = 4*-0.009924827314109;
par.pid5.min = 4*-0.000185910548429;
par.pid6.min = 4*-0.000016204697406;
par.pid1.max = 4* 0.015108317136373;
par.pid2.max = 4* 0.077402526904071;
par.pid3.max = 4*-0.000157121334290;
par.pid4.max = 4* 0.013910912174311;
par.pid5.max = 4* 0.015049903127292;
par.pid6.max = 4* 0.000077985222060;		