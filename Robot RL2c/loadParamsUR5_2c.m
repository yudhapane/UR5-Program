% Yudha Prawira Pane (c)
% created on      : Apr-29-2015
% last updated on : May-06-2015	

% Simulation parameters
par.ts    	 = 1/125;
par.t_end    = 5;
par.Ntrial   = 500;                  	% no of trials

% critic fourier basis function 
par.cBF.type                = 'four';
par.cBF.N                   = 4;                                               % Order of Fourier Basis
par.cBF.r                   = [par.zmin par.zmax; par.zdmin par.zdmax];         % State-space bounds
par.cBF.a                   = 1e-5; % increase this (subbu's meeting 08-May-2015)                                            % Learning rate
par.cBF.rn                  = [-1 1;-1 1];                                      % Projection x -> \bar{x}
par.cBF.T                   = [2 4];                                                % Period of Fourier Basis
[par.cBF.pb, par.cBF.alpha] = fourgenUR52_c(par.cBF);                           % Generate frequency matrix and learning rate vector
par.cBF.alpha                   = [par.cBF.alpha;par.cBF.alpha];

% actor fourier basis function 
par.aBF.type                    = 'four'; 
par.aBF.N                       = 4;                                                % Order of Fourier Basis
par.aBF.r                       = [par.zmin par.zmax; par.zdmin par.zdmax];         % State-space bounds
par.aBF.a                       = 5e-6;                                             % Learning rate
par.aBF.rn                      = [-1 1;-1 1];                                      % Projection x -> \bar{x}
par.aBF.f                       = 2;                                                % Which type of Fourier approximation. Choose '0' (sine), '1' (cosine) or 2 (sine+cosine). Note: '2' will generate twice as much parameters
par.aBF.T                       = [2 4];                                                % Period of Fourier Basis
[par.aBF.pb, par.aBF.alpha] 	= fourgenUR52_c(par.aBF);                           % Generate frequency matrix and learning rate vector
par.aBF.alpha                   = [par.aBF.alpha;par.aBF.alpha];

% cost function parameters
% par.Q        = [5e6 0; 0 10];    	% the error cost function penalty
par.Q        = [1e3 0; 0 1];    	% the error cost function penalty
par.R        = 1e-1;            	% the actor input cost function penalty
par.S        = 5e2;					% increment input cost function penalty

% Learning parameters
par.gamma    = 0.99;                    % discount term
par.lambda   = 0.65;                     % eligibility trace rate

% Saturation parameters
par.uSat     = 0.01;         % additive compensator saturation [rad/s]
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
par.plotSteps        = 10;    	% the actor, critic, td & return plot steps
par.acc              = 10;       % default robot joint acceleration [rad/s^2]
par.plotopt          = '2d';     % the option of the function approximators plot
par.rlPause          = 20;
par.actorSelect      = 1;
par.osLimit          = 0.3355;

% Generate parameter vectors
par.theta   = zeros(length(par.cBF.alpha),1); 	% Initial critic parameter vector based on value function initialization
par.phi     = zeros(length(par.aBF.alpha),1);  	% Initial energy-shaping parameter vector
% par.theta = [  -0.016824031672822
%    0.008444581021837
%    0.001721512161975
%   -0.000825053446040
%   -0.000777167243685
%    0.000112385359483
%    0.000376936217791
%   -0.005061683568699
%    0.005034825812541
%    0.000067877422407
%   -0.000768771634820
%   -0.000292926275952
%    0.000167256492348
%    0.000269837528310
%    0.001758908029494
%    0.000631667526312
%    0.000034445467958
%   -0.000347423900900
%   -0.000178098293999
%    0.000212273009210
%    0.000097502980560
%   -0.000063537044085
%    0.000273627101768
%    0.000113984009753
%   -0.000268612655233
%    0.000045340620447
%    0.000049883833341
%    0.000071856279303
%   -0.000381032787104
%    0.000276569407280
%    0.000078164399114
%   -0.000098278667809
%    0.000038354434121
%   -0.000040721094664
%    0.000109228721365
%   -0.000253213204413
%    0.000382995610260
%   -0.000111234790699
%    0.000077636432022
%   -0.000079648613304
%    0.000014804659735
%    0.000116332023267
%    0.000159613930381
%    0.000066813877065
%    0.000016438805737
%    0.000058668575101
%   -0.000143144965005
%    0.000163860058234
%   -0.000055688492436];
% par.phi =   [-0.003661986424574
%   -0.000194663813331
%    0.001555936699951
%    0.000555505293400
%   -0.000043440213269
%   -0.000353193018973
%   -0.000463818474406
%   -0.002365564618804
%    0.000971754472676
%    0.000774155175308
%    0.000174049843411
%   -0.000115336095032
%   -0.000353662562444
%   -0.000343693033010
%   -0.000181137632661
%    0.000499384832212
%    0.000232888768623
%    0.000082619226833
%   -0.000159668869467
%   -0.000305518570182
%   -0.000267133884823
%    0.000032283319438
%   -0.000013311522087
%    0.000050667930021
%    0.000025468601993
%   -0.000120294149570
%   -0.000214177573819
%   -0.000183301597214
%   -0.000135352752945
%   -0.000144295631702
%   -0.000016495377268
%   -0.000014990964571
%   -0.000093275591660
%   -0.000162043723235
%   -0.000092637592668
%   -0.000243963348550
%   -0.000147849488542
%   -0.000016079422505
%    0.000013023705254
%   -0.000072450230653
%   -0.000108265630235
%   -0.000041502067841
%   -0.000266185164233
%   -0.000158332676264
%   -0.000011163197692
%    0.000008087274153
%   -0.000038338710635
%   -0.000032526777976
%   -0.000003432280403];