%% Preparation
format long;
clc; close all;
clearvars -except arm UR5;
load('par-11-Jun-15__12-48 PM.mat');
load('ur5PoseLib.mat');

arm.moveJoints(ur5PoseLib.qHome,1,2,2);   	% move robot to the home position
pause(2.1);                                 % wait a bit more to make sure it gets there


%% Generate trajectory
SAMPLING_TIME   = par.ts;
n1              = 3/SAMPLING_TIME;
n2              = 7/SAMPLING_TIME;
n3              = n1;
N               = n1+n2+n3;
EXPERIMENT_TIME = N*par.ts;
time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
time(end)       = [];

arm.update();
q       = arm.getJointsPositions(); % auxiliary variable
w0  	= arm.getToolPositions();
Hom     = wrapToPi(arm.solveForwardKinematics(q)); % SE(4)
wrTraj(:,1:n1)  = repmat(w0,[1,n1]);
wrTraj(1,1:n1)  = linspace(w0(1), w0(1)-0.05, n1);
wrTraj(3,1:n1)  = linspace(w0(3), w0(3)+0.01, n1);
wrTraj(:,n1+1:n1+n2)    = repmat(wrTraj(:,n1),[1,n2]);
wrTraj(1,n1+1:n1+n2)    = linspace(wrTraj(1,n1), wrTraj(1,n1)-0.05, n2);
wrTraj(:,n1+n2+1:n1+n2+n3)    = repmat(wrTraj(:,n1+n2),[1,n3]);
wrTraj(1,n1+n2+1:n1+n2+n3)    = linspace(wrTraj(1,n1+n2), wrTraj(1,n1+n2)-0.05, n3);
wrTraj(3,n1+n2+1:n1+n2+n3)    = linspace(wrTraj(3,n1+n2), wrTraj(3,n1+n2)-0.01, n3);

%% Initialize variables
qTable(:,1)     = q;
qdTable(:,1)    = zeros(6,1);
wTable(:,1)   	= zeros(6,1);               % tool position
wdTable(:,1)    = zeros(6,1);               % tool velocity
wrTable(:,1)    = zeros(6,1);               % modified reference trajectory
sum_err       	= 0;
%% Run the robot
trial_count = 1;
for i=1:N-1
    %% Calculate RL-based additive reference modifier and modify the reference trajectory
    if ~mod(i,par.expSteps)  	% explore only once every defined time steps  
        if ~sat_flag(i-1) && trial_count>1
            if uad(i) > 0 % too high, bring down
                du(i)   = -0.5*par.max + 5e-5*sin(i*par.sinfreq+phase1)+ 5e-5*cos(i*par.cosfreq+phase2);  % phase-changing sinusoid exploration               
            else % uad < 0 (too low), bring up
                du(i)   =  0.5*par.max + 5e-5*sin(i*par.sinfreq+phase1)+ 5e-5*cos(i*par.cosfreq+phase2);  % phase-changing sinusoid exploration               
            end
        else
            du(i)    	= par.varRand*randn;    % ZMWN exploration
        end
    else
        du(i)     	= 0;
    end
    Delta_u         = du(i);   
    PhiA            = rbfUR5_4([wTable(3,i) wdTable(3,i)], par, 'actor');
    dref(i)       	= par.phi'*PhiA; 

    [uad(i), sat_flag(i)]   = satUR5_4(dref(i), par, Delta_u(1));                         

    wrTable(3,i+1)  = wrTraj(3,i+1) + uad(i);   % add the additive modifier to the old reference
    v               = [-wrTraj(1,i+1); -wrTraj(2,i+1); wrTraj(3,i+1)];  % adjust the signs according to the robot convention
    Hom(:,end)      = [v; 1];
    qsol            = wrapToPi(arm.solveInverseKinematics(Hom)); % auxiliary variable

    qrTraj(:,i+1)        = q;            
    qrTraj(1:4,i+1)      = qsol(1,1:4)';
%             qrTraj(4:6,i+1)      = qTable(4:6,1);

    %% Calculate the nominal control signal (PID) 
    if i == 1
    wrTable(3,i)    = wrTraj(3,i);
    [upid(:,i), err(:,i)]   = pid_control(qrTraj(:,i+1), ...
                              qTable(:,i), zeros(6,1), sum_err, par);
    else
    [upid(:,i), err(:,i)]   = pid_control(qrTraj(:,i+1), ...
                              qTable(:,i), err(:,i-1), sum_err, par);
    end
    %% Apply control input, measure state, receive reward        
    tic
    arm.setJointsSpeed(upid(:,i),par.acc,4*SAMPLING_TIME);
    PhiC            = rbfUR5_4([wTable(3,i) wdTable(3,i)], par, 'critic');
    V(i)            = par.theta'*PhiC;                    	% V(x(k))
    while(toc<SAMPLING_TIME)
    end

    arm.update();
    qTable(:,i+1) 	= arm.getJointsPositions();    
    wTable(:,i+1) 	= arm.getToolPositions();      
    wdTable(:,i+1)	= arm.getToolSpeeds();

    [r(i+1), r1(i+1), r2(i+1)] = costUR5_4([wTable(3,i) wdTable(3,i)], [wrTraj(3,i) 0], par); 	% calculate the immediate cost 

    %% Compute temporal difference & eligibility trace
    PhiCn           = rbfUR5_4([wTable(3,i+1) wdTable(3,i+1)], par, 'critic');
    V(i+1)          = par.theta'*PhiCn;                     % V(x(k+1))
    delta(i)        = r(i+1) + par.gamma*V(i+1) - V(i);    	% temporal difference 

end
plotLearning4val;           

