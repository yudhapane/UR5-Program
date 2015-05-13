%robotRLTest3 is a script to test the RL-based additive compensator to 
% the UR5 robot. The learning process is done while following a straight 
% line trajectory. RL is an actor-critic with fourier basis function as 
% the approximator
% 
% Yudha Prawira Pane (c)
% created on      : May-13-2015	
% last updated on : May-13-2015	

%% Start ups and initialization
format long;
clc; close all;
clearvars -except arm UR5;
loadParamsUR5_3;

% load necessary trajectories and poses
load('ur5PoseLib.mat');
load('nomConInput_RL3.mat');
load('trajLib_RL3.mat');
qrTraj = trajLib.qrTraj;
wrTraj = trajLib.wrTraj;
unom   = unom';

if (~exist('arm','var'))
    clc; clear; close all;
    startup; 
    arm         = URArm();
    IP_ADDRESS  = '192.168.1.50';
    arm.fopen(IP_ADDRESS);   
    arm.update();
end
arm.moveJoints(ur5PoseLib.qHome,1,2,2);   	% move robot to the home position
pause(2.1);                                 % wait a bit more to make sure it gets there

%% Trajectory and experiments-related variables
EXPERIMENT_TIME = 30;
SAMPLING_TIME   = 1/125;
N               = EXPERIMENT_TIME/SAMPLING_TIME; % number of samples 
time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
time(end)       = [];

wrefTRAJ        = zeros(6,N);   % reference trajectory, will be populated in the iteration
qrefTRAJ        = zeros(6,N);   % joint space reference trajectory

% check if nominal control input is correct
if(length(unom) ~= N-1)
    error('Wrong nominal control input loaded. Run circleTraj.m again to obtain it');
end

%% Non-volatile memory
k               = 1;                        % initialize index variable
Phi1(:,k)       = par.phi1;                  % actor1 parameters memory
Phi2(:,k)       = par.phi2;                  % actor2 parameters memory
Theta(:,k)      = par.theta;                % critic parameters memory
Ret(:,k)        = zeros(par.Ntrial,1);      % return memory
V(k)            = 0;                        % initialize value function
sat_flag(k)  	= 1;                        % actor parameter update selector, 1 for update & 0 for not update
oldCounter      = 1;
qdRef(:,k)      = zeros(6,1);               % the nominal reference velocity
u1(:,k)          = 0;                        % action calculated by actor
qTable(:,k)     = zeros(6,1);               % joint position
wTable(:,k)   	= zeros(6,1);               % tool position
wdTable(:,k)    = zeros(6,1);               % tool velocity
qd(:,k)         = zeros(6,1);               % joint velocity
delta(k)        = 0;                        % temporal difference
Delta(k)        = 0;    
penalty 		= 0;

%% Volatile memory
volDelta_u      = zeros(2,1);
voluad          = zeros(6,1);
volu            = zeros(6,1);
volqdotRef      = zeros(6,1);
volqdotRefu     = zeros(6,1);
volqTable       = zeros(6,1);               % joint position
volwTable       = zeros(6,1);               % tool position
volwdTable      = zeros(6,1);        		% tool velocity


%% Start learning trials
for trial_count = 0:par.Ntrial
    
    % Initialize various variables
    du1(1,:)       = 0;	% initialize input and exploration signal
    r(1)        = 0;	% initialize reward/cost
    rz(1)       = 0;
    rzdot(1)    = 0;
    penalty     = 0;    
    ov_flag   	= 0; 	% overshoot flag
    
    % move robot safely to the first position of the reference trajectory
    arm.moveTool(wrTraj(:,1), 1, 1, 2);
    pause(3);
    timer = tic;
    
    arm.update();        
    
    % Initialize book keeping variables
    if trial_count == 0
        w_nominal(:,1) 	= arm.getToolPositions();      
        wd_nominal(:,1)	= arm.getToolSpeeds();
    else
        qTable(:,1) 	= arm.getJointsPositions();    
        wTable(:,1)  	= arm.getToolPositions();
        wdTable(:,1)	= arm.getToolSpeeds();
    end
    
    % Pause whenever necessary e.g. to change the parameters on the run
    if ~mod(trial_count, par.rlPause) && trial_count > 0
        pause(0.1);
    end
    
    if ~trial_count % For the very first trial, run the robot using 
                    % open-loop inverse jacobian control	
        for i=1:N-1
            tic
            arm.setJointsSpeed(unom(:,i),par.acc,2*SAMPLING_TIME);

            while(toc<SAMPLING_TIME)
            end
            
            arm.update();      
            w_nominal(:,i+1) 	= arm.getToolPositions();      
            wd_nominal(:,i+1)	= arm.getToolSpeeds();
        end            
    else
        % Run the robot with RL-based additive input
        for i=1:N-1
            %% Calculate RL-based additive compensator
            if ~mod(i,par.expSteps)  	% explore only once every defined time steps            
                du1(i)    	= par.varRand*randn;  
                du2(i)    	= par.varRand*randn;                  
            else
                du1(i)     	= 0;
                du2(i)     	= 0;
            end
            Delta_u         = [du1(i) du2(i)];   
            PhiA1          	= fourUR5_3(wTable(1:2,i), par, 'actor1');
            PhiA2          	= fourUR5_3(wTable(1:2,i), par, 'actor2');
            u1(i)           = par.phi1'*PhiA1;
            u2(i)           = par.phi2'*PhiA2;
                                     
            [uad1(trial_count,i), sat_flag1(i)]   = satUR5_2(u1(i), par, Delta_u(1)); 
            [uad2(trial_count,i), sat_flag2(i)]   = satUR5_2(u2(i), par, Delta_u(2)); 
            
            uad          =  [0 0 0 sat_flag1(i)*uad1(trial_count,i) sat_flag2(i)*uad2(trial_count,i) 0]';

            volDelta_u(:,i) = Delta_u;

            %% Combine the nominal control input with the RL-based compensator		
            utot(:,i)     = unom(:,i) + uad;       

            %% Apply control input, measure state, receive reward        
            tic
            arm.setJointsSpeed(utot(:,i),par.acc,2*SAMPLING_TIME);

            while(toc<SAMPLING_TIME)
            end
            arm.update();  
            qTable(:,i+1) 	= arm.getJointsPositions();    
            wTable(:,i+1) 	= arm.getToolPositions();      
            wdTable(:,i+1)	= arm.getToolSpeeds();

            [r(i+1), r1(i+1), r2(i+1)] = costUR5_3(wTable(1:2,i), wrTraj(1:2,i), par); 	% calculate the immediate cost 

            %% Compute temporal difference & eligibility trace
            PhiC            = fourUR5_3(wTable(1:2,i), par, 'critic');
            PhiCn           = fourUR5_3(wTable(1:2,i+1), par, 'critic');
            V(i)            = par.theta'*PhiC;                    	% V(x(k))
            V(i+1)          = par.theta'*PhiCn;                     % V(x(k+1))
            delta(i)        = r(i+1) + par.gamma*V(i+1) - V(i);    	% temporal difference 

            %% Update critic and actor parameters
            % adjust learning rate according to td, the larger the
            % coefficient, the stronger the filter applied
%             alpha_c1 = par.cBF.alpha/(1+abs(10*delta(i)));   	
%             alpha_a1 = par.aBF1.alpha/(1+abs(10*delta(i)));    
%             alpha_a2 = par.aBF2.alpha/(1+abs(10*delta(i)));    

            % Update actor and critic
            par.theta       = par.theta + delta(i)*par.cBF.alpha.*PhiC;   	% critic            
            if par.actorSelect 
                par.phi1 	= par.phi1 + delta(i)*sat_flag1(i)*par.aBF1.alpha.*PhiA1;       % actor 1
                par.phi2 	= par.phi2 + delta(i)*sat_flag2(i)*par.aBF2.alpha.*PhiA2;       % actor 2
            else
                par.phi1  	= par.phi1 + delta(i)*sat_flag1(i)*Delta_u*par.aBF1.alpha.*PhiA1; 	% actor 1
                par.phi2  	= par.phi2 + delta(i)*sat_flag2(i)*Delta_u*par.aBF2.alpha.*PhiA2; 	% actor 1 
            end

            %% Compute return 
            Ret(trial_count,i+1)    = par.gamma*Ret(trial_count,i) + r(i+1);  % update return value

            % Update time step and initial state
            k = k+1;          % update index variable
        end
    end
    
    % Plotting purpose
    if ~mod(trial_count,par.plotSteps) 
        clf;
        figure(1); title(['Iteration: ' int2str(trial_count)]);
        plotLearning;           
        pause(0.2);
    end
    pause(0.1);
    par.Phi1{trial_count+1}       = par.phi1;
    par.Phi2{trial_count+1}       = par.phi2;
    par.Theta{trial_count+1}      = par.theta;
    par.Alpha_a1{trial_count+1}   = par.aBF1.alpha;
    par.Alpha_a2{trial_count+1}   = par.aBF2.alpha;
    par.Alpha_c{trial_count+1}    = par.cBF.alpha;
    par.Ret                       = Ret;        
    if (trial_count > 0)
        par.wTable{trial_count}     = wTable;
        par.wdTable{trial_count}    = wdTable;
        par.uadTable{trial_count}   = uad1(trial_count,:);
        par.uacTable{trial_count}   = utot(3,:);
    end
end    



pause(1);
arm.update();

%% Data Logging
err                     = arm.getToolPositions()-finalPos;
dataLOG.Name            = 'Robot Log Data';
dataLOG.Notes           = ['Created on: ' datestr(now) '   The robot was commanded using setJointsSpeed with constant offset'];
dataLOG.SamplingTime    = SAMPLING_TIME;
dataLOG.Time            = time;
dataLOG.refTRAJ         = wrTRAJ;
dataLOG.wTable          = wTable;
dataLOG.qrTraj          = qrTraj;
dataLOG.qTable          = qTable; 
dataLOG.ErrorX          = wrTRAJ(1,:)-wTable(1,:);
dataLOG.ErrorY          = wrTRAJ(2,:)-wTable(2,:);
dataLOG.ErrorZ          = wrTRAJ(3,:)-wTable(3,:);
dataLOG.rmsX            = rms(wrTRAJ(1,:)-wTable(1,:));
dataLOG.rmsY            = rms(wrTRAJ(2,:)-wTable(2,:));
dataLOG.rmsZ            = rms(wrTRAJ(3,:)-wTable(3,:));
dataLOG.MaxAbsX         = max(abs(wrTRAJ(1,:)-wTable(1,:)));
dataLOG.MaxAbsY         = max(abs(wrTRAJ(2,:)-wTable(2,:)));
dataLOG.MaxAbsZ         = max(abs(wrTRAJ(3,:)-wTable(3,:)));

%% Display trajectory and errors
figure; subplot(211);
plot(time(:), wrTRAJ(1,:), time(:), wTable(1,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory X-axis');
subplot(212);
plot(time(:), dataLOG.ErrorX);
title('trajectory error X-axis');

figure; subplot(211);
plot(time(:), wrTRAJ(2,:), time(:), wTable(2,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory Y-axis');
subplot(212);
plot(time(:), dataLOG.ErrorY);
title('trajectory error Y-axis');

figure; subplot(211);
plot(time(:), wrTRAJ(3,:), time(:), wTable(3,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory Z-axis');
subplot(212);
plot(time(:), dataLOG.ErrorZ);
title('trajectory error Z-axis');

%% Save data
savefolder = 'D:\Dropbox\TU Delft - MSc System & Control\Graduation Project (Thesis)\UR5 Robot\UR5 Programs\Robot Test\recorded data\';
save([savefolder 'dataLOG_' datestr(now,'dd-mmm-yyyy HH-MM-SS') '.mat'], 'dataLOG');
