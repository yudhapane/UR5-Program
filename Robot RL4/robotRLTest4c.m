%robotRLTest4c is a script to test the RL-based additive reference modifier to 
% the UR5 robot. The learning process is done while following a straight 
% line trajectory. RL is an actor-critic with radial basis function as 
% the approximator. This is the third version which uses position tracking 
% error and reference trajectory as the state.
% 
% Yudha Prawira Pane (c)
% created on      : June-17-2015	
% last updated on : June-17-2015	

%% Start ups and initialization
format long;
clc; close all;
clearvars -except arm UR5;
loadParamsUR5_4c;

% load necessary trajectories and poses
load('ur5PoseLib.mat');

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
[wrTraj, wdrTraj, time, N]  = genTraj(2, par, arm); % generate trajectory
arm.update();
w0              = arm.getToolPositions();   % initial cartesian pose
q0              = arm.getJointsPositions(); % initial joint position
r0              = w0(4:end);                % initial orientation
qrTraj{1}(:,1)	= q0;
for i = 1:N-1
    ds              = wrTraj(:,i+1) - wrTraj(:,i);    

    % Calculate the joint space trajectory using inverse jacobian    
    J               = UR5.jacob0(qrTraj{1}(:,i));
    dq              = J\ds;
    qrTraj{1}(:,i+1)= qrTraj{1}(:,i) + dq;
end


%% Non-volatile memory
k               = 1;                        % initialize index variable
Phi(:,k)        = par.phi;                	% actor parameters memory
Theta(:,k)      = par.theta;                % critic parameters memory
Ret(:,k)        = zeros(par.Ntrial,1);      % return memory
V(k)            = 0;                        % initialize value function
sat_flag(k)  	= 1;                        % actor parameter update selector, 1 for update & 0 for not update
oldCounter      = 1;
qdRef(:,k)      = zeros(6,1);               % the nominal reference velocity
dref(:,k)    	= 0;                     	% action calculated by actor
qTable(:,k)     = zeros(6,1);               % joint position
wTable(:,k)   	= zeros(6,1);               % tool position
weTable(:,k)    = zeros(6,1);               % tool position error
wdTable(:,k)    = zeros(6,1);               % tool velocity
wrTable(:,k)    = zeros(6,1);               % modified reference trajectory
qd(:,k)         = zeros(6,1);               % joint velocity
delta(k)        = 0;                        % temporal difference
Delta(k)        = 0;    
penalty 		= 0;

%% Volatile memory
voluad          = zeros(N,1);
volu            = zeros(N,1);
volqdotRef      = zeros(N,1);
volqdotRefu     = zeros(N,1);
volqTable       = zeros(N,1);               % joint position
volwTable       = zeros(N,1);               % tool position
volwdTable      = zeros(N,1);        		% tool velocity
err             = zeros(6,1);
sum_err       	= zeros(N,1);

%% Misc variables
wdref           = linspace(par.rmin, par.rmax, size(wrTraj,2));

%% Start learning trials
for trial_count = 0:par.Ntrial
    
    % Generate the sinusoid phase
    lower_lim = -2*pi;
    upper_lim =  2*pi;
    phase1 = (upper_lim-lower_lim).*rand + lower_lim;
    phase2 = (upper_lim-lower_lim).*rand + lower_lim;


    % Pause whenever necessary e.g. to change the parameters on the run
    if ~mod(trial_count, par.rlPause) && trial_count > 0
        pause(0.1);
    end
        
    % move robot safely to the first position of the reference trajectory
    arm.moveTool(wrTraj(:,1), 1, 1, 1.5);
    pause(2);
    timer   = tic;
    
    %% Recalculate trajectory every trial
    [wrTraj2, wdrTraj2, time, N]	= genTraj(2, par, arm);  % generate trajectory

    %% Initialize various variables
    r(1)        = 0;	% initialize reward/cost
    rz(1)       = 0;
    rzdot(1)    = 0;
    penalty     = 0;    
    ov_flag   	= 0; 	% overshoot flag
    err(:,:)    = 0;
    sum_err     = 0;
    e_c         = zeros(par.NrbfXc*par.NrbfYc, 1);
    % Obtain the homegenous transformation matrix 
    arm.update();        
    
    % Initialize book keeping variables
    if trial_count == 0
        qnomTable(:,1)  = arm.getJointsPositions();         
        wnomTable(:,1) 	= arm.getToolPositions();      
        wdnomTable(:,1)	= arm.getToolSpeeds();
        wdeTable(:,1)   = zeros(3,1);        
    else
        qTable(:,1) 	= arm.getJointsPositions();    
        wTable(:,1)  	= arm.getToolPositions();
        weTable(:,1)    = wrTraj2(:,1)-wTable(:,1);
        wdTable(:,1)	= arm.getToolSpeeds();
        wdeTable(:,1)   = wdrTraj(:,1)-wdTable(1:3,1);
    end
    
    if ~trial_count % For the very first trial, run the robot using 
                    % PID controller only	
                    
        uad         = zeros(N,1);  % initialize input and exploration signal
        du          = zeros(N,1);	    
        arm.update();
        q       = arm.getJointsPositions(); % auxiliary variable
        Hom     = wrapToPi(arm.solveForwardKinematics(q)); % SE(4)
        for i=1:N-1
            tic
            if i==1                
            [upid(:,i), err(:,i)]   = pid_control(qrTraj{1}(:,i+1), ...
                                      qnomTable(:,i), zeros(6,1), sum_err, par);
            else
            [upid(:,i), err(:,i)]   = pid_control(qrTraj{1}(:,i+1), ...
                                      qnomTable(:,i), err(:,i-1), sum_err, par);                
            end
            arm.setJointsSpeed(upid(:,i),par.acc,4*par.ts);

            while(toc<par.ts)
            end
            
            arm.update();      
            wnomTable(:,i+1) 	= arm.getToolPositions();      
            wdnomTable(:,i+1)	= arm.getToolSpeeds();
            qnomTable(:,i+1)    = arm.getJointsPositions();
        end            
    else
        % Run the robot with RL-based additive input
        sum_error = zeros(6,1); % reinitialize accumulated error
        for i=1:N-1
            %% Calculate RL-based additive reference modifier and modify the reference trajectory
            waktu2 = tic;
            if ~mod(i,par.expSteps)  	% explore only once every defined time steps            
                du(i)    	= par.varRand*randn;    % ZMWN exploration
%                 du(i)       = 5e-4*sin(i*par.sinfreq+phase1)+ 5e-5*cos(i*par.cosfreq+phase2);  % phase-changing sinusoid exploration               
            else
                du(i)     	= 0;
            end
            Delta_u         = du(i);   
            PhiA            = rbfUR5_4([weTable(3,i) wrTraj2(3,i)], par, 'actor');
            dref(i)       	= par.phi'*PhiA; 
                                     
            [uad(trial_count,i), sat_flag(i)]   = satUR5_4(dref(i), par, Delta_u(1));                         
            
            wrTable(3,i+1)  = wrTraj2(3,i+1) + uad(trial_count,i);   % add the additive modifier to the old reference
            v               = [-wrTraj2(1,i+1); -wrTraj2(2,i+1); wrTable(3,i+1)];  % adjust the signs according to the robot convention
            Hom(:,end)      = [v; 1];
            qsol            = wrapToPi(arm.solveInverseKinematics(Hom)); % auxiliary variable

            qrTraj{trial_count+1}(:,i+1)        = qrTraj{1}(:,i);            
            qrTraj{trial_count+1}(1:4,i+1)      = qsol(1,1:4)';
%             qrTraj{trial_count+1}(4:6,i+1)      = qTable(4:6,1);
            
            %% Calculate the nominal control signal (PID) 
            if i == 1
            wrTable(3,i)    = wrTraj2(3,i);
            [upid(:,i), err(:,i)]   = pid_control(qrTraj{trial_count+1}(:,i+1), ...
                                      qTable(:,i), zeros(6,1), sum_err, par);
            else
            [upid(:,i), err(:,i)]   = pid_control(qrTraj{trial_count+1}(:,i+1), ...
                                      qTable(:,i), err(:,i-1), sum_err, par);
            end
            Waktu2(i) = toc(waktu2);
            %% Apply control input, measure state, receive reward        
            tic
            arm.setJointsSpeed(upid(:,i),par.acc,4*par.ts);
            PhiC            = rbfUR5_4([weTable(3,i) wrTraj2(3,i)], par, 'critic');
            V(i)            = par.theta'*PhiC;                    	% V(x(k))
            while(toc<par.ts)
            end
            waktu1 = tic;
            arm.update();
            qTable(:,i+1) 	= arm.getJointsPositions();    
            wTable(:,i+1) 	= arm.getToolPositions();      
            wdTable(:,i+1)	= arm.getToolSpeeds();
            weTable(:,i+1)  = wrTraj2(:,i+1)-wTable(:,i+1);
            wdeTable(:,i+1) = wdrTraj2(:,i+1)-wdTable(1:3,i+1);
            [r(i+1), r1(i+1), r2(i+1)] = costUR5_4([wTable(3,i) wdeTable(3,i)], [wrTraj2(3,i) 0], par); 	% calculate the immediate cost 

            %% Compute temporal difference & eligibility trace
            PhiCn           = rbfUR5_4([weTable(3,i+1) wrTraj2(3,i+1)], par, 'critic');
            V(i+1)          = par.theta'*PhiCn;                     % V(x(k+1))
            delta(i)        = r(i+1) + par.gamma*V(i+1) - V(i);    	% temporal difference 

            %% Update critic and actor parameters   
            % Update actor and critic
            e_c             = par.lambda*par.gamma*e_c + PhiC;
            par.theta       = par.theta + delta(i)*par.alpha_c*e_c;   	% critic            
            if par.actorSelect 
                par.phi 	= par.phi + delta(i)*sat_flag(i)*par.alpha_a1*PhiA;       % actor 1
            else
                par.phi  	= par.phi + delta(i)*sat_flag(i)*Delta_u*par.alpha_a2*PhiA; 	% actor 1
            end

            %% Compute return 
            Ret(trial_count,i+1)    = par.gamma*Ret(trial_count,i) + r(i+1);  % update return value
            Waktu1(i) = toc(waktu1);
            % Update time step and initial state
%             k = k+1;          % update index variable
        end
    end
    
    % Plotting purpose
    if ~mod(trial_count,par.plotSteps) && trial_count>0
        clf;
        figure(1); title(['Iteration: ' int2str(trial_count)]);
        plotLearning4c;           
        pause(0.2);
    end
    pause(0.1);
    par.Phi{trial_count+1}        = par.phi;
    par.Theta{trial_count+1}      = par.theta;
    par.Alpha_a1{trial_count+1}   = par.alpha_a1;
    par.Alpha_a2{trial_count+1}   = par.alpha_a2;
    par.Alpha_c{trial_count+1}    = par.alpha_c;
    par.Ret                       = Ret;        
    if (trial_count > 0)
        par.wTable{trial_count}     = wTable;
        par.wdTable{trial_count}    = wdTable;
        par.weTable{trial_count}    = weTable;
        par.uadTable{trial_count}   = uad(trial_count,:);
    end
end    



pause(1);
arm.update();

%% Data Logging
dataLOG.Name            = 'Robot Log Data';
dataLOG.Notes           = ['Created on: ' datestr(now) '   The robot was commanded using setJointsSpeed with constant offset'];
dataLOG.SamplingTime    = SAMPLING_TIME;
dataLOG.Time            = time;
dataLOG.refTRAJ         = wrTraj;
dataLOG.wTable          = wTable;
dataLOG.qrTraj          = qrTraj;
dataLOG.qTable          = qTable; 
dataLOG.ErrorX          = wrTraj(1,:)-wTable(1,:);
dataLOG.ErrorY          = wrTraj(2,:)-wTable(2,:);
dataLOG.ErrorZ          = wrTraj(3,:)-wTable(3,:);
dataLOG.rmsX            = rms(wrTraj(1,:)-wTable(1,:));
dataLOG.rmsY            = rms(wrTraj(2,:)-wTable(2,:));
dataLOG.rmsZ            = rms(wrTraj(3,:)-wTable(3,:));
dataLOG.MaxAbsX         = max(abs(wrTraj(1,:)-wTable(1,:)));
dataLOG.MaxAbsY         = max(abs(wrTraj(2,:)-wTable(2,:)));
dataLOG.MaxAbsZ         = max(abs(wrTraj(3,:)-wTable(3,:)));

%% Display trajectory and errors
figure; subplot(211);
plot(time(:), wrTraj(1,:), time(:), wTable(1,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory X-axis');
subplot(212);
plot(time(:), dataLOG.ErrorX);
title('trajectory error X-axis');

figure; subplot(211);
plot(time(:), wrTraj(2,:), time(:), wTable(2,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory Y-axis');
subplot(212);
plot(time(:), dataLOG.ErrorY);
title('trajectory error Y-axis');

figure; subplot(211);
plot(time(:), wrTraj(3,:), time(:), wTable(3,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory Z-axis');
subplot(212);
plot(time(:), dataLOG.ErrorZ);
title('trajectory error Z-axis');

%% Save data
savefolder = 'D:\Dropbox\TU Delft - MSc System & Control\Graduation Project (Thesis)\UR5 Robot\UR5 Programs\Robot Test\recorded data\';
save([savefolder 'dataLOG_' datestr(now,'dd-mmm-yyyy HH-MM-SS') '.mat'], 'dataLOG');
