%robotRLTest2d is a script to test the RL-based additive compensator to 
% the UR5 robot. The learning process is done while following a straight 
% line trajectory. RL is an actor-critic with LLR as the function approximator
% 
% Yudha Prawira Pane (c)
% created on      : May-19-2015	
% last updated on : May-26-2015	

%% Start ups and initialization
format long;
clc; close all;
clearvars -except arm UR5 
clearvars -global
global LLR_memory_actor;
global LLR_memory_critic;


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


%% Create the trajectory
arm.update();
w0              = arm.getToolPositions();   % initial cartesian pose
q0              = arm.getJointsPositions(); % initial joint position
r0              = w0(4:end);                % initial orientation

% fill in the state space boundary parameters
par.zmin        = w0(3) - 0.0200;  
par.zmax        = w0(3) + 0.0200;
par.zdmin       = -5e-3;
par.zdmax       =  5e-3;
par.zphase      = w0(3);          	% the phase in the cosine 
loadParamsUR5_2d;                   % load the other parameters

EXPERIMENT_TIME = 5;
SAMPLING_TIME   = par.ts;
N               = EXPERIMENT_TIME/SAMPLING_TIME; % number of samples 
time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
time(end)       = [];

wrTRAJ          = zeros(6,N);   % reference trajectory, will be populated in the iteration
qrTRAJ          = zeros(6,N);   % joint space reference trajectory
offset          = [-0.10 0 0 0 0 0]';
wf              = w0 + offset;
dOffset         = offset/N;

for i = 1:N
    wrTRAJ(:,i)   = w0 + (i-1)*dOffset; % populate reference trajectory
end

% generate the joint space trajectory 
qrTRAJ(:,1)   = q0;
for i = 1:N-1
    ds              = wrTRAJ(:,i+1) - wrTRAJ(:,i);    

    % Calculate the joint space trajectory using inverse jacobian    
    J               = UR5.jacob0(qrTRAJ(:,i));
    dq              = J\ds;
    qrTRAJ(:,i+1) = qrTRAJ(:,i) + dq;
end


%% Non-volatile memory
k               = 1;                        % initialize index variable
Ret(:,k)        = zeros(par.Ntrial,1);      % return memory
V(k)            = 0;                        % initialize value function
sat_flag(k)  	= 1;                        % actor parameter update selector, 1 for update & 0 for not update
oldCounter      = 1;
qdRef(:,k)      = zeros(6,1);               % the nominal reference velocity
u(k)            = 0;                        % action calculated by actor
qTable(:,k)     = zeros(6,1);               % joint position
wTable(:,k)   	= zeros(6,1);               % tool position
wdTable(:,k)    = zeros(6,1);               % tool velocity
qd(:,k)         = zeros(6,1);               % joint velocity
delta(k)        = 0;                        % temporal difference
Delta(k)        = 0;    
rz(k)           = 0;
rzdot(k)        = 0;
penalty 		= 0;

%% Volatile memory
volDelta_u      = 0;
voluad          = 0;
volu            = 0;
volqdotRef      = zeros(6,1);
volqdotRefu     = zeros(6,1);
volqTable       = zeros(6,1);               % joint position
volwTable       = zeros(6,1);               % tool position
volwdTable      = zeros(6,1);        		% tool velocity

%% Misc variables
wdref           = linspace(par.zdmin, par.zdmax, size(wrTRAJ,2));

%% Start learning trials
for trial_count = 0:par.Ntrial
    
    % Initialize various variables
    du(1)       = 0;	% initialize input and exploration signal
    r(1)        = 0;	% initialize reward/cost
    rz(1)       = 0;	% initialize reward/cost corresponding to position error
    rzdot(1)    = 0;	% initialize reward/cost corresponding to velocity error
    e_c         = 1;	% initialize eligibility trace        
    
    arm.moveJoints(qrTRAJ(:,1),5,10,1); % move the robot to the first position of the qrTRAJ safely
    pause(1.2);
    
    arm.update();        
    
    if trial_count == 0 % if this is the 0-th trial, save the position and velocity resulting upon applying nominal control law
        w_nominal(:,1) 	= arm.getToolPositions();      
        wd_nominal(:,1)	= arm.getToolSpeeds();
    else                % else, save the states to different tables
        qTable(:,1) 	= arm.getJointsPositions();    
        wTable(:,1)  	= arm.getToolPositions();
        wdTable(:,1)	= arm.getToolSpeeds();
    end
    
    if (wdTable(3,1) > par.zdmax)   % manage saturated value
        wdTable(3,1)    = par.zdmax;
    end
    if (wdTable(3,1) < par.zdmin)   % manage saturated value
        wdTable(3,1)    = par.zdmin;
    end       
    
    % Pause whenever necessary e.g. to change parameters on the run
    if ~mod(trial_count, par.rlPause) && trial_count > 0
        pause(0.1);
    end
    
    if ~trial_count % For the 0-th trial, run the robot using 
                    % open-loop inverse jacobian control	
        for i=1:N-1
            
            % calculate the open loop controller
            qdRef(:,i)          = (qrTRAJ(:,i+1) - qrTRAJ(:,i))/SAMPLING_TIME;

            tic
            arm.setJointsSpeed(qdRef(:,i),par.acc,4*SAMPLING_TIME);

            while(toc<SAMPLING_TIME)
            end
            
            arm.update();      
            w_nominal(:,i+1) 	= arm.getToolPositions();      
            wd_nominal(:,i+1)	= arm.getToolSpeeds();
        end            
        
        % Initialize the actor and critic LLR memory
%         LLR_memory_actor{1}     = [w_nominal(3,1:10:end); wd_nominal(3,1:10:end); zeros(2,63)]';
%         LLR_memory_critic{1}    = [w_nominal(3,1:10:end); wd_nominal(3,1:10:end); zeros(2,63)]';
        LLR_memory_actor{1}     = [w_nominal(3,:); wd_nominal(3,:); zeros(2,length(w_nominal))]';
        LLR_memory_critic{1}    = [w_nominal(3,:); wd_nominal(3,:); zeros(2,length(w_nominal))]';
    	kd_critic   = kdtree_build(LLR_memory_critic{1}(:,1:2));
        kd_actor    = kdtree_build(LLR_memory_actor{1}(:,1:2));
    else
        % For the first trial onward run the robot with RL-based additive
        % compensator
        for i=1:N-1
            %% Calculate RL-based additive compensator
            if ~mod(i,par.expSteps)  	% calculate random input. Exploration only happens once every defined time steps            
                du(i)    	= par.varRand*randn;          
            else
                du(i)     	= 0;
            end
            Delta_u         = du(i);   
            
            % calculate control signal, but do not update the memory
            u(i)            = Locallinearmodel_actor([wTable(3,i); wdTable(3,i)], par.aBF.K, 0, [], kd_actor); 
            
            [uad(trial_count,i), sat_flag(i)]   = satUR5_2(u(i), par, Delta_u); 
            volDelta_u(i)                       = Delta_u;

            %% Combine the nominal control input with the RL-based compensator	
            qdRefu(:,i)     = qdRef(:,i)+ [0; 0; sat_flag(i)*uad(trial_count,i); 0; 0; 0];       

            %% Apply control input, measure state, receive reward        
            tic;
            arm.setJointsSpeed(qdRefu(:,i),par.acc,4*SAMPLING_TIME);

            % Compute V(x_k,u_k) here to achieve more efficiency
            V(i)    = Locallinearmodel_critic([wTable(3,i)  ; wdTable(3,i)  ], par.cBF.K, 0, [], kd_critic); % V(x(k))
            
            %% Update LLR memories
%             if ~mod(i,par.memUpdate) && ~mod(trial_count, par.memUpdateCount)
%             if llrsample>610  % sample with small probability
            % add actor and critic input-output samples to memories
            Locallinearmodel_critic([wTable(3,i); wdTable(3,i)], par.cBF.K, 1, V(i), kd_critic); % add input-output sample [x_k^T V(x_k)] to memory
            Locallinearmodel_actor ([wTable(3,i); wdTable(3,i)], par.aBF.K, 1, uad(trial_count,i), kd_actor); % add input-output sample [x_k^T u_k)] to memory

            % delete and rebuild trees
            kdtree_delete(kd_critic);
            kdtree_delete(kd_actor);
            kd_critic = kdtree_build(LLR_memory_critic{1}(:,1:2));
            kd_actor = kdtree_build(LLR_memory_actor{1}(:,1:2));

            while(toc<SAMPLING_TIME)
            end            
            tic;
            arm.update();  
            qTable(:,i+1) 	= arm.getJointsPositions();    
            wTable(:,i+1) 	= arm.getToolPositions();      
            wdTable(:,i+1)	= arm.getToolSpeeds();
            
            if (wdTable(3,i+1) > par.zdmax)
                wdTable(3,i+1)  = par.zdmax;
            end
            if (wdTable(3,i+1) < par.zdmin)
                wdTable(3,i+1)  = par.zdmin;
            end
            [r(i+1), rz(i+1), rzdot(i+1)] = costUR5_2c(wTable(3,i), wdTable(3,i),  wrTRAJ(3,i), par); 	% calculate the immediate cost 
        
            %% Compute temporal difference & eligibility trace                        
            V(i+1)  = Locallinearmodel_critic([wTable(3,i+1); wdTable(3,i+1)], par.cBF.K, 0, [], kd_critic); % V(x(k+1))
            delta(i)= r(i+1) + par.gamma*V(i+1) - V(i);    	% temporal difference 
    
                
            %% Update critic and actor parameters
            % obtain the indices of that sample and its neighbours
            [~, ~, ~, NNC] = Locallinearmodel_critic([wTable(3,i); wdTable(3,i)], par.cBF.K, 0, [], kd_critic); % obtain the sample and its corresponding neighbours indices
            [~, ~, ~, NNA] = Locallinearmodel_actor ([wTable(3,i); wdTable(3,i)], par.aBF.K, 0, [], kd_actor); % obtain the sample and its corresponding neighbours indices
            % adjust the memory towards the temporal differences
            LLR_memory_critic{1}(NNC,3) = LLR_memory_critic{1}(NNC,3) + par.cBF.a*delta(i);             % update critic memory      
            LLR_memory_actor{1}(NNA,3)  = LLR_memory_actor{1}(NNA,3)  + par.aBF.a1*delta(i)*Delta_u;    % update actor memory
%                 LLR_memory_actor{1}(NNA,3)  = LLR_memory_actor{1}(NNA,3)  + par.aBF.a2*delta(i);   % update actor

            % check for saturation
            upper_sat_idx = LLR_memory_actor{1}(NNA,3) > par.max;    % get index of (upper) saturated values 
            LLR_memory_actor{1}(NNA,3) = LLR_memory_actor{1}(NNA,3).*(1-upper_sat_idx) + upper_sat_idx*par.max;
            lower_sat_idx = LLR_memory_actor{1}(NNA,3) < par.min;    % get index of (lower) saturated values 
            LLR_memory_actor{1}(NNA,3) = LLR_memory_actor{1}(NNA,3).*(1-lower_sat_idx) + lower_sat_idx*par.min;                                                    
%             end
%             end
            
                        

            %% Compute return 
            Ret(trial_count,i+1)    = par.gamma*Ret(trial_count,i) + r(i+1);  % update return value

            % Update time step and initial state
            k = k+1;          % update index variable
            waktu(i) = toc;
            
        end
    end
    % Delete and rebuild trees
%     if trial_count
%         disp('deleting and rebuilding trees ...');
%         kdtree_delete(kd_critic);
%         kdtree_delete(kd_actor);
%         kd_critic = kdtree_build(LLR_memory_critic{1}(:,1:2));
%         kd_actor = kdtree_build(LLR_memory_actor{1}(:,1:2));
%     end
    
    % Plotting purpose
    if ~mod(trial_count,par.plotSteps) && trial_count
        clf;
        figure(1); title(['Iteration: ' int2str(trial_count)]);
        
        % critic
%         subplot(4,4,[1,2]); 
        positionVector1 = [0.05, 0.8, 0.43, 0.17];
        subplot('Position',positionVector1)
        plotOut = plotLLRUR5_2d(par, 'critic', par.plotopt, kd_critic); title(['\bf{CRITIC}  Iteration: ' int2str(trial_count)]); colorbar;
        xlabel('$z  \hspace{1mm}$ [mm]','Interpreter','Latex'); ylabel('$\dot{z}  \hspace{1mm}$ [mm]','Interpreter','Latex'); zlabel('$V(z)$ \hspace{1mm} [-]','Interpreter','Latex'); %colorbar 
        hold on; plot(wTable(3,:), wdTable(3,:), 'r.');
        plot(wrTRAJ(3,:),wdref,'b'); 
        plot(linspace(par.zmin, par.zmax, size(wrTRAJ,2)), zeros(1, size(wrTRAJ,2)));
        
        % actor
%         subplot(4,4,[5,6]);  
        positionVector2 = [0.05, 0.545, 0.43, 0.176];
        subplot('Position',positionVector2)
        plotOut = plotLLRUR5_2d(par, 'actor', par.plotopt, kd_actor); title('\bf{ACTOR}');  colorbar;
        hold on; plot(wTable(3,:), wdTable(3,:), 'r.'); 
        plot(wrTRAJ(3,:),wdref,'b');
        plot(linspace(par.zmin, par.zmax, size(wrTRAJ,2)), zeros(1, size(wrTRAJ,2)));
        xlabel('$z  \hspace{1mm}$ [mm]','Interpreter','Latex'); ylabel('$\dot{z}  \hspace{1mm}$ [mm]','Interpreter','Latex'); zlabel('$\pi(z)$ \hspace{1mm} [-]','Interpreter','Latex'); %colorbar 
        
        % position
%         subplot(4,4,[9,10]); 
        positionVector3 = [0.05, 0.305, 0.43, 0.16];
        subplot('Position',positionVector3)
        plot(time, 1000*wTable(3,:)); hold on; plot(time, 1000*wrTRAJ(3,:), 'r'); plot(time,1000*w_nominal(3,:), 'g');
        xlabel('time (seconds)'); ylabel('Z position (mm)'); xlim([0 par.t_end]); title('reference (red), RL (blue), no-RL (green)');       
        
        % velocity
%         subplot(4,4,[13,14]);
        positionVector4 = [0.05, 0.063, 0.43, 0.16];
        subplot('Position',positionVector4)
        plot(time, 1000*wdTable(3,:));
        xlabel('time (seconds)'); ylabel('Z velocity (mm/s)'); xlim([0 par.t_end]); title('Z velocity trajectory');                      
        
        % temporal difference
%         subplot(4,4,[7,8]); 
        positionVector5 = [0.55, 0.72, 0.43, 0.24];
        subplot('Position',positionVector5)
        plot(time(1:end-1), delta); title('\bf{Temporal difference}'); 
        hold on; plot(time(1:end-1), zeros(1, N-1));
        xlabel('time steps'); %ylim([-100 100]);
        
        % Return
%         subplot(4,4,[3,4]); 
        positionVector6 = [0.55, 0.39, 0.41, 0.24];
        subplot('Position',positionVector6)
        tempRet = sum(Ret,2);
        plotyy(1:1:par.Ntrial, sum(Ret,2), 51:1:par.Ntrial, tempRet(51:end)); title('\bf{Return}');   %ylim([-10 5]);
        xlabel('trials');
                  

        % additive input and exploration
%         subplot(4,4,[11,12]); 
        positionVector7 = [0.55, 0.063, 0.43, 0.24];
        subplot('Position',positionVector7)
        plot(time(1:end-1), u, time(1:end-1), volDelta_u, 'r'); 
        xlabel('time (seconds)'); ylabel('additive input (rad/s)'); xlim([0 par.t_end]); title('Actor (b) & exploration (r)');       
           
        pause(0.2);
    end
    pause(0.1);
    par.Ret                       = Ret;        
    if (trial_count > 0)
        par.wTable{trial_count}     = wTable;
        par.wdTable{trial_count}    = wdTable;
        par.uadTable{trial_count}   = uad(trial_count,:);
        par.uacTable{trial_count}   = qdRefu(3,:);
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
dataLOG.qrTRAJ          = qrTRAJ;
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
