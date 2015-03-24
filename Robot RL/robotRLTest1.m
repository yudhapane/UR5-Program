%robotRLTest1 is a script to test the RL-based additive compensator to 
% the UR5 robot. The learning process is done while following a straight 
% line trajectory. RL is an actor-critic with radial basis functions as 
% the approximator
% 
% Yudha Prawira Pane (c)
% created on      : Mar-23-2015
% last updated on : Mar-23-2015

%% Start ups and initialization
% format long;
close all;
% if (~exist('arm','var'))
%     clc; clear; close all;
%     startup; 
%     arm = URArm();
%     IP_ADDRESS = '192.168.1.50';
%     arm.fopen(IP_ADDRESS);   
%     arm.update();
% end
arm.moveJoints(params.qHome,1,2,3);
pause(3);

%% Trajectory and experiments-related variables
EXPERIMENT_TIME = 5;
SAMPLING_TIME   = 1/125;
N               = EXPERIMENT_TIME/SAMPLING_TIME; % number of samples 
time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
time(end)       = [];

wrefTRAJ        = zeros(6,N);   % reference trajectory, will be populated in the iteration
qrefTRAJ        = zeros(6,N);   % joint space reference trajectory

%% Create the trajectory
arm.update();
initPos         = arm.getToolPositions();
q0              = arm.getJointsPositions();
initOrientation = initPos(4:end);

offset              = [-0.20 0 0 0 0 0]';
finalPos            = initPos + offset;
discretizedOffset   = offset/N;

for i = 1:N
    wrefTRAJ(:,i) = initPos + (i-1)*discretizedOffset; % populate reference trajectory
end

qrefTRAJ(:,1) = q0;

% Generate joint space trajectory 
for i = 1:N-1
    ds = wrefTRAJ(:,i+1) - wrefTRAJ(:,i);    

    % Calculate the joint space trajectory using inverse jacobian    
    J = UR5.jacob0(qrefTRAJ(:,i));
    dq = J\ds;
    qrefTRAJ(:,i+1) = qrefTRAJ(:,i) + dq;
end


%% Non-volatile memory
k               = 1;                        % initialize index variable
Phi(:,k)        = params.phi;               % actor parameters memory
Theta(:,k)      = params.theta;             % critic parameters memory
Ret(:,k)        = zeros(params.Ntrial,1);   % return memory
V(k)            = 0;                        % initialize value function
uSel(k)         = 1;                        % actor parameter update selector, 1 for update & 0 for not update
iterIndex(k)    = 1;                        % iteration indexing. Useful for animation purpose   
oldCounter      = 1;
qdotRef(k)      = 0;                        % the nominal reference velocity
u(k)            = 0;                        % action calculated by actor
qTable(:,k)     = zeros(6,1);               % joint position
wTable(:,k)   	= zeros(6,1);               % tool position
jointsVel(:,k) 	= zeros(6,1);               % joint velocity
toolVel(:,k)  	= zeros(6,1);               % tool velocity

%% Volatile memory
volDelta_u  = 0;
voluad      = 0;
volqdotRef  = 0;
volu        = 0;


%% Start learning trials
for counter = 1:params.Ntrial
    arm.moveJoints(qTraj(:,1),2,3,3); % move the robot to the first position of the qTraj safely
    pause(3);
    arm.update();        
    qTable(:,k)     = arm.getJointsPositions();    
    wTable(:,k)     = arm.getToolPositions();
	volqTable(:,1)	= qTable(:,k);    
	volsTable(:,1)	= wTable(:,k);
        
    urand(k)    = 0;	% initialize input and exploration signal
    r(k)        = 0;	% initialize reward/cost
    e_c         = 0;
    
    for i=1:N-1
        if (counter > oldCounter)          
            oldCounter  = counter;
            iterIndex(k)= 1;      	% new trial detected, set to 1
        else
            iterIndex(k)= 0;     	% always set to zero unless new trial is detected           
        end

        %% Calculate RL-based additive compensator
        if mod(k,params.expSteps) == 0 	% explore only once every defined time steps
            urand(k)  = params.varRand*randn;                                      
        else
            urand(k)  = 0;
        end
        Delta_u             = urand(k);   

        w                   = qTable(:,k);  % get current joint position
        u(k)                = actorUR5_1(w(3), params); 
        volu(i)             = u(k);
        [uad(k), uSel(k)]   = satUR5_1(u(k), params, Delta_u);  
        Delta_u             = uad(k) - u(k); 
        volDelta_u(i)       = Delta_u;
        voluad(i)           = uad(k);  


        %% Combine the nominal control input with the RL-based compensator
        qdotRef(k)      = (qTraj(:,i+1) - qTraj(:,i))/SAMPLING_TIME;
        volqdotRef(i)   = qdotRef(k);
        qdotRefu(k)     = qdotRef(k) + uad(k);
        voldqdotRefu(i) = qdotRefu(k);
        
        %% Apply control input, measure state, receive reward        
        tic
        arm.setJointsSpeed(qdotRefu(k),acc,2*SAMPLING_TIME);

        while(toc<SAMPLING_TIME)
        end
        arm.update();  
        qTable(:,k+1)       = arm.getJointsPositions();    
        wTable(:,k+1)       = arm.getToolPositions();        
        volqTable(:,i+1)    = qTable(:,k+1);    
        volsTable(:,i+1)    = wTable(:,k+1);
        
        r(k+1)              = costUR5_1(wTable(3,k+1), wrefTRAJ(3,i+1), params); 	% calculate the immediate cost 
        
        %% Compute temporal difference & eligibility trace
        V(k)        = criticUR5_1(wTable(3,k), params);                    	% V(x(k))
        V(k+1)      = criticUR5_1(wTable(3,k+1), params);                	% V(x(k+1))
        delta(k)    = r(k+1) + params.gamma*V(k+1) - V(k);                  % temporal difference 
        e_c         = params.gamma*params.lambda*e_c + rbf(Q(1:2,k), params);

        %% Update critic and actor parameters
        % Update actor and critic
        params.theta	= params.theta + params.alpha_c*delta(k)*rbf(wTable(3,k), params);                 % critic
        params.phi      = params.phi + params.alpha_a*delta(k)*Delta_u*uSel(k)*rbf(wTable(3,k),params);   % actor 1 
           
        Phi(:,k+1)      = params.phi;    % save the parameters to memory
        Theta(:,k+1)    = params.theta;

        %% Compute return 
        Ret(counter,k+1)    = params.gamma*Ret(counter,k) + r(k+1);  % update return value

        %% Update time step and initial state
        k   = k+1;          % update index variable
    end

    % Plotting purpose
    if mod(counter,params.plotSteps) == 0
        clf;
        subplot(331); 
        plotOut = plotrbfUR5_1(params, 'critic'); title(['\bf{CRITIC}  Iteration: ' int2str(counter)]);
        xlabel('$\theta_1  \hspace{1mm}$ [rad]','Interpreter','Latex'); ylabel('$\theta_2$ \hspace{1mm} [rad/s]','Interpreter','Latex'); colorbar 
        subplot(332); 
        plotOut = plotrbfUR5_1(params, 'actor'); title('\bf{ACTOR}'); 
        xlabel('$\theta_1 \hspace{1mm}$ [rad]','Interpreter','Latex'); ylabel('$\theta_2$ \hspace{1mm} [rad/s]','Interpreter','Latex'); colorbar 
        subplot(333);
        plot(delta); title('\bf{Temporal difference}');% ylim([-10 5]);
        subplot(334);
        plot(sum(Ret,2)); title('\bf{Return}'); %ylim([-10 5]);
        pause(0.2);    
        subplot(335);
        plot(time,  volqdotRefu, time, voluad, 'r'); title('\bf{blue: uc    red: uad}');
        subplot(336);
        plot(time,  volqdotRef); title('\bf{Nominal controller no RL}'); 
        subplot(337); plot(time, wTable(2,:)); hold on; plot(time, wrefTRAJ(2,:), 'r');
        xlabel('time (seconds)'); xlim([0 params.t_end]); title('Workspace traj Y: reference (r), actual (b)');
        subplot(338); plot(time, wTable(3,:)); hold on; plot(time, wrefTRAJ(3,:), 'r'); plot(volClock,wtrajMemory(3,:), 'g');
        xlabel('time (seconds)'); xlim([0 params.t_end]); title('Workspace traj Z: reference (r), actual (b), nominal (g)');       
        subplot(339); plot(time, volu, time, volDelta_u, 'r'); 
        xlabel('time (seconds)'); xlim([0 params.t_end]); title('Actor (b) and random (r)');       

        pause(0.2);           
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
dataLOG.refTRAJ         = wrefTRAJ;
dataLOG.wTable          = wTable;
dataLOG.qrefTRAJ        = qrefTRAJ;
dataLOG.qTable          = qTable; 
dataLOG.ErrorX          = wrefTRAJ(1,:)-wTable(1,:);
dataLOG.ErrorY          = wrefTRAJ(2,:)-wTable(2,:);
dataLOG.ErrorZ          = wrefTRAJ(3,:)-wTable(3,:);
dataLOG.rmsX            = rms(wrefTRAJ(1,:)-wTable(1,:));
dataLOG.rmsY            = rms(wrefTRAJ(2,:)-wTable(2,:));
dataLOG.rmsZ            = rms(wrefTRAJ(3,:)-wTable(3,:));
dataLOG.MaxAbsX         = max(abs(wrefTRAJ(1,:)-wTable(1,:)));
dataLOG.MaxAbsY         = max(abs(wrefTRAJ(2,:)-wTable(2,:)));
dataLOG.MaxAbsZ         = max(abs(wrefTRAJ(3,:)-wTable(3,:)));

%% Display trajectory and errors
figure; subplot(211);
plot(time(:), wrefTRAJ(1,:), time(:), wTable(1,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory X-axis');
subplot(212);
plot(time(:), dataLOG.ErrorX);
title('trajectory error X-axis');

figure; subplot(211);
plot(time(:), wrefTRAJ(2,:), time(:), wTable(2,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory Y-axis');
subplot(212);
plot(time(:), dataLOG.ErrorY);
title('trajectory error Y-axis');

figure; subplot(211);
plot(time(:), wrefTRAJ(3,:), time(:), wTable(3,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory Z-axis');
subplot(212);
plot(time(:), dataLOG.ErrorZ);
title('trajectory error Z-axis');

%% Save data
savefolder = 'D:\Dropbox\TU Delft - MSc System & Control\Graduation Project (Thesis)\UR5 Robot\UR5 Programs\Robot Test\recorded data\';
save([savefolder 'dataLOG_' datestr(now,'dd-mmm-yyyy HH-MM-SS') '.mat'], 'dataLOG');
