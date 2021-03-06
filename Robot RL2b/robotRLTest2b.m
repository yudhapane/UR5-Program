%robotRLTest2b is a script to test the RL-based additive compensator to 
% the UR5 robot. The learning process is done while following a straight 
% line trajectory. RL is an actor-critic with radial basis functions as 
% the approximator
% 
% Yudha Prawira Pane (c)
% created on      : Apr-20-2015
% last updated on : Apr-29-2015	

%% Start ups and initialization
format long;
clc; close all;
clearvars -except arm UR5 
loadParamsUR5_2b;
load('wtrajMemoryfixed.mat');

if (~exist('arm','var'))
    clc; clear; close all;
    startup; 
    arm = URArm();
    IP_ADDRESS = '192.168.1.50';
    arm.fopen(IP_ADDRESS);   
    arm.update();
end
arm.moveJoints(params.qHome,1,2,2);
pause(2);

%% Trajectory and experiments-related variables
EXPERIMENT_TIME = 5;
SAMPLING_TIME   = params.ts;
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
qdotRef(:,k)    = zeros(6,1);               % the nominal reference velocity
u(k)            = 0;                        % action calculated by actor
qTable(:,k)     = zeros(6,1);               % joint position
wTable(:,k)   	= zeros(6,1);               % tool position
wdotTable(:,k)  = zeros(6,1);               % tool velocity
jointsVel(:,k) 	= zeros(6,1);               % joint velocity
delta(k)        = 0;                        % temporal difference
Delta(k)        = 0;    
rz(k)           = 0;
rzdot(k)        = 0;
penalty 		= 0;
uOvershoot      = 0;

%% Volatile memory
volDelta_u      = 0;
voluad          = 0;
volu            = 0;
volqdotRef      = zeros(6,1);
volqdotRefu     = zeros(6,1);
volqTable       = zeros(6,1);               % joint position
volwTable       = zeros(6,1);               % tool position
volwdotTable    = zeros(6,1);        		% tool velocity

%% Misc variables
wdotPlot = linspace(params.zdotllim, params.zdotulim, size(wrefTRAJ,2));

%% Start learning trials
for counter = 1:params.Ntrial
    arm.moveJoints(qrefTRAJ(:,1),5,10,1); % move the robot to the first position of the qrefTRAJ safely
    pause(1.2);
    arm.update();        
    qTable(:,1)         = arm.getJointsPositions();    
    wTable(:,1)         = arm.getToolPositions();
    wdotTable(:,1)      = arm.getToolSpeeds();
    if (wdotTable(3,1) > params.zdotulim)
        wdotTable(3,1)  = params.zdotulim;
    end
    if (wdotTable(3,1) < params.zdotllim)
        wdotTable(3,1)  = params.zdotllim;
    end
        
    urand(1)    = 0;	% initialize input and exploration signal
    r(1)        = 0;	% initialize reward/cost
    rz(1)       = 0;
    rzdot(1)    = 0;
    e_c         = 0;
    penalty     = 0;
    
    if( mod(counter, params.rlPause) == 0)
        pause(0.1);
    end
%     if counter > 30
%         params.alpha_a1 = 1e-06;
%     end
%     TIMER = tic;
    for i=1:N-1
        if (counter > oldCounter)          
            oldCounter  = counter;
            iterIndex(k)= 1;      	% new trial detected, set to 1
        else
            iterIndex(k)= 0;     	% always set to zero unless new trial is detected           
        end

        %% Calculate RL-based additive compensator
        if mod(i,params.expSteps) == 0 	% explore only once every defined time steps            
            urand(i)  = params.varRand*randn;          
            % amplify the random action variance
%             if counter > 50 && i > params.idxRand;
%                 urand(i)  = params.ampRand*params.varRand*randn;  
%                 params.expSteps = 1;
%             end
        else
            urand(i)  = 0;
        end
        Delta_u             = urand(i);   

        u(i)                = actorUR5_2b([wTable(3,i); wdotTable(3,i)], params); 
        [uad(counter,i), uSel(i)]   = satUR5_2(u(i), params, Delta_u);  
%         Delta_u             = uad(i) - u(i);
        volDelta_u(i)       = Delta_u;

        %% Combine the nominal control input with the RL-based compensator
 
        if (wTable(3,i) > params.osLimit) 
            if uad(counter,i) <= uad(counter-1,i)
%                 safeUadIdx(i)  = counter;
                uad(counter,i)  = uad(counter-1, i);
                penalty         = penalty + 1;
                uOvershoot(i)   = 1;        % toggle boundary violation flag ON
            end
        else
            penalty = penalty - 1;
            penalty = max([0, penalty]);
            uOvershoot(i)       = 0;        % boundary violation flag OFF
        end
					
			
        qdotRef(:,i)        = (qrefTRAJ(:,i+1) - qrefTRAJ(:,i))/SAMPLING_TIME;
        qdotRefu(:,i)       = qdotRef(:,i) + [0; 0; uSel(i)*uad(counter,i); 0; 0; 0];       
        
        %% Apply control input, measure state, receive reward        
        tic
        arm.setJointsSpeed(qdotRefu(:,i),params.acc,4*SAMPLING_TIME);

        while(toc<SAMPLING_TIME)
        end
        arm.update();  
        qTable(:,i+1)       = arm.getJointsPositions();    
        wTable(:,i+1)       = arm.getToolPositions();      
        wdotTable(:,i+1)    = arm.getToolSpeeds();
        if (wdotTable(3,i+1) > params.zdotulim)
            wdotTable(3,i+1)  = params.zdotulim;
        end
        if (wdotTable(3,i+1) < params.zdotllim)
            wdotTable(3,i+1)  = params.zdotllim;
        end
        

%         abs(wTable(3,k+1)-wrefTRAJ(3,i+1))
%         if(rms(qTable(:,k+1)-qrefTRAJ(:,i+1))>0.002)
%             arm.setJointsSpeed(zeros(6,1), 0, 10);
%             pause(10);
%             error('robot deviates from trajectory!');     
%         end
%         if i>1
       	[r(i+1), rz(i+1), rzdot(i+1)] = costUR5_2b(wTable(3,i), wdotTable(3,i),  wrefTRAJ(3,i), params); 	% calculate the immediate cost 
%         else
%             [r(i+1)      = costUR5_2(wTable(3,i), wdotTable(3,i),  uad(i), 0, wrefTRAJ(3,i), params); 	% calculate the immediate cost 
%         end
        %% Compute temporal difference & eligibility trace
        V(i)        = criticUR5_2b([wTable(3,i); wdotTable(3,i)], params);                    	% V(x(k))
        V(i+1)      = criticUR5_2b([wTable(3,i+1); wdotTable(3,i+1)], params);                	% V(x(k+1))
        delta(i)    = r(i+1) + params.gamma*V(i+1) - V(i);                  % temporal difference 
%         Delta(k)    = delta(i);
        %e_c         = params.gamma*params.lambda*e_c + rbfUR5_2b([wTable(3,i); wdotTable(3,i)], params);

        %% Update critic and actor parameters
        % Update actor and critic
%         if counter < params.Ntrial
            alpha_c1 = params.alpha_c1/(1+abs(1/300*delta(i)));         % adjust actor learning rate
            alpha_a1 = params.alpha_a1/(1+abs(1/300*delta(i)));         % adjust actor learning rate

%             drbfa(:,k) = rbfUR5_2b([wTable(3,i); wdotTable(3,i)], params, 'actor');
% 			drbfc(:,k) = rbfUR5_2b([wTable(3,i); wdotTable(3,i)], params, 'critic');

			drbfa = rbfUR5_2b([wTable(3,i); wdotTable(3,i)], params, 'actor');
			drbfc = rbfUR5_2b([wTable(3,i); wdotTable(3,i)], params, 'critic');
            params.theta	= params.theta + alpha_c1*delta(i)*drbfc;   	% critic            
            if params.actorSelect == 1
                params.phi      = params.phi + alpha_a1*delta(i)*uSel(i)*(1-uOvershoot(i))*drbfa;      % actor 1 
            else
                params.phi      = params.phi + params.alpha_a2*Delta_u*delta(i)*drbfa;   % actor 1 
            end
            
%             if urand(i) == 0
%                 params.phi      = params.phi + params.alpha_a1*delta(i)*drbfa;      % actor 1 
%             else
%         else
%             params.theta	= params.theta + params.alpha_c2*delta(i)*rbfUR5_2([wTable(3,i); wdotTable(3,i)], params);                 % critic
%             params.phi      = params.phi + params.alpha_a2*Delta_u*delta(i)*uSel(i)*rbfUR5_2([wTable(3,i); wdotTable(3,i)],params);   % actor 1 
%             params.varRand  = 8e-4;
%         end
%         Phi(:,k+1)      = params.phi;    % save the parameters to memory
%         Theta(:,k+1)    = params.theta;

        %% Compute return 
        Ret(counter,i+1)    = params.gamma*Ret(counter,i) + r(i+1);  % update return value

        %% Update time step and initial state
        k   = k+1;          % update index variable
    end
%     toc(TIMER)
    % Plotting purpose
    if mod(counter,params.plotSteps) == 0
        clf;
        figure(1); title(['Iteration: ' int2str(counter)]);
        
        % critic
%         subplot(4,4,[1,2]); 
        positionVector1 = [0.05, 0.8, 0.43, 0.17];
        subplot('Position',positionVector1)
        plotOut = plotrbfUR5_2b(params, 'critic', params.plotopt); title(['\bf{CRITIC}  Iteration: ' int2str(counter)]); colorbar;
        xlabel('$z  \hspace{1mm}$ [mm]','Interpreter','Latex'); ylabel('$\dot{z}  \hspace{1mm}$ [mm]','Interpreter','Latex'); zlabel('$V(z)$ \hspace{1mm} [-]','Interpreter','Latex'); %colorbar 
        hold on; plot(wTable(3,:), wdotTable(3,:), 'r.');
        plot(wrefTRAJ(3,:),wdotPlot,'b');
        
        % actor
%         subplot(4,4,[5,6]);  
        positionVector2 = [0.05, 0.545, 0.43, 0.176];
        subplot('Position',positionVector2)
        plotOut = plotrbfUR5_2b(params, 'actor', params.plotopt); title('\bf{ACTOR}');  colorbar;
        hold on; plot(wTable(3,:), wdotTable(3,:), 'r.'); 
        plot(wrefTRAJ(3,:),wdotPlot,'b');
        xlabel('$z  \hspace{1mm}$ [mm]','Interpreter','Latex'); ylabel('$\dot{z}  \hspace{1mm}$ [mm]','Interpreter','Latex'); zlabel('$\pi(z)$ \hspace{1mm} [-]','Interpreter','Latex'); %colorbar 
        
        % position
%         subplot(4,4,[9,10]); 
        positionVector3 = [0.05, 0.305, 0.43, 0.16];
        subplot('Position',positionVector3)
        plot(time, 1000*wTable(3,:)); hold on; plot(time, 1000*wrefTRAJ(3,:), 'r'); plot(time,1000*wtrajMemory(3,:), 'g');
        xlabel('time (seconds)'); ylabel('Z position (mm)'); xlim([0 params.t_end]); title('reference (red), RL (blue), no-RL (green)');       
        
        % velocity
%         subplot(4,4,[13,14]);
        positionVector4 = [0.05, 0.063, 0.43, 0.16];
        subplot('Position',positionVector4)
        plot(time, 1000*wdotTable(3,:));
        xlabel('time (seconds)'); ylabel('Z velocity (mm/s)'); xlim([0 params.t_end]); title('Z velocity trajectory');                      
        
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
        plotyy(1:1:params.Ntrial, sum(Ret,2), 51:1:params.Ntrial, tempRet(51:end)); title('\bf{Return}');   %ylim([-10 5]);
        xlabel('trials');
                  

        % additive input and exploration
%         subplot(4,4,[11,12]); 
        positionVector7 = [0.55, 0.063, 0.43, 0.24];
        subplot('Position',positionVector7)
        plot(time(1:end-1), u, time(1:end-1), volDelta_u, 'r'); 
        xlabel('time (seconds)'); ylabel('additive input (rad/s)'); xlim([0 params.t_end]); title('Actor (b) & exploration (r)');       
           
        pause(0.2);
    end
    pause(0.1);
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
