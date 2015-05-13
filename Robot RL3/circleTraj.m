%CIRCLETRAJ generates the circular trajectory lying on X-Y plane
% 
% Yudha Prawira Pane (c)
% created on      : May-12-2015
% last updated on : May-12-2015	

clc; close all; 
clearvars -except arm UR5 ;

%% Generate trajectory
EXPERIMENT_TIME = 30;
SAMPLING_TIME   = 1/125;
N               = EXPERIMENT_TIME/SAMPLING_TIME; % number of samples 
time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
time(end)       = [];

% workspace trajectory
w0 = [-0.4000 -0.1000 0.3000 -1.210855329176504 -1.209614067678408 1.208165992685823]';
r  = 0.05;  % radius
Theta   = linspace(0, 2*pi, N);
wrTraj  = [r*cos(Theta); r*sin(Theta); zeros(4,N)];
wrTraj  = repmat(w0,1,N)+wrTraj;

arm.moveTool(w0, 1, 1, 3); pause(4);
arm.update();

% joint space trajectory 
qrTraj(:,1)   = arm.getJointsPositions();
for i = 1:N-1
    ds              = wrTraj(:,i+1) - wrTraj(:,i);    

    % Calculate the joint space trajectory using inverse jacobian    
    J               = UR5.jacob0(qrTraj(:,i));
    dq              = J\ds;
    qrTraj(:,i+1)   = qrTraj(:,i) + dq;
end

arm.moveTool(wrTraj(:,1), 1, 1, 3);
pause(4);
timer = tic;
for i = 1:N-1
    arm.update();
    qTRAJ(i,:)  = arm.getJointsPositions();
    wTRAJ(i,:)  = arm.getToolPositions();    
    unom(i,:) 	= (qrTraj(:,i+1)-qrTraj(:,i))/SAMPLING_TIME;     
    tic
    arm.setJointsSpeed(unom(i,:), 20, 2*SAMPLING_TIME);
    while(toc<SAMPLING_TIME)
    end
end
toc(timer)
figure;
plot(wTRAJ(:,2), wTRAJ(:,1)); hold on; plot(wrTraj(2,:), wrTraj(1,:)); axis equal;
legend('trajectory', 'reference');
grid on;
% Save control input and reference trajectory
trajLib.qrTraj = qrTraj;
trajLib.wrTraj = wrTraj;
save('nomConInput_RL3', 'unom');
save('trajLib_RL3.mat', 'trajLib');

