<<<<<<< HEAD
%% Trajectory and experiments-related variables
EXPERIMENT_TIME = 5;
=======
% load necessary trajectories and poses
load('ur5PoseLib.mat');

arm.moveJoints(ur5PoseLib.qHome,1,2,2);   	% move robot to the home position
pause(2.1); 

%% Trajectory and experiments-related variables
EXPERIMENT_TIME = 10;
>>>>>>> programmed and tested RL-based reference shaping
SAMPLING_TIME   = 1/125;
N               = EXPERIMENT_TIME/SAMPLING_TIME; % number of samples 
time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
time(end)       = [];

wrefTRAJ        = zeros(6,N);   % reference trajectory, will be populated in the iteration
qrefTRAJ        = zeros(6,N);   % joint space reference trajectory

%% Create the trajectory

arm.update();
ts              = 1/125;
initPos         = arm.getToolPositions();
q0              = arm.getJointsPositions();
initOrientation = initPos(4:end);
qTRAJ           = zeros(6,1);
<<<<<<< HEAD
offset              = [-0.20 0 0 0 0 0]';
=======
qdTRAJ          = zeros(6,1);
offset              = [-0.25 0 0 0 0 0]';
>>>>>>> programmed and tested RL-based reference shaping
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
<<<<<<< HEAD
% time    = 0;
arm.update();
arm.moveTool(wrefTRAJ(:,1),0.5,1,5);
pause(6);
arm.moveTool(wrefTRAJ(:,end),0.5,1,5)
t = tic;
for i = 1 :625
    tic
    arm.update();
    wTRAJ2(:,i) = arm.getToolPositions();
=======

% time    = 0;

close all; clear wTRAJ2 qTRAJ qdTRAJ;
figure;
for j = 1:5
arm.update();
arm.moveTool(wrefTRAJ(:,1),1,3,5);
pause(6);
arm.moveTool(wrefTRAJ(:,end),1,3,EXPERIMENT_TIME)
% arm.moveTool(wrefTRAJ(:,end),0.05,0.1);
t = tic;
for i = 1 :N
    tic
    arm.update();
    wTRAJ2(:,i) = arm.getToolPositions();
    qTRAJ(:,i)  = arm.getJointsPositions();
    qdTRAJ(:,i) = arm.getJointsSpeeds();
>>>>>>> programmed and tested RL-based reference shaping
    while toc<ts
    end
end
toc(t)
<<<<<<< HEAD
figure; 
plot(wTRAJ(3,:))
=======
plot(wTRAJ2(3,:)); hold on;
arm.setJointsSpeed(zeros(6,1), 1,0.5);
pause(0.5);
end
figure; plot(qTRAJ(1,:)); 
figure; plot(qTRAJ(2,:)); 
figure; plot(qTRAJ(3,:)); 
figure; plot(qTRAJ(4,:));
figure; plot(qdTRAJ(1,:)); 
figure; plot(qdTRAJ(2,:));
figure; plot(qdTRAJ(3,:));
figure; plot(qdTRAJ(4,:));
% figure; plot(cumsum(qdTRAJ(1,:)));
% figure; plot(cumsum(qdTRAJ(2,:)));
% figure; plot(cumsum(qdTRAJ(3,:)));
% figure; plot(cumsum(qdTRAJ(4,:)));
% end
>>>>>>> programmed and tested RL-based reference shaping
