% robotTestRun is a script to test the UR5 robot to execute a simple,
% straight line trajectory. First, the straight line workspace trajectory 
% along the X- axis will be generated. Afterwards, the joint space the 
% conversion is performed using inverse jacobian. At the end it will save 
% the reference, actual trajectories, errors, and plots.
% 
% Yudha Prawira Pane (c)
% created on      : Jan-12-2015
% last updated on : Mar-24-2015

%% Start ups and initialization
% format long;
close all;
if (~exist('arm','var'))
    clc; clear; close all;
    startup; 
    arm = URArm();
    IP_ADDRESS = '192.168.1.50';
    arm.fopen(IP_ADDRESS);   
    arm.update();
end

qHome = [-0.1921 -1.8577 2.0274 -0.1697 1.3787 3.1416]; 
arm.moveJoints(qHome,1,2,3);
pause(3);

%% Conditioning variables
EXPERIMENT_TIME = 5;
SAMPLING_TIME   = 1/125;
N               = EXPERIMENT_TIME/SAMPLING_TIME; % number of samples 
time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
time(end)       = [];

%% State variables
jointsPos   = zeros(6,1);
jointsVel   = zeros(6,1);
toolPos     = zeros(6,1);
toolVel     = zeros(6,1);
refTRAJ     = zeros(6,N);   % reference trajectory, will be populated in the iteration
qrefTRAJ    = zeros(6,N);

%% Create trajectories
% echo on;
arm.update();
initPos         = arm.getToolPositions();
q0              = arm.getJointsPositions();
initOrientation = initPos(4:end);

<<<<<<< HEAD
offset              = [-0.20 0 0 0 0 0]';
=======
offset              = [-0.15 0 0 0 0 0]';
>>>>>>> programmed and tested RL-based reference shaping
finalPos            = initPos + offset;
discretizedOffset   = offset/N;

for i = 1:N
    refTRAJ(:,i) = initPos + (i-1)*discretizedOffset; % populate reference trajectory
end

qrefTRAJ(:,1) = q0;

% Generate joint space trajectory using inverse jacobian
for i = 1:N-1
    ds = refTRAJ(:,i+1) - refTRAJ(:,i);    

    % Calculate the joint space trajectory using inverse jacobian    
    J = UR5.jacob0(qrefTRAJ(:,i));
    dq = J\ds;
    qrefTRAJ(:,i+1) = qrefTRAJ(:,i) + dq;
end

clc();
arm.update();
acceleration = 2;

% Move robot!!
[qTable, qdotTable, qedotTable, toolTRAJ] = trackReference1(arm, qrefTRAJ);
pause(1);


figure; plot(toolTRAJ(1,1:end-1)); hold on; plot(refTRAJ(1,1:end-1), 'r');
title('tool trajectory X-axis');
legend('actual trajectory', 'reference trajectory');

figure; plot(toolTRAJ(2,1:end-1)); hold on; plot(refTRAJ(2,1:end-1), 'r');
legend('actual trajectory', 'reference trajectory');
title('tool trajectory Y-axis');

figure; plot(toolTRAJ(3,1:end-1)); hold on; plot(refTRAJ(3,1:end-1), 'r');
legend('actual trajectory', 'reference trajectory');
title('tool trajectory Z-axis');

% t = tic;
% for i = 1:N
%     tic
%     arm.update(); % update the robot's state
%     toolPos = arm.getToolPositions();
% 
%     qdotRef = dq/SAMPLING_TIME       
%     arm.setJointsSpeed(qdotRef, acceleration, 2*SAMPLING_TIME); 
% %     arm.moveTool(toolPosInit+i*discretizedOffset, speed, acceleration, 2*SAMPLING_TIME);
% %     arm.setToolSpeed(discretizedOffset/SAMPLING_TIME,acceleration,2*SAMPLING_TIME);    
%     while (toc<SAMPLING_TIME)
%     end    
% end
% 
% toc(t)
pause(1);
arm.update();

%% Data Logging
err                     = arm.getToolPositions()-finalPos;
dataLOG.Name            = 'Robot Log Data';
dataLOG.Notes           = ['Created on: ' datestr(now) '   The robot was commanded using setJointsSpeed with constant offset'];
dataLOG.SamplingTime    = SAMPLING_TIME;
dataLOG.Time            = time;
dataLOG.refTRAJ         = refTRAJ;
dataLOG.toolTRAJ        = toolTRAJ;
dataLOG.qrefTRAJ        = qrefTRAJ;
dataLOG.qTRAJ           = qTable; 
dataLOG.ErrorX          = refTRAJ(1,:)-toolTRAJ(1,:);
dataLOG.ErrorY          = refTRAJ(2,:)-toolTRAJ(2,:);
dataLOG.ErrorZ          = refTRAJ(3,:)-toolTRAJ(3,:);
dataLOG.rmsX            = rms(refTRAJ(1,:)-toolTRAJ(1,:));
dataLOG.rmsY            = rms(refTRAJ(2,:)-toolTRAJ(2,:));
dataLOG.rmsZ            = rms(refTRAJ(3,:)-toolTRAJ(3,:));
dataLOG.MaxAbsX         = max(abs(refTRAJ(1,:)-toolTRAJ(1,:)));
dataLOG.MaxAbsY         = max(abs(refTRAJ(2,:)-toolTRAJ(2,:)));
dataLOG.MaxAbsZ         = max(abs(refTRAJ(3,:)-toolTRAJ(3,:)));

%% Display trajectory and errors
figure; subplot(211);
<<<<<<< HEAD
plot(time(:), refTRAJ(1,:), time(:), toolTRAJ(1,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory X-axis');
subplot(212);
plot(time(:), dataLOG.ErrorX);
title('trajectory error X-axis');

figure; subplot(211);
plot(time(:), refTRAJ(2,:), time(:), toolTRAJ(2,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory Y-axis');
subplot(212);
plot(time(:), dataLOG.ErrorY);
title('trajectory error Y-axis');

figure; subplot(211);
plot(time(:), refTRAJ(3,:), time(:), toolTRAJ(3,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory Z-axis');
subplot(212);
plot(time(:), dataLOG.ErrorZ);
title('trajectory error Z-axis');
=======
plot(time(:), 1000*refTRAJ(1,:), time(:), 1000*toolTRAJ(1,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory X-axis [mm]'); grid on; 
subplot(212);
plot(time(:), 1000*dataLOG.ErrorX);
title('trajectory error X-axis'); grid on;

figure; subplot(211); 
plot(time(:), 1000*refTRAJ(2,:), time(:), 1000*toolTRAJ(2,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory Y-axis [mm]'); grid on;
subplot(212);
plot(time(:), 1000*dataLOG.ErrorY);
title('trajectory error Y-axis [mm]'); grid on;

figure; subplot(211);
plot(time(:), 1000*refTRAJ(3,:), time(:), 1000*toolTRAJ(3,:));
legend('reference traj', 'tool traj'); title('reference vs actual trajectory Z-axis [mm]'); grid on;
subplot(212);
plot(time(:), 1000*dataLOG.ErrorZ);
title('trajectory error Z-axis [mm]'); grid on;
>>>>>>> programmed and tested RL-based reference shaping

%% Save data
savefolder = 'D:\Dropbox\TU Delft - MSc System & Control\Graduation Project (Thesis)\UR5 Robot\UR5 Programs\Robot Test\recorded data\';
save([savefolder 'dataLOG_' datestr(now,'dd-mmm-yyyy HH-MM-SS') '.mat'], 'dataLOG');
