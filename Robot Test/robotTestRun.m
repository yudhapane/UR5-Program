% robotTestRun is a script to test the UR5 robot to execute a simple
% trajectories
% 
% Yudha Prawira Pane (c)
% Jan-12-2015

%% Start ups and initialization
% format long;
if (~exist('arm','var'))
    clc; clear; close all;
    startup; 
    arm = URArm();
    IP_ADDRESS = '192.168.1.50';
    arm.fopen(IP_ADDRESS);   
    arm.update();
end
arm.moveJoints(qHome,1,2);
pause(2.5);

%% Conditioning variables
EXPERIMENT_TIME = 10;
SAMPLING_TIME = 1/125;
N = EXPERIMENT_TIME/SAMPLING_TIME; % number of samples 
time = 0:SAMPLING_TIME:EXPERIMENT_TIME; time(end) = [];

%% State variables
jointsPos = zeros(6,1);
jointsVel = zeros(6,1);
toolPos = zeros(6,1);
toolVel = zeros(6,1);
refTRAJ = zeros(6,N); % reference trajectory, will be populated in the iteration
toolTRAJ = zeros(6,N);

%% Create trajectories
% echo on;
arm.update();
initPos = arm.getToolPositions();
initPosition = initPos(1:3);
initOrientation = initPos(4:end);
TOOL_ORIENTATION = initOrientation; % constant tool's orientation

offset = [-0.40 0 0 0 0 0]';
finalPos = initPos + offset;
discretizedOffset = offset/N;

for i = 1:N
    refTRAJ(:,i) = initPos + (i-1)*discretizedOffset; % populate reference trajectory
end

clc();
arm.update();
acceleration = 0.1;
% toolPosInit = arm.getToolPositions(); % get current tool position
% tic
% arm.moveTool(toolPos+offset, speed,acceleration);
% while (toc<time)
% end
% toc
% arm.update();
% deltaPos = arm.getToolPositions()-toolPos
t = tic;
for i = 1:N
    tic
    
    i
    arm.update(); % update the robot's state
    toolPos = arm.getToolPositions();
    ds = refTRAJ(:,i+1) - toolPos
    
    % Calculate inverse jacobian
    q = arm.getJointsPositions();
    J = UR5.jacob0(q);
    dq = inv(J)*ds
    qdotRef = dq/SAMPLING_TIME       
%     arm.setJointsSpeed(qdotRef, acceleration, 2*SAMPLING_TIME); 
    pause
%     arm.moveTool(toolPosInit+i*discretizedOffset, speed, acceleration, 2*SAMPLING_TIME);
%     arm.setToolSpeed(discretizedOffset/SAMPLING_TIME,acceleration,2*SAMPLING_TIME);    
    while (toc<SAMPLING_TIME)
    end    
end

toc(t)
pause(1);
arm.update();

%% Error Calculation
err = arm.getToolPositions()-finalPos;
posERR.Name = 'Tool Position Error Data';
posERR.Notes = ['Created on: ' datestr(now) '   The robot was commanded using setToolSpeed with constant offset'];
posERR.SamplingTime = SAMPLING_TIME;
posERR.Time = time;
posERR.ErrorX = refTRAJ(1,:)-toolTRAJ(1,:);
posERR.ErrorY = refTRAJ(2,:)-toolTRAJ(2,:);
posERR.ErrorZ = refTRAJ(3,:)-toolTRAJ(3,:);
posERR.rmsX = rms(refTRAJ(1,:)-toolTRAJ(1,:));
posERR.rmsY = rms(refTRAJ(2,:)-toolTRAJ(2,:));
posERR.rmsZ = rms(refTRAJ(3,:)-toolTRAJ(3,:));
posERR.MaxAbsX = max(abs(refTRAJ(1,:)-toolTRAJ(1,:)));
posERR.MaxAbsY = max(abs(refTRAJ(2,:)-toolTRAJ(2,:)));
posERR.MaxAbsZ = max(abs(refTRAJ(3,:)-toolTRAJ(3,:)));

%% Display errors
figure; subplot(2,1,1);
plot(time, refTRAJ(1,:), time, toolTRAJ(1,:));
legend('reference traj', 'tool traj');
subplot(2,1,2);
plot(time, abs(posERR.ErrorX));

figure; subplot(2,1,1);
plot(time, refTRAJ(2,:), time, toolTRAJ(2,:));
legend('reference traj', 'tool traj');
subplot(2,1,2);
plot(time, abs(posERR.ErrorY));

figure; subplot(2,1,1);
plot(time, refTRAJ(3,:), time, toolTRAJ(3,:));
legend('reference traj', 'tool traj');
subplot(2,1,2);
plot(time, abs(posERR.ErrorZ));

%% Save data
savefolder = 'D:\Dropbox\TU Delft - MSc System & Control\Graduation Project (Thesis)\UR5 Robot\UR5 Programs\Robot Test\recorded data\';
save([savefolder 'posERR_' datestr(now,'dd-mmm-yyyy HH-MM-SS') '.mat'], 'posERR');
