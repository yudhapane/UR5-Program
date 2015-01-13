% robotTestRun is a script to test the UR5 robot to execute a simple
% trajectories
% 
% Yudha Prawira Pane (c)
% Jan-12-2015

%% Start ups and initialization

if (~exist('arm','var'))
    clc; clear; close all;
    startup; 
    arm = URArm();
    IP_ADDRESS = '192.168.1.50';
    arm.fopen(IP_ADDRESS);   
    arm.update();
end

%% Conditioning variables
EXPERIMENT_TIME = 3;
SAMPLING_TIME = 0.01;
N = EXPERIMENT_TIME/SAMPLING_TIME; % number of samples 

%% State variables
jointsPos = zeros(6,1);
jointsVel = zeros(6,1);
toolPos = zeros(6,1);
toolVel = zeros(6,1);

%% Create trajectories
% echo on;
arm.update();
initPos = arm.getToolPositions();
initPosition = initPos(1:3);
initOrientation = initPos(4:end);
TOOL_ORIENTATION = initOrientation; % constant tool's orientation

offset = [-0.01 0 0 0 0 0]';
discretizedOffset = offset/N;
speed = 0.5;
acceleration = 0.1;
time = 5;

% clc();
% arm.update();
% offset = [0.1 0 0 0 0 0]';
% time = 5;
% toolPos = arm.getToolPositions()
% tic
% arm.moveTool(toolPos+offset, speed, 1, time);
% while (toc<time)
% end
% toc
% arm.update();
% deltaPos = arm.getToolPositions()-toolPos

for i = 1:N
    tic
    arm.update(); % update the robot's state
    jointsPos = arm.getJointsPositions();
    jointsVel = arm.getJointsSpeeds();
    toolPos = arm.getToolPositions();
    toolVel = arm.getToolSpeeds();        
    
    arm.moveTool(toolPos+discretizedOffset, speed, acceleration, time);
    
    while (toc<SAMPLING_TIME)
    end
    t = tic;
%     pause
end
    

