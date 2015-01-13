% robotTestRun is a script to test the UR5 robot to execute a simple
% trajectories
% 
% Yudha Prawira Pane (c)
% Jan-12-2015

%% Start ups and initialization
format long;
if (~exist('arm','var'))
    clc; clear; close all;
    startup; 
    arm = URArm();
    IP_ADDRESS = '192.168.1.50';
    arm.fopen(IP_ADDRESS);   
    arm.update();
end

%% Conditioning variables
EXPERIMENT_TIME = 2;
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

offset = [-0.10 0 0 0 0 0]';
discretizedOffset = offset/N;


clc();
arm.update();
% time = 5;
speed = 0.05;
acceleration = 10;
toolPosInit = arm.getToolPositions(); % get current tool position
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
    arm.update(); % update the robot's state
    arm.moveTool(toolPosInit+i*discretizedOffset, speed, acceleration);
    
    while (toc<SAMPLING_TIME)
    end
end
toc(t)
arm.update();
arm.getToolPositions()-toolPosInit;


