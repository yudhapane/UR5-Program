%UR5STARTUP  start up script before working on the robot
% 
% Yudha Prawira Pane (c)
% created on      : Jan-14-2015
% last updated on : Mar-23-2015


%% Preparation
format long;
clc; clear; close all;
warning off;

%% Robot connection & preparation
if (~exist('arm','var'))
    arm = URArm();
    IP_ADDRESS = '192.168.1.50';
    arm.fopen(IP_ADDRESS);   
    arm.update();
end

UR5 = ur5Kinematics;	% define the robot kinematic   

%% Define useful variables
defSpeed    = 0.2;  % default speed
defAcc      = 1;    % default acceleration 

% Joint space [rad]
qHome = [-0.1921 -1.8577 2.0274 -0.1697 1.3787 3.1416]; 

% Tool space [m]
pHomeH  = UR5.fkine(qHome);
pHome   = pHomeH(1:3, end);
pHomeR  = [0 0 -1; 1 0 0; 0 -1 0];
pHomeH  = [pHomeR pHome; 0 0 0 1];


arm.moveJoints(qHome, defSpeed, defAcc);    % move robot to home position
UR5.plot(qHome);                            % visualize the robot



