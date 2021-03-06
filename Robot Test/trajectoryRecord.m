%TRAJECTORYRECORD   record robot's trajectories. 
% TRAJECTORYRECORD(OPT) generates trajectory, command robots, and record
% the state in real time. 
% 
% TRAJECTORYRECORD(OPT) record trajectory with options OPT. 
% OPT == 1 for recording only purpose e.g. to record while being commanded
% by teaching pendant 
% Yudha Prawira Pane (c)
% Jan-13-2015

%% Initialization
function [] = trajectoryRecord(opt)
    if nargin < 1
        error 'Not enough input argument(s), please fill in integer [1..3]'
    end
    
    close all; clc;
    if (~exist('arm','var')) % connect if the robot is not defined yet
        startup; 
        arm = URArm();
        IP_ADDRESS = '192.168.1.50';
        arm.fopen(IP_ADDRESS);   
        arm.update();
    end
    
    %% Conditioning variables
    EXPERIMENT_DURATION = 15;
    SAMPLING_TIME = 0.01;
    NUMBER_OF_SAMPLES = EXPERIMENT_DURATION/SAMPLING_TIME; % number of samples 

    %% Define variables
    jointsPos = zeros(6,1);
    jointsVel = zeros(6,1);
    toolPos = zeros(6,1);
    toolVel = zeros(6,1);
    toolPosRecord = zeros(8,NUMBER_OF_SAMPLES);
    offset = [-0.1 0 0 0 0 0]';
    speed = 0.5;
    acceleration = 0.1;
    time = 5;
    test = '2';

    if opt==1 
        %% Record trajectories with moveTool function (using Pendant)
        disp('Recording tool position trajectories ...'); 
        t = tic;
        for i=1:NUMBER_OF_SAMPLES
            tic;
            arm.update();
            toolPosRecord(3:8,i) = arm.getToolPositions();
            if(i==10/SAMPLING_TIME)
                disp('10 seconds already passed');
            end
            while (toc<SAMPLING_TIME)
            end   
            toolPosRecord(1:2,i) = [toc; i];    
        end
        toc(t)

        disp('Done!');
        toolPosRecMM(3:8,:) = toolPosRecord(3:8,:)*1000; % convert to mm
        toolPosRecMM(1:2,:) = toolPosRecord(1:2,:);

        figure;
        subplot(5,1,1);
        plot(toolPosRecMM(1,:),'.'); xlabel('time samples'); ylabel('Sampling time');
        subplot(5,1,2);
        plot(toolPosRecMM(2,:),'.'); xlabel('time samples'); ylabel('Sample counts');
        subplot(5,1,3);
        plot(toolPosRecMM(3,:)); xlabel('time samples'); ylabel('X position');
        subplot(5,1,4);
        plot(toolPosRecMM(4,:)); xlabel('time samples'); ylabel('Y position');
        subplot(5,1,5);
        plot(toolPosRecMM(5,:)); xlabel('time samples'); ylabel('Z position');

        save(['recorded data\data',test,'.mat'],'toolPosRec');
    elseif opt == 2
        %% Create trajectory, command robot, and record data (using marco's method)
        arm.update();
        jointsPos = arm.getJointsPositions();
        jointsSpeed = arm.getJointsSpeed();
%         arm.setJointsSpeed(
        
    end
