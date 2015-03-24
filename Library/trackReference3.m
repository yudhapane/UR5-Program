function [qTable, qdotTable, qedotTable, sTable] = trackReference3(varargin)
%TRACKREFERENCE3  execute joint space trajectory
% 
% trackReference3(varargin) executes a joint space trajectory 
% tracking routine with 3 possibilities. The difference between this
% function and trackReference1 is the possibility to input varying number
% of parameters
% 1. trackReference3('ROBOT', ARM, 'QTRAJ', QTRAJ) command robot ARM to 
% execute joint position trajectory QTRAJ . 
% 
% Yudha Prawira Pane (c)
% created on      : Feb-13-2015
% last updated on : Mar-23-2015

    % Preparation
    opt = struct('robot',0,'qtraj',0,'qdottraj',0,'ts',0.008);

    %# read the acceptable names
    optNames = fieldnames(opt);
    
    nArgs = length(varargin);
    if round(nArgs/2)~=nArgs/2
       error('EXAMPLE needs propertyName/propertyValue pairs')
    end

    for pair = reshape(varargin,2,[]) %# pair is {propName;propValue}
       inpName = lower(pair{1}); %# make case insensitive

       if any(strcmp(inpName,optNames))
          %# overwrite opt. If you want you can test for the right class here
          %# Also, if you find out that there is an opt you keep getting wrong,
          %# you can use "if strcmp(inpName,'problemOption'),testMore,end"-statements
          opt.(inpName) = pair{2};
       else
          error('%s is not a recognized parameter name',inpName)
       end
    end
    
    % Assign input parameters into variables
    arm = opt.robot;
    SAMPLING_TIME = opt.ts;
    qTraj = opt.qtraj;
    qdotTraj = opt.qdottraj;
    acc = 20;
    
    if nargin < 2
        error 'Not enough input argument(s)'
    end
    
%     acc = input('enter desired acceleration: ')
    if(isempty(acc))
        acc = 10;
    end
    if size(qTraj,1)~=6     % assuming N steps >> DOF
        qTraj = qTraj';     % trajectory must be column-wise
    end
    N = size(qTraj,2);      % get number of steps
    axis([0 N -3.14 3.14]);

    % Define variables 
    qHome = [-0.1921 -1.8577 2.0274 -0.1697 1.3787 3.1416];
    qTable = zeros(6,N);
    sTable = zeros(6,N);

    arm.moveJoints(qHome,2,3); % move the robot to home position first
    arm.moveJoints(qTraj(:,1),2,3); % move the robot to the first position of the qTraj safely
    pause(2.5);
    tcount = tic;
    for i=1:N-1
        tic
        arm.update();

        % DIRECT SPEED COMMAND
        qdotRef = (qTraj(:,i+1) - qTraj(:,i))/SAMPLING_TIME;
        arm.setJointsSpeed(qdotRef,acc,2*SAMPLING_TIME);

        while(toc<SAMPLING_TIME)
        end
        qrec = arm.getJointsPositions();    
        srec = arm.getToolPositions();
        qTable(:,i) = qrec;
        sTable(:,i) = srec;
    end
    arm.update();
    qTable(:,N) = arm.getJointsPositions();
    disp('Duration of tracking:');
    toc(tcount)
    time = 1:1:N;
    clf;
    subplot(3,1,1);
    plot(time, rad2deg(qTable(1,:))); hold; plot(time, rad2deg(qTraj(1,:)),'r');
    subplot(3,1,2);
    plot(time, rad2deg(qTable(2,:))); hold; plot(time, rad2deg(qTraj(2,:)),'r');
    subplot(3,1,3);
    plot(time, rad2deg(qTable(3,:))); hold; plot(time, rad2deg(qTraj(3,:)),'r'); 
    qError1 = qTraj(1,:)-qTable(1,:);
    maxqError1 = rad2deg(max(qError1));
    qError2 = qTraj(2,:)-qTable(2,:);
    maxqError2 = rad2deg(max(qError2));
    qError3 = qTraj(3,:)-qTable(3,:);
    maxqError3 = rad2deg(max(qError3));


    ERMS_joint(1) = rad2deg(rms(qError1));
    E_max_joint(1) = rad2deg(max(abs(qError1)));

    ERMS_joint(2) = rad2deg(rms(qError2));
    E_max_joint(2) = rad2deg(max(abs(qError2)));

    ERMS_joint(3) = rad2deg(rms(qError3));
    E_max_joint(3) = rad2deg(max(abs(qError3)));


    ERMS = [ERMS_joint; E_max_joint];
    save(['ERMS_P' datestr(now,'dd-mmm-yyyy HH-MM-SS') '.mat'], 'ERMS');
    qdotTable = [];
    qedotTable = [];
    
%     if nargin == 3
%         SAMPLING_TIME = 0.008;
%         if isempty(qTraj)&&~isempty(qdotTraj)            
%             acc = input('enter desired acceleration: ')
%             if(isempty(acc))
%                 acc = 3;
%             end
%             if size(qdotTraj,1)~=6          % assuming N steps >> DOF
%                 qdotTraj = qdotTraj';       % trajectory must be column-wise
%             end
%             N = size(qdotTraj,2);           % get number of steps
% 
%             % Define variables 
%             qHome = [-0.1921 -1.8577 2.0274 -0.1697 1.3787 3.1416];
%             qdotTable = zeros(6,N);
%             qTable = zeros(6,N);
%             qedotTable = zeros(6,N);
% 
%             arm.update();          
%             for i=1:N 
%                 tic                
%                 % update and get robot state
%                 arm.update();
%                 qdot = arm.getJointsSpeeds();  
%                 q = arm.getJointsPositions();
%                 err = qdotTraj(:,i) - qdot;
%                 
%                 % Fill in record variables
%                 qdotTable(:,i) = qdot;
%                 qTable(:,i) = q;
%                 qedotTable(:,i) = err;
% 
%                 % set robot speed
%                 arm.setJointsSpeed(qdotTraj(:,i), acc, 3*SAMPLING_TIME);   
%                 
%                 % for the sake of real time 
%                 while(toc<SAMPLING_TIME)
%                 end
%             end
%         end
%     end
    
        