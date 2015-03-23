function [qTable, qdotTable, qedotTable, sTable] = trackReference1(arm, qTraj, qdotTraj)
%TRACKREFERENCE1  execute trajectory with set joint speed commands
% 
% trackReference1(ARM, QTRAJ, QDOTTRAJ) executes a joint space trajectory 
% tracking routine with 3 possibilities:
% 1. trackReference1(ARM, QTRAJ) command robot ARM to execute position 
%    trajectory QTRAJ by first calculating the joint speed based on the
%    joint trajectories
% 
% 
% Yudha Prawira Pane (c)
% created on      : Jan-14-2015
% last updated on : Mar-23-2015

    SAMPLING_TIME = 0.008;
    if nargin < 2
        error 'Not enough input argument(s)'
    end
    
    if nargin == 2 % track reference given positions only 
        acc = input('enter desired acceleration: ');
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

%         Kp = 20;
%         arm.moveJoints(qHome,2,3); % move the robot to home position first
        arm.moveJoints(qTraj(:,1),2,3,3); % move the robot to the first position of the qTraj safely
        pause(3);
        tcount = tic;
        

        
%         for i=1:N-1
%             tic
%             arm.update();
%             q = arm.getJointsPositions();
%             err = qTraj(:,i) - q;
%             % PID
%             % qdotRef = Kp*err;
%             % arm.setJointsSpeed(qdotRef, acc, 2*SAMPLING_TIME);
%             % DIRECT SPEED COMMAND
%             qdotRef = (qTraj(:,i+1) - qTraj(:,i))/SAMPLING_TIME;
%             arm.setJointsSpeed(qdotRef,acc,2*SAMPLING_TIME);
%             while(toc<SAMPLING_TIME)
%             end
%             qrec = arm.getJointsPositions();
%             sTable(:,i+1) = arm.getToolPositions();
%             qTable(:,i) = qrec;
%         end
%         arm.update();
%         qTable(:,N) = arm.getJointsPositions();

        % fill in the first index of the trajectory table
        arm.update();
        qTable(:,1) = arm.getJointsPositions();    
        sTable(:,1) = arm.getToolPositions();     

        for i=1:N-1
            tic            
            % PID
%             q = arm.getJointsPositions();             
%             err = qTraj(:,i) - q;            
%             qdotRef = Kp*err;
%             arm.setJointsSpeed(qdotRef, acc, 2*SAMPLING_TIME);       
            
            % DIRECT SPEED COMMAND
            qdotRef = (qTraj(:,i+1) - qTraj(:,i))/SAMPLING_TIME;
            arm.setJointsSpeed(qdotRef,acc,2*SAMPLING_TIME);
            
            while(toc<SAMPLING_TIME)
            end
            arm.update();  
            qTable(:,i+1) = arm.getJointsPositions();    
            sTable(:,i+1) = arm.getToolPositions();
        end
        
        % Information (elapsed time and plots)
        disp('Duration of tracking:');
        toc(tcount)
        time = 1:1:N;
        clf;
        subplot(321);
        plot(time, rad2deg(qTable(1,:))); hold; plot(time, rad2deg(qTraj(1,:)),'r');
        title('reference vs actual joint-1 trajectory');
        subplot(322);
        plot(time, rad2deg(qTable(2,:))); hold; plot(time, rad2deg(qTraj(2,:)),'r');
        title('reference vs actual joint-2 trajectory');
        subplot(323);
        plot(time, rad2deg(qTable(3,:))); hold; plot(time, rad2deg(qTraj(3,:)),'r'); 
        title('reference vs actual joint-3 trajectory');
        subplot(324);
        plot(time, rad2deg(qTable(4,:))); hold; plot(time, rad2deg(qTraj(4,:)),'r');
        title('reference vs actual joint-4 trajectory');
        subplot(325);
        plot(time, rad2deg(qTable(5,:))); hold; plot(time, rad2deg(qTraj(5,:)),'r');
        title('reference vs actual joint-5 trajectory');
        subplot(326);
        plot(time, rad2deg(qTable(6,:))); hold; plot(time, rad2deg(qTraj(6,:)),'r'); 
        title('reference vs actual joint-6 trajectory');
        
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
    end
    
    if nargin == 3
        SAMPLING_TIME = 0.008;
        if isempty(qTraj)&&~isempty(qdotTraj)            
            acc = input('enter desired acceleration: ');
            if(isempty(acc))
                acc = 3;
            end
            if size(qdotTraj,1)~=6          % assuming N steps >> DOF
                qdotTraj = qdotTraj';       % trajectory must be column-wise
            end
            N = size(qdotTraj,2);           % get number of steps

            % Define variables 
            qHome = [-0.1921 -1.8577 2.0274 -0.1697 1.3787 3.1416];
            qdotTable = zeros(6,N);
            qTable = zeros(6,N);
            qedotTable = zeros(6,N);

            arm.update();          
            for i=1:N 
                tic                
                % update and get robot state
                arm.update();
                qdot = arm.getJointsSpeeds();  
                q = arm.getJointsPositions();
                err = qdotTraj(:,i) - qdot;
                
                % Fill in record variables
                qdotTable(:,i) = qdot;
                qTable(:,i) = q;
                qedotTable(:,i) = err;

                % set robot speed
                arm.setJointsSpeed(qdotTraj(:,i), acc, 3*SAMPLING_TIME);   
                
                % for the sake of real time 
                while(toc<SAMPLING_TIME)
                end
            end
        end
    end
    
        