%TRACKREFERENCE1  execute trajectory with proportional controller
% 
% REFERENCETRACK1(
% 
% Yudha Prawira Pane (c)
% Created on Jan-14-2015
function [] = trackReference1(arm, qTraj, qdotTraj)
    SAMPLING_TIME = 0.01;
    if nargin < 2
        error 'Not enough input argument(s)'
    end
    
    if nargin == 2 % track reference given positions only 
        acc = input('enter desired acceleration: ')
        if(isempty(acc))
            acc = 3;
        end
        acc = 1;
        if size(qTraj,1)~=6     % assuming N steps >> DOF
            qTraj = qTraj';     % trajectory must be column-wise
        end
        N = size(qTraj,2);      % get number of steps
        axis([0 N -3.14 3.14]);
        
        % Define variables 
        qHome = [-0.1921 -1.8577 2.0274 -0.1697 1.3787 3.1416];
        err = zeros(6,1);
        qTable = zeros(6,N);

        Kp = 3;
                
        arm.moveJoints(qHome,2,3); % move the robot to home position first
        pause(5);
        
        for i=1:N 
            tic
            arm.update();
            q = arm.getJointsPositions();    
            qTable(:,i) = q;
            err = qTraj(:,i) - q;
            qdotRef = Kp*err;
            arm.setJointsSpeed(qdotRef, acc,SAMPLING_TIME);            
            while(toc<SAMPLING_TIME)
            end
        end
        time = 1:1:N;
        clf;
        subplot(3,1,1);
        plot(time, qTable(1,:)); hold; plot(time, qTraj(1,:),'r');
        subplot(3,1,2);
        plot(time, qTable(2,:)); hold; plot(time, qTraj(2,:),'r');
        subplot(3,1,3);
        plot(time, qTable(3,:)); hold; plot(time, qTraj(3,:),'r');        
    end
    
        