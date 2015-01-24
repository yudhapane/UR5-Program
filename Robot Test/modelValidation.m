%MODELVALIDATION validates ur5 model from Marco de Gier work
function [qTable, qdotTable, qedotTable] = modelValidation(arm)
    close all;
    
    jointModel{1} = load('iden_joint1_scanner_0.05_sys.mat');
	jointModel{2} = load('iden_joint2_scanner_0.05_sys.mat');
	jointModel{3} = load('iden_joint3_scanner_0.05_sys.mat');
	jointModel{4} = load('iden_joint4_scanner_0.05_sys.mat');
	jointModel{5} = load('iden_joint5_scanner_0.05_sys.mat');
	jointModel{6} = load('iden_joint6_scanner_0.05_sys.mat');
	
    %% Define input signals
    ui = [zeros(1,30) 0.2*ones(1,250) -0.15*ones(1,350)];
    N = size(ui,2);
    u{1} = [ui; zeros(5,N)];
    u{2} = [zeros(1,N); ui; zeros(4,N)];
    u{3} = [zeros(2,N); ui; zeros(3,N)];
    u{4} = [zeros(3,N); ui; zeros(2,N)];
    u{5} = [zeros(4,N); ui; zeros(1,N)];
    u{6} = [zeros(5,N); ui];

    
    %% Define each joint home position. This represents a safe position
    % on which we can excite an individual robot's joint
     
    
    %% Joint 1
    nJoint = 6;

    for i = 1:nJoint
        n = size(jointModel{i}.sys.a,1);
        qdotTraj = u{i};
        t = 0:1:N-1;
        t = t';
        tS = 0.008;
        t = t*tS;
        qHome = [-pi/2 -1.8577 2.0274 -0.1697 1.3787 3.1416];
        arm.moveJoints(qHome,2,5); % move the robot to home position first
        [y,t,x] = lsim(jointModel{i}.sys, ui, t, zeros(n,1));    
        y(:,1) = y(:,1) + qHome(i);
        [qTable, qdotTable, qedotTable] = trackReference1(arm, [], qdotTraj);

        pause(2); % wait until the robot is finished

        qSim = y';

        % save recorded data in structure format
        modelValData.name = ['model validation data' int2str(i)];
        modelValData.qSim = qSim;  
        modelValData.qTable    = qTable;             
        modelValData.qdotTable = qdotTable;
        modelValData.qedotTable = qedotTable;
        modelValData.u = ui;
		modelValData.vaf1 = vaf(qTable(i,:), qSim(1,:));
        modelValData.vaf2 = vaf(qdotTable(i,:), qSim(2,:));
        
		modelValDATA{i} = modelValData;

        figure; 
        subplot(2,1,1);
        time = (1:1:N)*tS;
        plot(time, qTable(i,:)); hold on; plot(time, qSim(1,:),'r');         
        xlabel('Time [s]'); ylabel('Angle [rad]');
        legend('measurement', 'simulation');
        
        subplot(2,1,2);
        plot(time, qdotTable(i,:)); hold on; plot(time, y(:,2),'r'); 
        plot(time, ui, 'y'); xlabel('Time [s]'); ylabel('Angle vel [rad/s]');
        legend('measurement', 'simulation', 'input');
        savefigfolder = 'D:\Dropbox\TU Delft - MSc System & Control\Graduation Project (Thesis)\UR5 Robot\UR5 Programs\Robot Test\figures\';
        savefig([savefigfolder 'Model Validation Joint-' int2str(i) '.fig']);
    end
    
    savefolder = 'D:\Dropbox\TU Delft - MSc System & Control\Graduation Project (Thesis)\UR5 Robot\UR5 Programs\Robot Test\recorded data\';
    save([savefolder 'modelValDATA' datestr(now,'dd-mmm-yyyy HH-MM-SS') '.mat'], 'modelValDATA');
