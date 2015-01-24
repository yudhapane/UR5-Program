%GENERATETRAJ1 generate various trajectories
%
% the trajectories are set as workspace variables
% 
% Yudha Prawira Pane (c)
% Created on Jan-14-2015

qEnd1 = deg2rad([-5.16 -62.44 67.79 -5.21 84.83 180.01]); % this values were obtained from polyscope 
qTraj1 = jtraj(qHome, qEnd1, 200); % generate trajectory with 100 steps

load 'Line.mat'
N = size(r{1},2);
qTraj2 = zeros(6,N);
for i = 1:6
    qTraj2(i,:) = r{i}(1,:);
end

format long;
ERMS_PID = load('ERMS_PID.mat');
ERMS_pid = ERMS_PID.ERMS
ERMS_MPC = load('ERMS_MPC.mat');
ERMSf_mpc = ERMS_MPC.ERMS

tic
arm.setJointsSpeed([0.1 0 0 0 0 0], 1, 1);
pause(1.05);
toc
arm.setJointsSpeed([-0.1 0 0 0 0 0], 1, 1);
