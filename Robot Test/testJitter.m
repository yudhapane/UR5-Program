%% Trajectory and experiments-related variables
EXPERIMENT_TIME = 5;
SAMPLING_TIME   = 1/125;
N               = EXPERIMENT_TIME/SAMPLING_TIME; % number of samples 
time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
time(end)       = [];

wrefTRAJ        = zeros(6,N);   % reference trajectory, will be populated in the iteration
qrefTRAJ        = zeros(6,N);   % joint space reference trajectory

%% Create the trajectory

arm.update();
ts              = 1/125;
initPos         = arm.getToolPositions();
q0              = arm.getJointsPositions();
initOrientation = initPos(4:end);
qTRAJ           = zeros(6,1);
offset              = [-0.20 0 0 0 0 0]';
finalPos            = initPos + offset;
discretizedOffset   = offset/N;

for i = 1:N
    wrefTRAJ(:,i) = initPos + (i-1)*discretizedOffset; % populate reference trajectory
end

qrefTRAJ(:,1) = q0;

% Generate joint space trajectory 
for i = 1:N-1
    ds = wrefTRAJ(:,i+1) - wrefTRAJ(:,i);    

    % Calculate the joint space trajectory using inverse jacobian    
    J = UR5.jacob0(qrefTRAJ(:,i));
    dq = J\ds;
    qrefTRAJ(:,i+1) = qrefTRAJ(:,i) + dq;
end
% time    = 0;
arm.update();
arm.moveTool(wrefTRAJ(:,1),0.5,1,5);
pause(6);
arm.moveTool(wrefTRAJ(:,end),0.5,1,5)
t = tic;
for i = 1 :625
    tic
    arm.update();
    wTRAJ2(:,i) = arm.getToolPositions();
    while toc<ts
    end
end
toc(t)
figure; 
plot(wTRAJ(3,:))