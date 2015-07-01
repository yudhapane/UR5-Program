loadParamsUR5_4;

qTable  = zeros(6,1);
qdTable = zeros(6,1);
wTable  = zeros(6,1);
wdTable = zeros(6,1);

EXPERIMENT_TIME = 3;
SAMPLING_TIME   = par.ts;
N               = EXPERIMENT_TIME/SAMPLING_TIME; % number of samples 
time            = 0:SAMPLING_TIME:EXPERIMENT_TIME; 
time(end)       = [];

arm.update();
q0      = arm.getJointsPositions();
qref    = q0 + [deg2rad(50); zeros(5,1)];

%% Case 1: PID controlled
arm.update();
for i = 1: N
    arm.update();
    qTable(:,i) = arm.getJointsPositions();
    err         = 0;
end
    clear qTable;
    arm.moveJoints(q0,1,5,3);
    dur = tic;
    while toc(dur)<3
        timer = tic;
        while toc(timer)<par.ts
        end
        arm.update();
        qTable(:,i) = arm.getJointsPositions();
        qdTable(:,i) = arm.getJointsSpeeds();
        i = i+1;
    end