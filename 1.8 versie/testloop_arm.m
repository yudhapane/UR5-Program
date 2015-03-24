% testloop_arm
% This is the file you want to run when first starting on the robot
% Fankai Zhang TU Delft DCSC Robotics lab (c)
% October 28, 2013

%%
% type:
% help URArm
%% initialize
startup
arm = URArm();
%% open
ip = '192.168.1.50';
arm.fopen(ip)
%%
Tf = 10;
Ts = 1/125;
N = floor(Tf/Ts);

t = tic;

joints_pos = zeros(N,6);
joints_speed = zeros(N,6);
tool_pos = zeros(N,6);
tool_pos2 = zeros(N,6); 
tool_speed = zeros(N,6);
tool_accelerometer = zeros(N,3);
tool_forces = zeros(N,6);

disp('start loop');

for i = 1:N
    
    % read data
    arm.update();
    joints_pos(i,:) = arm.getJointsPositions();
    joints_speed(i,:) = arm.getJointsSpeeds();
    tool_pos(i,:) = arm.getToolPositions();
    tool_pos2(i,:) = arm.getToolPositions2(); % should get the same results the ros version 
    tool_speed(i,:) = arm.getToolSpeeds();
    tool_accelerometer(i,:) = arm.getToolAccelerometer();
    tool_forces(i,:) = arm.getToolForces();
    
    % realtime synchronisation
    while toc(t) < Ts;
    end
    t = tic;
end

disp('done')

%% close
% arm.fclose();