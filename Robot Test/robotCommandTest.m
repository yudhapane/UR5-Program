% MOVETOOLCOMMANDTEST a script for testing the internal controllers
% of the UR5 Robot in various types of reference inputs:
% 1. tool Position
% 2. tool Velocity
% 3. joint Position
% 4. joint Velocity
% with three speeds 
% The initial and final position is conditioned such that the trajectory
% is similar to MPC controller in Printing_M.m script.
% At the end of the test, the MPC controller will be run
% 
% Yudha Prawira Pane (c)
% Created on Feb-12-2015

%% Constants
toolPosInit =[-0.364507550227020;
              -0.056367974493252;
               0.335944567769995;
              -1.210963379968542;
              -1.210479967461380;
               1.208456498582886];

toolPosEnd = [-0.563902461923550;
              -0.056254529674121;
               0.335944567769995;
              -1.208973318092507;
              -1.208944993233558;
               1.209303677550983];
           
toolPosEnd2 = [-0.603902461923550;
              -0.056254529674121;
               0.335900000000000;
              -1.208973318092507;
              -1.208944993233558;
               1.209303677550983];

jointPosInit=[-0.191446573932524;
              -1.856613239213558;
               2.027629010446364;
              -0.167475708242196;
               1.379159163003900;
               3.141293335730242];  
           
jointPosEnd =[-0.111586153532510;
              -1.363747532210244;
               1.556567932739275;
              -0.191604746667730;
               1.459836881180825;
               3.141293335730242];      
   
ts = 0.008; 


%% Tool Position -- Speed = normal
time = 13.395639; 
ts1 = 4*ts;
N = round(time/ts1);
toolPOS1 = zeros(6,N);
arm.update();
arm.moveTool(toolPosInit, 1,3,2);
pause(2.5);
arm.moveTool(toolPosEnd, 1,1,time);
dur = tic;
for i = 1:N
    tic
    arm.update();
    toolPOS1(:,i) = arm.getToolPositions();
    while(toc<ts1)
    end
end
disp('Total duration:');
toc(dur)
           
%% Tool Position -- Speed = 2x
time = 6.697819500000000; 
ts2 = 2*ts; 
N = round(time/ts2);
toolPOS2 = zeros(6,N);
arm.update();
arm.moveTool(toolPosInit, 1,3,2);
pause(2.5);
arm.moveTool(toolPosEnd, 1,1,time);
dur = tic;
for i = 1:N
    tic
    arm.update();
    toolPOS2(:,i) = arm.getToolPositions();
    while(toc<ts2)
    end
end
disp('Total duration:');
toc(dur)

%% Tool Position -- Speed = 4x
time = 3.348909750000000; 
ts3 = ts; 
N = round(time/ts3);
toolPOS3 = zeros(6,N);
arm.update();
arm.moveTool(toolPosInit, 1,3,2);
pause(2.5);
arm.moveTool(toolPosEnd, 1,1,time);
dur = tic;
for i = 1:N
    tic
    arm.update();
    toolPOS3(:,i) = arm.getToolPositions();
    while(toc<ts3)
    end
end
disp('Total duration:');
toc(dur)


%% Tool Velocity -- Speed = normal
time = 13.395639; 
ts1 = 4*ts;
N = round(time/ts1);
toolPOS4 = zeros(6,N);
arm.update();
arm.moveTool(toolPosInit, 1,3,2);
toolPosOffset = (toolPosEnd - toolPosInit)/N;
toolSpeedRef = toolPosOffset/ts1;
pause(2.5);
dur = tic;
for i = 1:N
    tic
    arm.update();
    toolPOS4(:,i) = arm.getToolPositions();    
    arm.setToolSpeed(toolSpeedRef,10,2*ts1);
    while(toc<ts1)
    end
end
disp('Total duration:');
toc(dur)         

%% Tool Velocity -- Speed = 2x
time = 6.697819500000000; 
ts2 = 2*ts;
N = round(time/ts2);
toolPOS5 = zeros(6,N);
arm.update();
arm.moveTool(toolPosInit, 1,3,2);
toolPosOffset = (toolPosEnd - toolPosInit)/N;
toolSpeedRef = toolPosOffset/ts2;
pause(2.5);
dur = tic;
for i = 1:N
    tic
    arm.update();
    toolPOS5(:,i) = arm.getToolPositions();    
    arm.setToolSpeed(toolSpeedRef,10,ts2);
    while(toc<ts2)
    end
end
disp('Total duration:');
toc(dur)  

%% Tool Velocity -- Speed = 4x
time = 3.348909750000000; 
ts3 = ts;
N = round(time/ts3);
toolPOS6 = zeros(6,N);
arm.update();
arm.moveTool(toolPosInit, 1,3,2);
toolPosOffset = (toolPosEnd - toolPosInit)/N;
toolSpeedRef = toolPosOffset/ts3;
pause(2.5);
dur = tic;
for i = 1:N
    tic
    arm.update();
    toolPOS6(:,i) = arm.getToolPositions();    
    arm.setToolSpeed(toolSpeedRef,10,ts3);
    while(toc<ts3)
    end
end
disp('Total duration:');
toc(dur)  

%% Joint Position -- Speed = normal
time = 13.395639; 
ts1 = 4*ts;
N = round(time/ts1);
toolPOS7 = zeros(6,N);
arm.update();
arm.moveJoints(jointPosInit, 1,3,2);
pause(2.5);
arm.moveJoints(jointPosEnd, 1,20,time);
dur = tic;
for i = 1:N
    tic
    arm.update();
    toolPOS7(:,i) = arm.getToolPositions();
    while(toc<ts1)
    end
end
disp('Total duration:');
toc(dur)
          
%% Joint Position -- Speed = 2x
time = 6.697819500000000; 
ts2 = 2*ts;
N = round(time/ts2);
toolPOS8 = zeros(6,N);
arm.update();
arm.moveJoints(jointPosInit, 1,3,2);
pause(2.5);
arm.moveJoints(jointPosEnd, 1,20,time);
dur = tic;
for i = 1:N
    tic
    arm.update();
    toolPOS8(:,i) = arm.getToolPositions();
    while(toc<ts2)
    end
end
disp('Total duration:');
toc(dur)

%% Joint Position -- Speed = 4x
time = 3.348909750000000; 
ts3 = ts;
N = round(time/ts3);
toolPOS9 = zeros(6,N);
arm.update();
arm.moveJoints(jointPosInit, 1,3,2);
pause(2.5);
arm.moveJoints(jointPosEnd, 1,20,time);
dur = tic;
for i = 1:N
    tic
    arm.update();
    toolPOS9(:,i) = arm.getToolPositions();
    while(toc<ts3)
    end
end
disp('Total duration:');
toc(dur)

%% Joint Velocity -- Speed = normal
time = 13.395639; 
ts1 = 4*ts;
N = round(time/ts1);
toolPosOffset = (toolPosEnd - toolPosInit)/N;
for i = 1:N
    refTRAJ(:,i) = toolPosInit + (i-1)*toolPosOffset; % populate reference trajectory
end
qTRAJ(:,1) = jointPosInit;

% Generate joint space trajectory using inverse jacobian
for i = 1:N-1
    ds = refTRAJ(:,i+1) - refTRAJ(:,i);    

    % Calculate the joint space trajectory using inverse jacobian    
    J = UR5.jacob0(qTRAJ(:,i));
    dq = inv(J)*ds;
    qTRAJ(:,i+1) = qTRAJ(:,i) + dq;
end

[qTable, qdotTable, qedotTable, toolPOS10] = trackReference3('robot',arm, 'qtraj', qTRAJ,'ts',ts1);
  
%% Joint Velocity -- Speed = 2x
time = 6.697819500000000; 
ts2 = 2*ts;
N = round(time/ts2);
toolPosOffset = (toolPosEnd - toolPosInit)/N;
for i = 1:N
    refTRAJ(:,i) = toolPosInit + (i-1)*toolPosOffset; % populate reference trajectory
end
qTRAJ(:,1) = jointPosInit;

% Generate joint space trajectory using inverse jacobian
for i = 1:N-1
    ds = refTRAJ(:,i+1) - refTRAJ(:,i);    

    % Calculate the joint space trajectory using inverse jacobian    
    J = UR5.jacob0(qTRAJ(:,i));
    dq = inv(J)*ds;
    qTRAJ(:,i+1) = qTRAJ(:,i) + dq;
end

[qTable, qdotTable, qedotTable, toolPOS11] = trackReference3('robot',arm, 'qtraj', qTRAJ,'ts',ts2);

          
%% Joint Velocity -- Speed = 4x
time = 3.348909750000000; 
ts3 = ts;
N = round(time/ts3);
toolPosOffset = (toolPosEnd - toolPosInit)/N;
for i = 1:N
    refTRAJ(:,i) = toolPosInit + (i-1)*toolPosOffset; % populate reference trajectory
end
qTRAJ(:,1) = jointPosInit;

% Generate joint space trajectory using inverse jacobian
for i = 1:N-1
    ds = refTRAJ(:,i+1) - refTRAJ(:,i);    

    % Calculate the joint space trajectory using inverse jacobian    
    J = UR5.jacob0(qTRAJ(:,i));
    dq = inv(J)*ds;
    qTRAJ(:,i+1) = qTRAJ(:,i) + dq;
end

[qTable, qdotTable, qedotTable, toolPOS12] = trackReference3('robot',arm, 'qtraj', qTRAJ,'ts',ts3);
          
