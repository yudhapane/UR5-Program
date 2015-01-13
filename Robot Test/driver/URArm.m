function obj = URArm()
% URArm provides methods to read data from and write data to 
% the UR robotic arm via tcp/ip.
%
%
% arm = URArm;
%
% Initialization:
%   ipaddress = '192.168.0.100'; % example
%   arm.fopen(ipaddress);
%
% Read:
%   [joints_positions_,joints_speeds_,joints_currents_,...
%             joints_temperatures_,tool_positions_,tool_speeds_,...
%             tool_forces_,tool_accelerometer_,digital_input_] =
%             arm.fread();
% or
%   arm.update();
%   pos = arm.getJointsPositions();
%   speed = arm.getJointsSpeeds();
%   toolpos = arm.getToolPositions(); % [x,y,z,rx,ry,rz] last 3 is
%   axis-angle representation NOT euler angles!
%   toolpos = arm.getToolPositions2(): [x,y,z,roll,yaw,pitch], derive tool 
%   position from the joints angles
%   toolspeed = arm.getToolSpeeds();
%   temp = arm.getMotorTemperatures();
%   currents = arm.getJointsCurrents();
%   forces = arm.getToolForces();
%   accel = arm.getToolAccelerometer();
%   digi_input = arm.getDigitalInput();
%
% Write: [always in radians]
%   arm.moveJoints(input_angles,speed,acceleration)
%   arm.moveTool(input_pose,speed,acceleration) % [x,y,z,rx,ry,rz] NOT
%   euler angles
%   arm.setJointsSpeed(input_speeds,acceleration,time)
%   arm.setToolSpeed(input_speeds,acceleration,time)
%
% Waypoints:
%   arm.appendWaypoint(angles)
%   arm.playWaypoints()
%   arm.clearWaypoints()
%   arm.getWaypoints()
%   arm.setWaypoints(angles_matrix) % each row is a waypoint
%
% Misc:
%   arm.powerdown()
%   arm.isConnected()
%   arm.stopJoints(acceleration)
%   arm.stopTool(acceleration)
%   arm.log(text)
%   arm.home()
%   arm.setDigitalOutput(port,stat)
%   arm.setAnalogOutput(port,stat)
%   arm.openGripper()
%   arm.closeGripper()
%   arm.sendCustomCommand(custom_cmd_string)
%
% Close:
% arm.fclose();
%
% Example:
%	ipaddress = '192.168.0.50';
%	arm = URArm();	
%   arm.fopen(ipaddress);
%	joints = arm.getJoints();
%   arm.moveJoints(pos,speed,acceleration);
%
% Fankai Zhang TU Delft DCSC Robotics lab (c)
% June 27, 2013

try
    cmd_ = URArmMex;
catch err
    rethrow(err);
end
obj.fopen = @open;
obj.fclose = @close;
obj.fread = @getData;
obj.update = @update;
obj.getJointsPositions = @getJointsPositions;
obj.getJointsSpeeds = @getJointsSpeeds;
obj.getToolPositions = @getToolPositions;
obj.getToolPositions2 = @getToolPositions2;
obj.getToolSpeeds = @getToolSpeeds;
obj.getJointsCurrents = @getJointsCurrents;
obj.getMotorTemperatures = @getMotorTemperatures;
obj.moveJoints = @moveJoints;
obj.moveJointsSelective = @moveJointsSelective;
obj.moveJointsIncremental = @moveJointsIncremental;
obj.moveTool = @moveTool;
obj.setJointsSpeed = @setJointsSpeed;
obj.setToolSpeed = @setToolSpeed;
obj.setToolTaskFrameVelocity = @setToolTaskFrameVelocity;
obj.isConnected = @isConnected;
obj.powerdown = @powerdown;
obj.stopJoints = @stopJoints;
obj.stopTool = @stopTool;
obj.log = @log;
obj.home = @home;
obj.sendCustomCommand = @sendCustomCommand;
obj.getToolForces = @getToolForces;
obj.getToolAccelerometer = @getToolAccelerometer;
obj.getDigitalInput = @getDigitalInput;
obj.setDigitalOut = @setDigitalOut;
obj.setAnalogOut = @setAnalogOut;
obj.solveForwardKinematics = @solveForwardKinematics;
obj.solveInverseKinematics = @solveInverseKinematics;
obj.closeGripper = @closeGripper;
obj.openGripper = @openGripper;
obj.showInfo = @showInfo;

obj.appendWaypoint = @appendWaypoint;
obj.clearWaypoints = @clearWaypoints;
obj.getNextWaypoint = @getNextWaypoint;
obj.playWaypoints = @playWaypoints;
obj.getWaypoints = @getWaypoints;
obj.setWaypoints = @setWaypoints;

obj.enableLog = @enableLog;

joints_positions_ = zeros(6,1);
joints_speeds_ = zeros(6,1);
joints_currents_ = zeros(6,1);
joints_temperatures_ = zeros(6,1);
tool_positions_ = zeros(6,1);
tool_speeds_ = zeros(6,1);
tool_forces_ = zeros(6,1);
tool_accelerometer_ = zeros(3,1);
digital_input_ = 0;
waypoints_ = [];
waypoint_itr_ = 0;

    function open(ip)
        fprintf(1,'Establishing connection to URArm @ %s, please wait ...\n',...
            ip);
        URArmMex(cmd_.OPEN,ip);
        if ~isConnected()
            error('URArm:Connect',...
                'Unable to connect to URArm!\nPlease check cable or ip!');
        else
            pause(1); % wait a bit to receive some data in the background
            fprintf(1,'URArm connection established!\n');
        end
    end

    function close
        URArmMex(cmd_.CLOSE);
        fprintf(1,'URArm connection closed!\n');
    end

    function varargout = getData
        [joints_positions_,joints_speeds_,joints_currents_,...
            joints_temperatures_,tool_positions_,tool_speeds_,...
            tool_forces_,tool_accelerometer_,digital_input_]...
            = URArmMex(cmd_.GET_ALL_DATA);
        
        if nargin > 0
            varargout{1} = joints_positions_;
        end
        
        if nargin > 1
            varargout{2} = joints_speeds_;
        end
        
        if nargin > 2
            varargout{3} = joints_currents_;
        end
        
        if nargin > 3
            varargout{4} = joints_temperatures_;
        end
        
        if nargin > 4
            varargout{5} = tool_positions_;
        end
        
        if nargin > 5
            varargout{6} = tool_speeds_;
        end
        
        if nargin > 6
            varargout{7} = tool_forces_;
        end
        
        if nargin > 7
            varargout{8} = tool_accelerometer_;
        end
        
        if nargin > 8
            varargout{9} = digital_input_;
        end
    end

    function update
        getData;
    end
    function out = getJointsPositions
        % returns joints angles in radians for each joint
        out = joints_positions_;
    end

    function out = getJointsSpeeds
        % returns speed in rad/s for each joint
        out = joints_speeds_;
    end

    function out = getToolPositions        
        % Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), 
        % where rx, ry and rz is a rotation vector representation 
        % of the tool orientation
        out = tool_positions_;
    end

    function out = getToolPositions2
        out = URArmMex(cmd_.GET_TOOL_POSITIONS2);
    end

    function out = getToolSpeeds
        out = tool_speeds_;
    end
    function out = getToolForces
        out = tool_forces_;
    end
    function out = getToolAccelerometer
        % Tool x,y and z accelerometer values
        out = tool_accelerometer_;
    end
    function out = getDigitalInput
        out = digital_input_;
    end

    function out = getJointsCurrents
        out = joints_currents_;
    end

    function out = getMotorTemperatures
        out = joints_temperatures_;
    end

    function moveJoints(pos,speed,accel,time,blend_radius)
        if isempty(pos)
            return;
        end
        if nargin < 5 || isempty(blend_radius)
            blend_radius = 0;
        end
        if nargin < 4 || isempty(time)
            time = 0;
        end
        if nargin < 3 || isempty(accel)
            accel = 1.2;
        end
        if nargin < 2 || isempty(speed)
            speed = 0.3;
        end        
        if accel == 0 || speed == 0
            return;
        end
        URArmMex(cmd_.MOVE_JOINTS,pos,accel,speed,time,blend_radius);
    end

    function moveJointsSelective(pos,selection,speed,accel)
        if nargin < 4 || isempty(accel)
            accel = 1.2;
        end
        if nargin < 3 || isempty(speed)
            speed = 0.3;
        end
        joints = getJointsPositions();
        joints(selection) = pos;
        moveJoints(joints,speed,accel);
    end

    function moveJointsIncremental(val,selection,speed,accel)
        if nargin < 4 || isempty(accel)
            accel = 1.2;
        end
        if nargin < 3 || isempty(speed)
            speed = 0.3;
        end
        if isempty(val)
            val = 0.01;
        end
        joints = getJointsPositions();
        joints(selection) = joints(selection)+val;
        moveJoints(joints,speed,accel);
    end

    function moveTool(pos,speed,accel,time,blend_radius)
        if isempty(pos)
            return;
        end
        if nargin < 5 || isempty(blend_radius)
            blend_radius = 0;
        end
        if nargin < 4 || isempty(time)
            time = 0;
        end
        if nargin < 3 || isempty(accel)
            accel = 1.2;
        end
        if nargin < 2 || isempty(speed)
            speed = 0.3;
        end
        if accel == 0 || speed == 0
            return;
        end
        URArmMex(cmd_.MOVE_TOOL,pos,accel,speed,time,blend_radius);
    end

    function setJointsSpeed(speed, accel, time)
        URArmMex(cmd_.SET_JOINTS_SPEED,speed,accel,time);
    end

    function setToolSpeed(speed,accel,time)
        URArmMex(cmd_.SET_TOOL_SPEED,speed,accel,time);
    end

    function setToolTaskFrameVelocity(linpos,angpos,accel,time)
        URArmMex(cmd_.SET_TOOLTASKFRAME,linpos,angpos,accel,time);
    end

    function out = isConnected()
        out = URArmMex(cmd_.IS_CONNECTED);
    end

    function powerdown()
        URArmMex(cmd_.POWERDOWN);
    end

    function stopJoints(accel)
        if nargin < 1 || isempty(accel)
            accel = 1.2;
        end
        URArmMex(cmd_.STOP_JOINTS,accel);
    end

    function stopTool(accel)
        if nargin < 1 || isempty(accel)
            accel = 1.2;
        end
        URArmMex(cmd_.STOP_TOOL,accel);
    end

    function log(text)
        URArmMex(cmd_.LOG,text);
    end

    function sendCustomCommand(cmd)
        URArmMex(cmd_.SEND_CUSTOM_COMMAND,cmd);
    end

    function home(pos,speed,accel)
        if nargin < 1 || isempty(pos)
            pos = [0,-pi/2,0,-pi/2,0,0];
        end
        if nargin < 3 || isempty(accel)
            accel = 1.2;
        end
        if nargin < 2 || isempty(speed)
            speed = 0.3;
        end
        URArmMex(cmd_.HOME,pos,speed,accel);
    end

    function setDigitalOut(id,v)
        URArmMex(cmd_.SET_DIGITAL_OUT,id,v);
    end
    function setAnalogOut(id,v)
        URArmMex(cmd_.SET_ANALOG_OUT,id,v);
    end
    function end_effector_pos = solveForwardKinematics(angles)
        end_effector_pos = URArmMex(cmd_.SOLVE_FORWARD_KINEMATICS,angles);
    end
    function out = solveInverseKinematics(end_effector_pos)
        out = URArmMex(cmd_.SOLVE_INVERSE_KINEMATICS,end_effector_pos);
    end
    function closeGripper
        setDigitalOut(0,0);
    end
    function openGripper
        setDigitalOut(0,1);
    end
    function showInfo
        update();
        disp('joints positions:');
        disp(mat2str(getJointsPositions()'));
        disp('joints speeds:');
        disp(mat2str(getJointsSpeeds()'));
        disp('joints currents:');
        disp(mat2str(getJointsCurrents()'));        
        disp('joints temperatures:');
        disp(mat2str(getMotorTemperatures()'));
        disp(' ');
        
        disp('tool positions');
        disp(mat2str(getToolPositions()'));
        disp('tool speeds');
        disp(mat2str(getToolSpeeds()'));
        disp('tool forces');
        disp(mat2str(getToolForces()'));
        disp('tool accelerometer');
        disp(mat2str(getToolAccelerometer()'));
        disp(' ');
        
        disp('digital input');
        disp(getDigitalInput());
        disp(' ');
    end

    function appendWaypoint(angles)
        if nargin < 1
            update();
            angles = getJointsPositions();
        end
        waypoints_ = [waypoints_;angles(:)'];
    end
    function clearWaypoints()
        waypoints_ = [];
        waypoint_itr_ = 0;
    end
    function wp = getWaypoints()
        wp = waypoints_;
    end
    function setWaypoints(wp)
        waypoints_ = wp;
    end
    function wp = getNextWaypoint()
        if ~isempty(waypoints_)
            wp = waypoints_(mod(waypoint_itr_,size(waypoints_,1))+1,:);
            waypoint_itr_ = waypoint_itr_ + 1;
        else
            wp = [];
        end
    end
    function out = checkGoalReached(pos,dest)
        out = all(abs(pos(:)-dest(:)) <= 0.001);
    end
    function playWaypoints(speed,accel)
        if nargin < 2 || isempty(accel)
            accel = 1.2;
        end
        if nargin < 1 || isempty(speed)
            speed = 0.3;
        end
        if isempty(waypoints_)
            return;
        end
        t_ = tic;
        Ts = 1/100;
        
        cleanupObj = onCleanup(@() cleanupFun);
        
        while true
            wp = getNextWaypoint();
            moveJoints(wp,speed,accel);
            while true
                update();
                wp_ = getJointsPositions();
                if checkGoalReached(wp,wp_)
                    break;
                end
                
                while toc(t_) < Ts;
                end
                t_ = tic;
            end
        end
    end

    function cleanupFun
        stopJoints();
    end

    function enableLog
        URArmMex(cmd_.ENABLE_LOG,end_effector_pos);
    end
end
