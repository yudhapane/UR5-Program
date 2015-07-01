%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% Simple GUI for interacting with the EdRo Robot.
%
% History:  01/2009, G.A.D.Lopes@tudelft.nl
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function varargout = RobotDemo(varargin)
% ROBOTDEMO M-file for RobotDemo.fig
%      ROBOTDEMO, by itself, creates a new ROBOTDEMO or raises the existing
%      singleton*.
%
%      H = ROBOTDEMO returns the handle to a new ROBOTDEMO or the handle to
%      the existing singleton*.
%
%      ROBOTDEMO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ROBOTDEMO.M with the given input arguments.
%
%      ROBOTDEMO('Property','Value',...) creates a new ROBOTDEMO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before RobotDemo_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to RobotDemo_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help RobotDemo

% Last Modified by GUIDE v2.5 28-Jan-2009 11:49:39

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @RobotDemo_OpeningFcn, ...
                   'gui_OutputFcn',  @RobotDemo_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before RobotDemo is made visible.
function RobotDemo_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to RobotDemo (see VARARGIN)

% Choose default command line output for RobotDemo
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes RobotDemo wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = RobotDemo_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in DemonstrateButton.
function DemonstrateButton_Callback(hObject, eventdata, handles)
% hObject    handle to DemonstrateButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    set(handles.ImitateButton,'Enable','off');
    set(handles.ImitateDemoButton,'Enable','off');

% Control the Ed-Ro robot to a desired pose (0 by default)
%    RTROBOT_P(Port,ref,Kp)
%
%       Port ... port, e.g., 'COM1' (default Port = 'COM1')
%       ref  ... reference to initialize to (default ref = 0)
%       Kp   ... proportional gain (default Kp = 3)

% (c) Robert Babuska, 2005
%close all;
joint = [1 2 3];                                % index of joint(s) to be controlled
% joint 1 = base angle
% joint 2 = aluminum arm angle
% joint 3 = plastic arm angle
                                            % (1 = base, 2 = shoulder, etc.)
                                            % can be a vector too
n = length(joint);                          % number of joints to be controlled
joint = joint(:)+(0:6:6*(n-1))';            % convert into sequential index
joint_select = zeros(6,n);                  % prepare zeros
joint_select(joint) = 1;                    % assing 1 at the right places

Port = 'COM3';              % default port
ref = 0;                    % default reference
Kp = 3;                     % default control gain

h = 0.05;                                       % sampling period
eps = 0.05;                                     % tolerance around ref
umax = 1;                                     % maximum allowed control 

xx = zeros(2,7);                                % aux var for BE conversion

% initialize, open the port, define timer
hwinit;                                         % read HW parameters
daoutgain = daoutgain(1:6);                     % ignore laser
daoutoffs = daoutoffs(1:6);                     % ignore laser
com = rs232('GetParams','default');             % get default RS232 parameters
com.Port = Port;                                % define port number
com.BaudRate = 115200;                          % define Baud rate
com.ReadTimeout = 1;                            % define short timeout
com.WriteTimeout = 1;                           % define short timeout
[params,result]=rs232('open',com);              % open the port

%% Display
axes(handles.axes2);
newplot;
[Z,H]=CreateRobot();
view(142.5,30);

axis([-30 30 -30 30 -5 50]);
axis manual

axes(handles.axes3);
newplot;
title('Demonstration');
axis([0 200 -1 1]);
axis manual
drawnow

%% control loop
for i = 1 : 200,                                 % run for at most 40*h seconds
    t = clock;
    rs232('write',com,uint8(128));              % control byte for read D/A
    xx(:) = rs232('read',com,14);               % read 14 bytes from 7 sensors
    x = double(xx'*[256 1]');                   % convert to big endian
    x = adingain(:).*(x + adinoffs(:));         % scale to physical range
    x = joint_select'*x(1:6);                   % select the right variable   
    xlog(i,:) = x';
    
    % Display
%    if (get(handles.checkbox1,'Value')>0)
        DrawRobot(Z,H,x(1),x(2),x(3),0);
        plot(xlog);
        axis([0 200 -1 1]);
        drawnow;
 %   end
    
    while all(clock-t < h-0.1*h), end;          % wait for the next sample
end;
%toc

% reset actuators, close the port
rs232('write',com,uint8(56));                   % laser off
rs232('write',com,uint8(0));                    % control byte for send all actuators
rs232('write',com,uint8(127*ones(6,1)));        % reset all 6 actuators to 0 (127 bin)
rs232('close',com);                             % close serial port

assignin('base','x',xlog);
x=xlog;
save measurements x
MakeRef;


set(handles.ImitateButton,'Enable','on');
set(handles.ImitateDemoButton,'Enable','on');

% --- Executes on button press in ImitateButton.
function ImitateButton_Callback(hObject, eventdata, handles)
% hObject    handle to ImitateButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ImitateButton

if (get(hObject,'Value'))
    set(handles.DemonstrateButton,'Enable','off');
    set(handles.ImitateDemoButton,'Enable','off');
    
% do imitation



global LLR_memory;

joint = [1 2 3 4];                                % index of joint(s) to be controlled
% joint 1 = base angle
% joint 2 = aluminum arm angle
% joint 3 = plastic arm angle
                                                % (1 = base, 2 = shoulder, etc.)
                                                % can be a vector too
n = length(joint);                              % number of joints to be controlled
joint = joint(:)+(0:6:6*(n-1))';                % convert into sequential index
joint_select = zeros(6,n);                      % prepare zeros
joint_select(joint) = 1;                        % assing 1 at the right places

Port = 'COM3';               % default port
ref = 0;                     % default reference
Kp = 3;                      % default control gain

h = 0.05;                                       % sampling period
eps = 0.05;                                     % tolerance around ref
umax = 0.7;                                     % maximum allowed control 

xx = zeros(2,7);                                % aux var for BE conversion

% initialize, open the port, define timer
hwinit;                                         % read HW parameters
daoutgain = daoutgain(1:6);                     % ignore laser
daoutoffs = daoutoffs(1:6);                     % ignore laser
com = rs232('GetParams','default');             % get default RS232 parameters
com.Port = 'COM3';                                % define port number
com.BaudRate = 115200;                          % define Baud rate
com.ReadTimeout = 1;                            % define short timeout
com.WriteTimeout = 1;                           % define short timeout
[params,result]=rs232('open',com);              % open the port


% load memorybackup
load referencemodel;  % you can create a reference model by demonstrate.m
load weights;
load processmemory; % this is the current process memory

%% initialize memory
lxr    = 6;         % size of input of reference model
lyr    = 6;         % size of output of reference model
mpr    = 1;         % pointer to reference memory
flagr  = 0;         % do not learn
kr     = 15;        % number of nearest neighbours used in local regression
% sizeR = 500;
LLR_memory{1,mpr} = [Ref(1:6,:) ; W*Ref(7:end,:) ; zeros(1,length(Ref))];   % load the reference samples in memory
% detailed description:
% LLR_memory=[x1(k) x2(k) x3(k) xd1(k) xd2(k) xd3(k) | W*(x1(k+1) x2(k+1)
% x3(k+1) xd1(k+1) xd2(k+1) xd3(k+1) | param1 ]


%title('Reference Vector Field');
% 3D plot of reference vector fields:
%line([Ref(1,:);Ref(1,:)+Ref(4,:)],[Ref(2,:);Ref(2,:)+Ref(5,:)],[Ref(3,:);Ref(3,:)+Ref(6,:)],'Color','k');
%hold on;
%drawnow





lxp    = 15;        % size of input of reference model
lyp    = 3;         % size of output of reference model
mpp    = 2;         % pointer to reference memory
flagp1 = 0;         % do not learn
flagp2 = 1;         % learn 
kp     = 40;        % number of nearest neighbours used in local regression
sizeP  = 1000; 
LLR_memory{1,mpp} = [randn(lxp,sizeP) ; 0.1*randn(lyp,sizeP) ; zeros(1,sizeP)]; % empty processmemory
LLR_memory{1,mpp} = processmemory;  % load the previous processmemory





%% initialize states and stuff
rs232('write',com,uint8(128));                       % control byte for read D/A
xx(:)   = rs232('read',com,14);                        % read 14 bytes from 7 sensors
x       = double(xx'*[256 1]');                            % convert to big endian
x       = adingain(:).*(x + adinoffs(:));                  % scale to physical range
x       = joint_select'*x(1:6);                            % select the right variable 
input   = [x(1:3) ;zeros(3,1)];
u       = zeros(3,1);

% display state   
axes(handles.axes1);
newplot;
currentstatepoint=plot3([x(1)],[x(2)],[x(3)],'or');
hold on;

% 3D plot of reference vector fields:

referenceplot=line([Ref(1,:);Ref(1,:)+Ref(4,:)],[Ref(2,:);Ref(2,:)+Ref(5,:)],[Ref(3,:);Ref(3,:)+Ref(6,:)],'Color','k');
view(3);
camproj perspective;
pbaspect([1 1 1]);
axis manual;
axis([-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2]);
hold off;

%title('Learned Vector Field');
%LRef=LLR_memory{1,mpp};
%plot3(LRef(1,:),LRef(2,:),LRef(3,:),'.');


axes(handles.axes2);
newplot;
[ZZ,HH]=CreateRobot();
view(142.5,30);
axis([-30 30 -30 30 -5 50]);
axis manual;

LRef=LLR_memory{1,mpp};

axes(handles.axes4);
memoryplot=plot(LRef(1,:),LRef(2,:),'.');


i=1;
while (get(hObject,'Value'))
    i=i+1;
    t = clock;
%% Read state
    inputprev = input;
    uprev = u(1:3);
    rs232('write',com,uint8(128));                       % control byte for read D/A
    xx(:) = rs232('read',com,14);                        % read 14 bytes from 7 sensors
    x = double(xx'*[256 1]');                            % convert to big endian
    x = adingain(:).*(x + adinoffs(:));                  % scale to physical range
    x = joint_select'*x(1:6);                            % select the right variable  
    
  %  if (get(handles.checkbox1,'Value')>0)
        % display state   
        set(currentstatepoint,'xdata',x(1),'ydata',x(2),'zdata',x(3));
    
         % display memory
        LRef=LLR_memory{1,mpp};
        set(memoryplot,'xdata',LRef(1,:),'ydata',LRef(2,:));
        
        DrawRobot(ZZ,HH,x(1),x(2),x(3),x(4));

    %    subplot(2,2,3);
    %    plot(LRef(1,:),LRef(3,:),'.')
    %line([LRef(1,:);LRef(1,:)+LRef(4,:)],[LRef(2,:);LRef(2,:)+LRef(5,:)],[LRef(3,:);LRef(3,:)+LRef(6,:)],'Color','b');
    %line([LRef(1,:);LRef(7,:)],[LRef(2,:);LRef(8,:)],[LRef(3,:);LRef(9,:)],'Color','b');
    
    %plot3(LRef(1,:),LRef(2,:),LRef(3,:),'.')
    %view(3);
    %camproj perspective;
    %pbaspect([1 1 1]);
    %drawnow
        drawnow;
   % end
    
    input  =[x(1:3) ; (x(1:3)-inputprev(1:3))];          % deterministic state description
%% Reference model
    [ref model ns NN]  = Locallinearmodel(input,kr,flagr,[],mpr,1);%[prediction model index memoryindex]
    u = Locallinearmodel([W*input;ref],kp,flagp1,[],mpp);    % dead beat control
    u = [u ; .4.*(-2*x(2)-x(3)-x(4))];
    u = min(umax,max(-umax,u));                          % saturate control action
    ud = joint_select*u;                                 % complete control vector
    ud = daoutgain(:).*ud + daoutoffs(:);                % scale to digital range

%% Send u to system
    rs232('write',com,uint8(0));                         % control byte for send all actuators
    rs232('write',com,uint8(ud)');                        % write 6 bytes to 6 actuators

%% Learn inverse process model
    Locallinearmodel([W*inputprev;W*input],kp,flagp2,uprev,mpp,0);
    reference(:,i) = ref;
    state(:,i)     = input;
    actuation(:,i) = u(1:3);
    while all(clock-t < h-0.1*h), end;                   % wait for the next sample


end


processmemory=LLR_memory{1,mpp};
save processmemory processmemory;  % remember processmemory for next imitation







%% reset actuators, close the port

rs232('write',com,uint8(56));                   % laser off
rs232('write',com,uint8(0));                    % control byte for send all actuators
rs232('write',com,uint8(127*ones(6,1)));        % reset all 6 actuators to 0 (127 bin)
rs232('close',com);                             % close serial port



else
    set(handles.DemonstrateButton,'Enable','on');
    set(handles.ImitateDemoButton,'Enable','on');
end


% --- Executes on button press in QuitButton.
function QuitButton_Callback(hObject, eventdata, handles)
% hObject    handle to QuitButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close


% --- Executes on button press in ImitateDemoButton.
function ImitateDemoButton_Callback(hObject, eventdata, handles)
% hObject    handle to ImitateButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of ImitateButton

if (get(hObject,'Value'))
    set(handles.DemonstrateButton,'Enable','off');
    set(handles.ImitateButton,'Enable','off');
    
% do imitation



global LLR_memory;

joint = [1 2 3 4];                                % index of joint(s) to be controlled
% joint 1 = base angle
% joint 2 = aluminum arm angle
% joint 3 = plastic arm angle
                                                % (1 = base, 2 = shoulder, etc.)
                                                % can be a vector too
n = length(joint);                              % number of joints to be controlled
joint = joint(:)+(0:6:6*(n-1))';                % convert into sequential index
joint_select = zeros(6,n);                      % prepare zeros
joint_select(joint) = 1;                        % assing 1 at the right places

Port = 'COM3';               % default port
ref = 0;                     % default reference
Kp = 3;                      % default control gain

h = 0.05;                                       % sampling period
eps = 0.05;                                     % tolerance around ref
umax = 0.7;                                     % maximum allowed control 

xx = zeros(2,7);                                % aux var for BE conversion

% initialize, open the port, define timer
hwinit;                                         % read HW parameters
daoutgain = daoutgain(1:6);                     % ignore laser
daoutoffs = daoutoffs(1:6);                     % ignore laser
com = rs232('GetParams','default');             % get default RS232 parameters
com.Port = 'COM3';                                % define port number
com.BaudRate = 115200;                          % define Baud rate
com.ReadTimeout = 1;                            % define short timeout
com.WriteTimeout = 1;                           % define short timeout
[params,result]=rs232('open',com);              % open the port


% load memorybackup
load referencemodel;  % you can create a reference model by demonstrate.m
load weights;
load processmemory; % this is the current process memory

%% initialize memory
lxr    = 6;         % size of input of reference model
lyr    = 6;         % size of output of reference model
mpr    = 1;         % pointer to reference memory
flagr  = 0;         % do not learn
kr     = 15;        % number of nearest neighbours used in local regression
% sizeR = 500;
LLR_memory{1,mpr} = [Ref(1:6,:) ; W*Ref(7:end,:) ; zeros(1,length(Ref))];   % load the reference samples in memory
% detailed description:
% LLR_memory=[x1(k) x2(k) x3(k) xd1(k) xd2(k) xd3(k) | W*(x1(k+1) x2(k+1)
% x3(k+1) xd1(k+1) xd2(k+1) xd3(k+1) | param1 ]


%title('Reference Vector Field');
% 3D plot of reference vector fields:
%line([Ref(1,:);Ref(1,:)+Ref(4,:)],[Ref(2,:);Ref(2,:)+Ref(5,:)],[Ref(3,:);Ref(3,:)+Ref(6,:)],'Color','k');
%hold on;
%drawnow





lxp    = 12;        % size of input of reference model
lyp    = 3;         % size of output of reference model
mpp    = 2;         % pointer to reference memory
flagp1 = 0;         % do not learn
flagp2 = 1;         % learn 
kp     = 40;        % number of nearest neighbours used in local regression
sizeP  = 500; 
LLR_memory{1,mpp} = [randn(lxp,sizeP) ; 0.1*randn(lyp,sizeP) ; zeros(1,sizeP)]; % empty processmemory
%size(LLR_memory{1,mpp})
%LLR_memory{1,mpp} = processmemory;  % load the previous processmemory
%size(LLR_memory{1,mpp})
lxp    = 15;        % size of input of reference model
lyp    = 3;         % size of output of reference model




%% initialize states and stuff
rs232('write',com,uint8(128));                       % control byte for read D/A
xx(:)   = rs232('read',com,14);                        % read 14 bytes from 7 sensors
x       = double(xx'*[256 1]');                            % convert to big endian
x       = adingain(:).*(x + adinoffs(:));                  % scale to physical range
x       = joint_select'*x(1:6);                            % select the right variable 
input   = [x(1:3) ;zeros(3,1)];
u       = zeros(3,1);

% display state   
axes(handles.axes1);
newplot;
currentstatepoint=plot3([x(1)],[x(2)],[x(3)],'or');
hold on;

% 3D plot of reference vector fields:

referenceplot=line([Ref(1,:);Ref(1,:)+Ref(4,:)],[Ref(2,:);Ref(2,:)+Ref(5,:)],[Ref(3,:);Ref(3,:)+Ref(6,:)],'Color','k');
view(3);
camproj perspective;
pbaspect([1 1 1]);
axis manual;
axis([-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2]);
hold off;

%title('Learned Vector Field');
%LRef=LLR_memory{1,mpp};
%plot3(LRef(1,:),LRef(2,:),LRef(3,:),'.');


axes(handles.axes2);
newplot;
[ZZ,HH]=CreateRobot();
view(142.5,30);
axis([-30 30 -30 30 -5 50]);
axis manual;

LRef=LLR_memory{1,mpp};

axes(handles.axes4);
memoryplot=plot(LRef(1,:),LRef(2,:),'.','Color','red');


tic; % start stopwatch
resetToStableParameters=0; % flag for remembering that parameters have been updated.

i=1;
while (get(hObject,'Value'))
    
    if (toc > 20 & not(resetToStableParameters) )
        copyfile('processmemory-original.mat','processmemory.mat'); 
        copyfile('weights-original.mat','weights.mat'); 

        load processmemory; % this is the current process memory
        
        LLR_memory{1,mpp} = processmemory; 
        
        axes(handles.axes4);
        memoryplot=plot(processmemory(1,:),processmemory(2,:),'.');
        
        resetToStableParameters=1;
    end
    
    
    i=i+1;
    t = clock;
%% Read state
    inputprev = input;
    uprev = u(1:3);
    rs232('write',com,uint8(128));                       % control byte for read D/A
    xx(:) = rs232('read',com,14);                        % read 14 bytes from 7 sensors
    x = double(xx'*[256 1]');                            % convert to big endian
    x = adingain(:).*(x + adinoffs(:));                  % scale to physical range
    x = joint_select'*x(1:6);                            % select the right variable  
    
  %  if (get(handles.checkbox1,'Value')>0)
        % display state   
        set(currentstatepoint,'xdata',x(1),'ydata',x(2),'zdata',x(3));
    
         % display memory
        LRef=LLR_memory{1,mpp};
        set(memoryplot,'xdata',LRef(1,:),'ydata',LRef(2,:));
        
        DrawRobot(ZZ,HH,x(1),x(2),x(3),x(4));

    %    subplot(2,2,3);
    %    plot(LRef(1,:),LRef(3,:),'.')
    %line([LRef(1,:);LRef(1,:)+LRef(4,:)],[LRef(2,:);LRef(2,:)+LRef(5,:)],[LRef(3,:);LRef(3,:)+LRef(6,:)],'Color','b');
    %line([LRef(1,:);LRef(7,:)],[LRef(2,:);LRef(8,:)],[LRef(3,:);LRef(9,:)],'Color','b');
    
    %plot3(LRef(1,:),LRef(2,:),LRef(3,:),'.')
    %view(3);
    %camproj perspective;
    %pbaspect([1 1 1]);
    %drawnow
        drawnow;
   % end
    
    input  =[x(1:3) ; (x(1:3)-inputprev(1:3))];          % deterministic state description
%% Reference model
    [ref model ns NN]  = Locallinearmodel(input,kr,flagr,[],mpr,1);%[prediction model index memoryindex]
    u = Locallinearmodel([W*input;ref],kp,flagp1,[],mpp);    % dead beat control
    u = [u ; .4.*(-2*x(2)-x(3)-x(4))];
    u = min(umax,max(-umax,u));                          % saturate control action
    ud = joint_select*u;                                 % complete control vector
    ud = daoutgain(:).*ud + daoutoffs(:);                % scale to digital range
%% Send u to system
    rs232('write',com,uint8(0));                         % control byte for send all actuators
    rs232('write',com,uint8(ud)');                        % write 6 bytes to 6 actuators
%% Learn inverse process model
    Locallinearmodel([W*inputprev;W*input],kp,flagp2,uprev,mpp,0);
    reference(:,i) = ref;
    state(:,i)     = input;
    actuation(:,i) = u(1:3);
    while all(clock-t < h-0.1*h), end;                   % wait for the next sample


end


processmemory=LLR_memory{1,mpp};
save processmemory processmemory;  % remember processmemory for next imitation







%% reset actuators, close the port
rs232('write',com,uint8(56));                   % laser off
rs232('write',com,uint8(0));                    % control byte for send all actuators
rs232('write',com,uint8(127*ones(6,1)));        % reset all 6 actuators to 0 (127 bin)
rs232('close',com);                             % close serial port



else
    set(handles.DemonstrateButton,'Enable','on');
    set(handles.ImitateButton,'Enable','on');
end


% --- Executes on button press in ResetButton.
function ResetButton_Callback(hObject, eventdata, handles)
% hObject    handle to ResetButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

copyfile('processmemory-original.mat','processmemory.mat'); 
copyfile('weights-original.mat','weights.mat'); 

load processmemory; % this is the current process memory
axes(handles.axes4);
memoryplot=plot(processmemory(1,:),processmemory(2,:),'.');




