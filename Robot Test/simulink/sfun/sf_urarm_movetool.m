function sf_urarm_movetool(block)
% sf_urarm_movetool, a level-2 s-function for moving the tool of the ur arm
%
% Fankai Zhang 2013-10-16


setup(block);

function setup(block)

% Register the number of ports.
block.NumInputPorts  = 5;
block.NumOutputPorts = 0;

% Set up the port properties to be inherited or dynamic.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% pos: [x,y,z,roll,pitch,yaw]
block.InputPort(1).DatatypeID  = 0; % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).SamplingMode  = 'Sample';
block.InputPort(1).Dimensions = 6;

% acceleration, speed, time, blend radius
for i= 2:5
    block.InputPort(i).DatatypeID  = 0; % double
    block.InputPort(i).Complexity  = 'Real';
    block.InputPort(i).SamplingMode  = 'Sample';
    block.InputPort(i).Dimensions = 1;
end


% Register the parameters.
block.NumDialogPrms     = 2;

block.SampleTimes = [block.DialogPrm(2).Data, 0];

% -----------------------------------------------------------------
% Options
% -----------------------------------------------------------------
% Specify if Accelerator should use TLC or call back to the
% M-file
block.SetAccelRunOnTLC(false);



% -----------------------------------------------------------------
% Register methods called at run-time
% -----------------------------------------------------------------

block.RegBlockMethod('ProcessParameters', @ProcessPrms);
block.RegBlockMethod('Outputs', @Outputs);
% block.RegBlockMethod('Terminate', @Terminate);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

function ProcessPrms(block)

block.AutoUpdateRuntimePrms;

function InitializeConditions(block)
block.Dwork(1).Data = 0;
tic

function DoPostPropSetup(block)
block.NumDworks = 1;

block.Dwork(1).Name            = 'SampleCounter';
block.Dwork(1).Dimensions      = 1;
block.Dwork(1).DatatypeID      = 0;      % double
block.Dwork(1).Complexity      = 'Real'; % real

function Outputs(block)

urarm = block.DialogPrm(1).Data;
Ts = block.DialogPrm(2).Data;

pos = block.InputPort(1).Data;
acc = block.InputPort(2).Data;
speed = block.InputPort(3).Data;
time = block.InputPort(4).Data;
blend_radius = block.InputPort(5).Data;

urarm.moveTool(pos,speed,acc,time,blend_radius);

n = block.Dwork(1).Data + 1;
while toc < n*Ts;end;
block.Dwork(1).Data = n;
toc;



