function sf_urarm_settool(block)
% sf_urarm_settool, a level-2 s-function for setting the speed of the ur arm
% tool.
%
% Fankai Zhang 2013-10-17


setup(block);

function setup(block)

% Register the number of ports.
block.NumInputPorts  = 3;
block.NumOutputPorts = 0;

% Set up the port properties to be inherited or dynamic.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% speed [x,y,z,roll,pitch,yaw]
block.InputPort(1).DatatypeID  = 0; % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).SamplingMode  = 'Sample';
block.InputPort(1).Dimensions = 6;

% acceleration
block.InputPort(2).DatatypeID  = 0; % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).SamplingMode  = 'Sample';
block.InputPort(2).Dimensions = 1;

% time
block.InputPort(3).DatatypeID  = 0; % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).SamplingMode  = 'Sample';
block.InputPort(3).Dimensions = 1;


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

speed = block.InputPort(1).Data;
acc = block.InputPort(2).Data;
time = block.InputPort(3).Data;

urarm.setToolSpeed(speed,acc,time);

n = block.Dwork(1).Data + 1;
while toc < n*Ts; end;
block.Dwork(1).Data = n;
toc;





