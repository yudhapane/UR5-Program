function sf_urarm_in(block)
% sf_urarm_in, a level-2 s-function for reading data from the ur arm
%
% Fankai Zhang 28-04-2011

setup(block);

function setup(block)

% Register the number of ports.
block.NumInputPorts  = 0;
block.NumOutputPorts = 7;

% Set up the port properties to be inherited or dynamic.
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override the output port properties.
for i = 1:block.NumOutputPorts-1
    block.OutputPort(i).DatatypeID  = 0; % double
    block.OutputPort(i).Complexity  = 'Real';
    block.OutputPort(i).SamplingMode  = 'Sample';
    block.OutputPort(i).Dimensions = 6;
end
block.OutputPort(7).DatatypeID  = 0; % double
block.OutputPort(7).Complexity  = 'Real';
block.OutputPort(7).SamplingMode  = 'Sample';
block.OutputPort(7).Dimensions = 3;

% Register the parameters.
block.NumDialogPrms = 1;

% Register the sample times.
block.SampleTimes = [0 0];

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


function ProcessPrms(block)

block.AutoUpdateRuntimePrms;


function Outputs(block)

urarm = block.DialogPrm(1).Data;

urarm.update();
block.OutputPort(1).Data = urarm.getToolPositions();
block.OutputPort(2).Data = urarm.getToolSpeeds();
block.OutputPort(3).Data = urarm.getJointsCurrents();
block.OutputPort(4).Data = urarm.getJointsPositions();
block.OutputPort(5).Data = urarm.getJointsSpeeds();
block.OutputPort(6).Data = urarm.getToolForces();
block.OutputPort(7).Data = urarm.getToolAccelerometer();


