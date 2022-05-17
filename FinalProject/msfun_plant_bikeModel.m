function msfun_plant_bikeModel(block)
% % %MSFUNTMPL_BASIC A Template for a Level-2 MATLAB S-Function
% % %   The MATLAB S-function is written as a MATLAB function with the
% % %   same name as the S-function. Replace 'msfuntmpl_basic' with the 
% % %   name of your S-function.
% % 
% % %   Copyright 2003-2018 The MathWorks, Inc.

% This function aims to be the plant for the bike model

%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C MEX counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports
block.NumInputPorts  = 1; % System Inputs
block.NumOutputPorts = 1; % System Outputs

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
block.InputPort(1).Dimensions  = 2; %p = 2 (u = [delta, u]')
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;

% Override output port properties
block.OutputPort(1).Dimensions  = 5; %q = 5 (y = eye(5) x)
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';



block.NumContStates = 5;


% Register parameters
block.NumDialogPrms     = 4;
% params: x0, c, wb, m

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [0 0]; % continous...

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
% block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C MEX counterpart: mdlSetWorkWidths
%%
function DoPostPropSetup(block)
    
    block.NumDworks = 1;
    
    block.Dwork(1).Name = 'U';
    block.Dwork(1).Dimensions   = 2;
    block.Dwork(1).DatatypeID      = 0;
    block.Dwork(1).Complexity      = 'Real';
    block.Dwork(1).UsedAsDiscState = true;
    
% end DoPostPropSetup
% 
% % %%
% % %% InitializeConditions:
% % %%   Functionality    : Called at the start of simulation and if it is 
% % %%                      present in an enabled subsystem configured to reset 
% % %%                      states, it will be called when the enabled subsystem
% % %%                      restarts execution to reset the states.
% % %%   Required         : No
% % %%   C MEX counterpart: mdlInitializeConditions
% % %%
% % function InitializeConditions(block)
% % 
% %     
% % %end InitializeConditions


%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C MEX counterpart: mdlStart
%%
function Start(block)

    % Initial Conditions (for CT)
    x0 = block.DialogPrm(1).Data;
    block.ContStates.Data = x0;
    
    % Initial Conditions (for DT)
    block.Dwork(1).Data = [0,10];

%end Start

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C MEX counterpart: mdlOutputs
%%
function Outputs(block)

  block.OutputPort(1).Data = block.ContStates.Data;

%end Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C MEX counterpart: mdlUpdate
%%
function Update(block)

    block.Dwork(1).Data = block.InputPort(1).Data;

%end Update

%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C MEX counterpart: mdlDerivatives
%%
function Derivatives(block)
    
    % Dynamics Parms:
    a = block.DialogPrm(1).Data;
    b = block.DialogPrm(2).Data;
    m = block.DialogPrm(3).Data;
    I = block.DialogPrm(4).Data;
    c_f = block.DialogPrm(5).Data;
    c_r = block.DialogPrm(6).Data;
    x_0 = block.DialogPrm(7).Data;

    % States and Inputs
    X = block.ContStates.Data;
    U = block.InputPort(1).Data;
    
    % Bicycle CT Dynamics
    block.Derivatives.Data = nonlin_ct_bicycle(X, U, c, wb, m);
%end Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate

