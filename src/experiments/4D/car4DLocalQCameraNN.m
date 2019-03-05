function params = car4DLocalQCameraNN()
%% Environment Params.
% Setup environment bounds.
params.lowEnv = [0;0];
params.upEnv = [10;7];

%% ROS Params.
params.verifiedTopicName = '/verified_traj';
params.occuMapTopicName = '/map';
params.planTopicName = '/planned_traj';

%% Grid Params.
gridLow = [params.lowEnv;-pi;0.0];
gridUp = [params.upEnv;pi;0.6];
N = [31;31;21;11];
params.pdDims = 3;
params.grid = createGrid(gridLow, gridUp, N, params.pdDims);

%% Planning Params.
params.xinit = [2.0; 2.5; pi/2; 0.];
params.xgoal = [8.5; 2.5; -pi/2; 0.];

%   neural network, vision-based planner      --> 'nn'
params.plannerName = 'nn';

%% Dynamical System Params.
params.wMax = 1.1;          % maxangular control
params.aRange = [-0.4, 0.4];    % acceleration control range

% Define dynamic system. 
params.dynSys = Plane4D(params.xinit, params.wMax, params.aRange);

%% Safety Update Params.

% Use this to toggle the safety computation on/off.
params.useSafety = true;

% What kind of update method do we want to use?
%   typical solver                  --> 'HJI'
%   local Q algorithm               --> 'localQ' 
params.updateMethod = 'localQ';

% If we want to warm start with prior value function.
params.warmStart = true;

% Update epislon
%   used in 'localQ' for determining which states to update
%   used in 'HJI' for convergenceThreshold 
params.updateEpsilon = 0.02;

% Control is trying to maximize value function.
params.uMode = 'max';

% Time horizon to compute BRT for.
params.tMax = 50;

%% Sensing Params.
params.senseShape = 'camera';
params.initialR = 1.5; % The initial radius of the safe region
params.senseFOV = pi/6; % The (half) field-of-view of the camera
params.initSenseData = {[params.xinit(1);params.xinit(2);params.xinit(3)], [params.senseFOV; params.initialR]};

%% Simulation Params.
% Timestep for computation and simulation.
params.dt = 0.05;

% Variables for determining when to replan & reupdate safe set.
params.planFreq = 10;
params.safetyFreq = 10;

% How close to the boundary we need to be to apply safety control.
params.safetyTol = 0.2;

end
