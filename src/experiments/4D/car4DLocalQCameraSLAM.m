function params = car4DLocalQCameraSLAM()
%% Environment Params.
% Setup environment bounds.

% experiment in the bay
lowEnvBay = [-1; -2]; 
upEnvBay = [6; 5]; 

% experiment in hall
lowEnvHall = [-1; -4];
upEnvHall = [7; 4];

params.lowEnv = lowEnvHall; 
params.upEnv = upEnvHall;

% Environment types include:
%   hand-coded obstacles        --> 'hand'
%   stanford building dataset   --> 'sbpd'
%   SLAM environment            --> 'slam'
params.envType = 'slam'; 

%% ROS Params.
params.verifiedTopicName = '/verified_traj';
params.occuMapTopicName = '/slam_map';
params.planTopicName = '/planned_traj';

%% Grid Params.
gridLow = [params.lowEnv;-pi;-0.1];
gridUp = [params.upEnv;pi;0.7];
N = [31;31;21;11]; 
params.pdDims = 3;
params.grid = createGrid(gridLow, gridUp, N, params.pdDims);

%% Planning Params.
xgoalBay = [4.9; 4.26; 0.; 0.];
xgoalHall = [6; 1; 0.; 0.];

params.xinit = [0; 0; 0.; 0.];    
params.xgoal =  xgoalHall;

%   neural network, vision-based planner      --> 'nn'
params.plannerName = 'nn';
params.loadTrueOccuMaps = false; % if we can load in ground-truth occupancy maps.
params.goalEps = 0.3; % threshold for when we are considered close enough to goal.

%% Dynamical System Params.
params.wMax = 0.7;              % maxangular control
params.aRange = [-0.6, 0.6];    % acceleration control range
params.vRange = [0.0, 0.6];     % speed range

% Define dynamic system. 
params.dynSys = Plane4D(params.xinit, params.wMax, params.aRange, params.vRange);

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
params.updateEpsilon = 0.01;

% Control is trying to maximize value function.
params.uMode = 'max';
params.dMode = [];  % we don't want to compute with disturbance.

% Time horizon to compute BRT for.
params.tMax = 2;

%% Sensing Params.
params.senseShape = 'camera';
params.initialR = 0.6;  % The initial radius of the safe region
params.senseFOV = pi/4; % The (half) field-of-view of the camera
params.farPlane = 20;   % The far clipping plane of the camera
params.initSenseData = {[params.xinit(1);params.xinit(2);params.xinit(3)], ...
    [params.senseFOV; params.initialR; params.farPlane]};

%% Simulation Params.
% Timestep for computation and simulation.
params.dt = 0.05;

% Variables for determining when to replan & reupdate safe set.
params.planFreq = 10;
params.safetyFreq = 10;

% How close to the boundary we need to be to apply safety control.
params.safetyTol = 0.2;
end
