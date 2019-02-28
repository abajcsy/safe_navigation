function params = car3DHJILidar()
%% Environment Params.
% Setup environment bounds.
params.lowEnv = [0;0];
params.upEnv = [10;7];

% Obstacles lower & upper bounds in 2D
params.obsShape = 'rectangle';
params.obstacles = {{[4;1], [7;4]}};

%% Grid Params.
gridLow = [params.lowEnv;-pi];
gridUp = [params.upEnv;pi];
N = [31;31;21];
params.pdDims = 3;
params.grid = createGrid(gridLow, gridUp, N, params.pdDims);

%% Planning Params.
params.xinit = [2.0; 2.5; pi/2];
params.xgoal = [8.5; 2.5; -pi/2];

%   hand-engineered trajectory      --> 'hand'
%   rapidly-exploring random-tree   --> 'rrt'
params.plannerName = 'rrt';
params.maxIter = 50;   % max number of iterations
params.dx = 0.01;      % size of step along edges for collision-checking
params.rrtGoalEps = 0.3;    % how close RRT has to sample to goal.

%% Dynamical System Params.
params.wMax = 1;
params.vrange = [0.5,1];

% Define dynamic system.            
% Create dubins car where u = [v, w]
params.dynSys = Plane(params.xinit, params.wMax, params.vrange);

%% Safety Update Params.

% Use this to toggle the safety computation on/off.
params.useSafety = false;

% What kind of update method do we want to use?
%   typical solver                  --> 'HJI'
%   local Q algorithm               --> 'localQ' 
params.updateMethod = 'HJI';

% If we want to warm start with prior value function.
params.warmStart = false;

% Update epislon
%   used in 'localQ' for determining which states to update
%   used in 'HJI' for convergenceThreshold 
params.updateEpsilon = 0.01;

% Control is trying to maximize value function.
params.uMode = 'max';

% Time horizon to compute BRT for.
params.tMax = 50;

%% Sensing Params.
params.senseShape = 'lidar';
params.senseRad = 3;
params.initSenseData = {[params.xinit(1);params.xinit(2);params.xinit(3)], [params.senseRad]};

%% Simulation Params.
% Timestep for computation and simulation.
params.dt = 0.05;
params.T = 800; 

% Threshold for when we are considered close enough to goal, we stop simulation.
params.goalEps = 0.4;

% Variables for determining when to replan & reupdate safe set.
params.planFreq = 10;
params.safetyFreq = 10;

% How close to the boundary we need to be to apply safety control.
params.safetyTol = 0.2;

%% Plotting Params.

% Do we want to visualize the simulation?
% (say false if you want to save on speed and just save out results).
params.visualize = true;

%% Data Saving Params. 
% If we want to save the sequence of value functions, compute times, etc..
params.saveOutputData = true;

% Create filename if we want to save things out.
% Naming convention:
%   [updateMethod][warm][sensing][date].mat
if params.saveOutputData
    if params.warmStart
        name = strcat(params.updateMethod, 'warm');
    else
        name = params.updateMethod;
    end
    name = strcat(name, params.senseShape);
    name = strcat(name, params.dynSys.nx, 'D');
    params.filename = strcat(name, datestr(now,'YYYYMMDD_hhmmss'),'.mat');
end
end
