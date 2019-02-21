%% Run the safety & planning procedure. 
% TODO:
% - implement planner and optimal controller scheme
% - what if we knew some prior info about environment and other stuff we didnt?

% Clear old figure plotting and variables.
clf 
clc 
clear all

% Setup environment bounds.
lowEnv = [0;0];
upEnv = [10;7];

% Are we using a 2D or a 3D system?
numDims = 3;

% Setup obstacle ['rectangle' or 'circle'].
obsShape = 'rectangle';
if strcmp(obsShape, 'rectangle')
    % Create lower and upper bounds on rectangle.
    lowRealObs = [4;1];
    upRealObs = [7;4];
else
    % Setup circular obstacle.
    lowRealObs = [5.5;2.5]; % center of circle
    upRealObs = [1.5;1.5]; % radius of circle
end

% Setup lower and upper computation domains and discretization.
if numDims == 2
    gridLow = lowEnv;
    gridUp = upEnv;
    N = [31;31];
elseif numDims == 3
    gridLow = [lowEnv;-pi];
    gridUp = [upEnv;pi];
    N = [31;31;21];
else
    error('I cannot handle %d number of dims!\n', numDims);
end

% Timestep for computation and simulation.
dt = 0.05;

% Initial and final condition.
if numDims == 3
    xinit = [2.0; 2.5; pi/2];
    xgoal = [8.5; 2.5; -pi/2];
else
    xinit = [2.0; 2.5];
    xgoal = [8.5; 2.5];
end

%% Construct sensed region.
senseShape = 'camera';

if strcmp(senseShape, 'circle')
  senseRad = 1.5;
  senseData = {[xinit(1);xinit(2);xinit(3)], [senseRad;senseRad]};
elseif strcmp(senseShape, 'camera')
  initialR = 1.5; % The initial radius of the safe region
  senseFOV = pi/6; % The (half) field-of-view of the camera
  senseData = {[xinit(1);xinit(2);xinit(3)], [senseFOV; initialR]};
else
  error('unknown sesnor type');
end

%% Compute first safe set based on sensing. 

% What kind of update method do we want to use?
%   typical solver                  --> 'HJI'
%   local Q algorithm               --> 'localQ' 
updateMethod = 'localQ';

% If we want to warm start with prior value function.
warmStart = true;

% Update epislon
%   used in 'localQ' for determining which states to update
%   used in 'HJI' for convergenceThreshold 
updateEpsilon = 0.01;

% If we want to save the sequence of value functions, compute times, etc..
saveOutputData = false;
if warmStart
    name = strcat(updateMethod, 'warm');
else
    name = updateMethod;
end
filename = strcat(name, datestr(now,'YYYYMMDD_hhmmss'),'.mat');

% Setup avoid set object and compute first set.
currTime = 1;
setObj = AvoidSet(gridLow, gridUp, lowRealObs, upRealObs, obsShape, ...
    xinit, N, dt, updateEpsilon, warmStart, updateMethod);

% Update occupancy grid based on sensing. 
setObj.updateOccupancyMap(senseData, senseShape);

% Compute avoid set based on current sensing.
%setObj.computeAvoidSet(currTime);

%% Plot initial conditions, sensing, and safe set.
hold on

% Plot environment, car, and sensing.
plt = Plotter(lowEnv, upEnv, lowRealObs, upRealObs, obsShape);
plt.updatePlot(xinit, xgoal, setObj);
pause(dt);

%% Setup which planner you want to use.
%   hand-engineered trajectory      --> 'hand'
%   rapidly-exploring random-tree   --> 'rrt'
% (todo)
%   spline-based planner            --> 'spline'
%   learning, vision-based planner  --> 'learn'
plannerName = 'rrt';

if strcmp(plannerName, 'rrt')
    % setup parameters
    maxIter = 1000;
    dx = 0.01; % size of step along edges for collision-checking
    showTree = false;
    
    % Create RRT obj.
    planner = RRT(setObj.grid, setObj.occupancy_map_plan);
    % build rrt
    nodes = planner.build(xinit(1:2), xgoal(1:2), maxIter, dx, showTree);
    % get optimal path
    path = nodes.getPath(xgoal(1:2));
    % (optional) plot optimal path
    planner.plotPath(path);
    
    % Create PID controller to track RRT trajectory.
    controller = PIDController(setObj.dynSys);
    controller.updatePath(path);
end

%% Simulate dubins car moving around environment and the safe set changing

% Total simulation timesteps.
T = 500; 
x = setObj.dynSys.x;

% Threshold for where 
goalEps = 0.3;

for t=1:T
    % Switch control and planning based on planner.
    if strcmp(plannerName, 'hand')
        u = getControlDubins(t);
    elseif strcmp(plannerName, 'rrt')
        u = controller.getControl(t, x);
    else
        error("Can't run unsupported planner! %s\n", plannerName);
    end
    
    % If we are close enough to the goal, stop simulation.
    if norm(x(1:2) - xgoal(1:2)) < goalEps
        break;
    end
    
%     % Check if we are on boundary of safe set. If we are, apply safety 
%     % controller instead. 
%     [uOpt, onBoundary] = setObj.checkAndGetSafetyControl(x);
%     if onBoundary
%        u = uOpt;
%     end

    % Apply control to dynamics.
    setObj.dynSys.updateState(u, dt, setObj.dynSys.x);
    x = setObj.dynSys.x;
    
    % Get the new sensing region.
    if strcmp(senseShape, 'circle')
      senseData = {[x(1);x(2);x(3)], [senseRad;senseRad]};
    elseif strcmp(senseShape, 'camera')
      senseData = {[x(1);x(2);x(3)], [senseFOV; initialR]};
    else
      error('unknown sesnor type');
    end  
    
    % Update occupancy map, l(x), and the avoid set.
    setObj.updateOccupancyMap(senseData, senseShape);
    %setObj.computeAvoidSet(t+1);
    
    % Update plotting.
	plt.updatePlot(x, xgoal, setObj);
    
    % Pause based on timestep.
    pause(dt);
end

if saveOutputData
    % Save out the sequence of value functions.
    valueFunCellArr = setObj.valueFunCellArr; 
    lxCellArr = setObj.lxCellArr; 
    QSizeCellArr = setObj.QSizeCellArr;
    solnTimes = setObj.solnTimes;
    fovCellArr = setObj.fovCellArr;
    repo = what('safe_navigation');
    savePath = strcat(repo.path, '/data/', filename);
    save(savePath, 'valueFunCellArr', 'lxCellArr', 'QSizeCellArr', 'solnTimes', 'fovCellArr');
end