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
gridLow = [lowEnv;-pi];
gridUp = [upEnv;pi];
N = [31;31;21];

% Timestep for computation and simulation.
dt = 0.05;

% Initial condition.
xinit = [2.0; 2.5; pi/2];

% Goal position.
xgoal = [8.5; 2.5; -pi/2];

%% Create plotter.
plt = Plotter(lowEnv, upEnv, lowRealObs, upRealObs, obsShape);

%% Construct sensed region.

% get the sensing radius (circle) 
senseShape = 'circle';
senseRad = 1.5;
senseData = [[xinit(1);xinit(2)], [senseRad;senseRad]];

%% Compute first safe set based on sensing. 

% If we want to warm start with prior value function.
warmStart = true;

% If we want to save the sequence of value functions.
saveValueFuns = false;
filename = 'localVxLxE005.mat';

% If we want to load data for any numerical comparisons. 
runComparison = false;

% Update epislon -- used in HJIPDE_solve_local()
updateEpsilon = 0.005;

% Setup avoid set object and compute first set.
currTime = 1;
setObj = AvoidSet(gridLow, gridUp, lowRealObs, upRealObs, obsShape, ...
    xinit, N, dt, updateEpsilon, warmStart, saveValueFuns, runComparison);
setObj.computeAvoidSet(senseData, senseShape, currTime);

%% Plot initial conditions, sensing, and safe set.
hold on

% Plot l(x) and V(x).
visSet = true;
cmapHot = 'hot';
cmapBone = 'bone';
valueFunc = plt.plotFuncLevelSet(setObj.grid, setObj.valueFun(:,:,:,end), xinit(3), visSet, [1,0,0], cmapHot);
%beliefObstacle = plt.plotFuncLevelSet(setObj.grid, setObj.lCurr, x(3), visSet, [0.5,0.5,0.5], cmapBone);

% Plot environment, car, and sensing.
envHandle = plt.plotEnvironment();
senseVis = plt.plotSensing(xinit, senseRad, senseShape);
carVis = plt.plotCar(xinit);
plt.plotBoundaryPadding(setObj.boundLow, setObj.boundUp);

%% Simulate dubins car moving around environment and the safe set changing

% Total simulation timesteps.
T = 200; 
x = setObj.dynSys.x;

for t=1:T
    % Get the current control.
    u = getControl(t);
    
    % Check if we are on boundary of safe set. If we are, apply safety 
    % controller instead. 
    [uOpt, onBoundary] = setObj.checkAndGetSafetyControl(x);
    if onBoundary
       u = uOpt;
    end

    % Apply control to dynamics.
    setObj.dynSys.updateState(u, dt, setObj.dynSys.x);
    x = setObj.dynSys.x;
    
    % Get the new sensing radius (circle).
    senseData = [[x(1);x(2)],[senseRad;senseRad]];    
    
	% -------------------- DEBUGGING -------------------- %
    % x = 4.8753, 5.2864, 0.0708
    %if abs(x(1) - 4.8753) < 0.01 && abs(x(2) - 5.2864) < 0.01
    %    theta = x(3);
    %    setObj.checkIfErrorVanishes(senseData, theta)
    %end
    % --------------------------------------------------- %
    
    % Update l(x) and the avoid set.
    setObj.computeAvoidSet(senseData, senseShape, t+1);
    
    % -------------- Plotting -------------- %
    % Delete old visualizations.
    delete(valueFunc);
    %delete(beliefObstacle);
    
    % Plot belief obstacle (i.e. everything unsensed) and the value function.
    % 	belief obstacle -- original l(x) which can be found at valueFun(1)
    % 	converged value function -- V_converged which can be found at valueFun(end)
    valueFunc = plt.plotFuncLevelSet(setObj.grid, setObj.valueFun(:,:,:,end), x(3), visSet, [1,0,0], cmapHot);
    %beliefObstacle = plt.plotFuncLevelSet(setObj.grid, setObj.lCurr, x(3), visSet, [0,0,0], cmapBone);
    
	% Delete old visualizations.
    delete(carVis);
    delete(senseVis);
    
    % Plot the state of the car (point), environment, and sensing.
	senseVis = plt.plotSensing(x, senseRad, senseShape);
    carVis = plt.plotCar(x);
    envHandle = plt.plotEnvironment();
    % ----------------------------------- %
    
    % Pause based on timestep.
    pause(dt);
end

if saveValueFuns 
    % Save out the sequence of value functions.
    valueFunCellArr = setObj.valueFunCellArr; 
    lxCellArr = setObj.lxCellArr; 
    repo = what('safe_navigation');
    savePath = strcat(repo.path, '/data/', filename);
    save(savePath, 'valueFunCellArr', 'lxCellArr');
end