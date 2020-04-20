clf 
clc 
clear all

% Setup environment bounds.
lowEnv = [-2.5;-2.5];
upEnv = [2.5;2.5];

% Are we using a 2D or a 3D system?
numDims = 3;

% Setup obstacle ['rectangle' or 'circle'].
% obsShape = 'rectangle';
% if strcmp(obsShape, 'rectangle')
%     % Create lower and upper bounds on rectangle.
%     lowRealObs = [2;2];
%     upRealObs = [7;4];
% else
%     % Setup circular obstacle.
%     lowRealObs = [5.5;2.5]; % center of circle
%     upRealObs = [1.5;1.5]; % radius of circle
% end

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
    xinit = [-2.0; -2.0; pi/2];
    xgoal = [2.0; 2.0; -pi/2];
else
    xinit = [-2.0; -2.0];
    xgoal = [2.0; 2.0];
end

obstacles = [-2, 1.5, 1, 1; -1.5, -1, 0.5, 3; 0.5, 0.5, 1, 1];
for i = length(obstacles):
    for j = length(obstacles):
        
    end 
end 
%% Construct sensed region.


%% Compute first safe set based on sensing. 

% What kind of update method do we want to use?
%   typical solver                  --> 'HJI'
%   local Q algorithm               --> 'localQ' 
updateMethod = 'HJI';

% If we want to warm start with prior value function.
warmStart = true;

% Update epislon
%   used in 'localQ' for determining which states to update
%   used in 'HJI' for convergenceThreshold 
updateEpsilon = 0.01;

% Setup avoid set object and compute first set.
trajectory = [-3., 1.1; -2.5, 2.; -1.3, 2.8; 0., 3.; 1.5, 3.; 3., 3.];

currTime = 1;
setObj = AvoidSet(gridLow, gridUp, lowRealObs, upRealObs, obsShape, ...
    xinit, N, dt, updateEpsilon, warmStart, updateMethod);
currTime = currTime + 1;
setObj.computeAvoidSet(senseData, senseShape, currTime);
currTime = currTime + 1; 
setObj.computeAvoidSet(senseData, senseShape, 2);
