% Clear old figure plotting and variables.
clf 
clc 
clear 
close all;

% Load the occupancy map
load('/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/slam_lx_2D.mat');

% Construct the grid for the HJI computation
gridLow = [grid.min;-pi;-0.1];
gridUp = [grid.max;pi;0.6];
N = [grid.N; 11; 11];
g = createGrid(gridLow, gridUp, N, 3);

% Construct data0 for the HJI computation
data0 = repmat(flipud(lx_2D'), 1, 1, N(3), N(4));
t0 = 0;
tMax = 20;
dt = 0.02;
tau = t0:dt:tMax;

% % Make sure the data0 is correct
% f = figure(1);
% hold on
% [grid2D, data2D] = proj(g, data0, [0 0 1 1], [pi/2 0]);
% contourf(grid2D.xs{1}, grid2D.xs{2}, data2D, [0,0]);

% Construct the dynamics
wMax = 1.1;              % maxangular control
aRange = [-0.6, 0.6];    % acceleration control range
vRange = [0.0, 0.2];     % speed range
xinit = [0; 0; 0.; 0.];
dynSys = Plane4D(xinit, wMax, aRange, vRange);

% Compute the reachable set
% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dynSys;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = 'max';

%% Compute value function
HJIextraArgs.visualize = false; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update
HJIextraArgs.stopConverge = 1;
HJIextraArgs.ignoreBoundary = 0;
HJIextraArgs.convergeThreshold = 0.02;
HJIextraArgs.targets = data0;

[data, tau2, ~] = ...
  HJIPDE_solve(data0, tau, schemeData, 'minVwithL', HJIextraArgs);

save('/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/brs_Vx_4D.mat', 'data', 'g');
