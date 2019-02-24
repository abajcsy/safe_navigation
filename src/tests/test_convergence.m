%% File used to test why the warm-starting causes HJIPDE_solve to get 
%  stuck and unable to converge at some points in the environment. 

% Clear old figure plotting and variables.
clf 
clc 
clear all

% Load the V(x), l(x), lxOld(x), 
%   stuckRect -- gives data related to where computation gets stuck.
%   oneStep   -- gives data for one sensing step.
repo = what('safe_navigation');
path = strcat(repo.path, '/data/', 'stuckVxLxE005.mat');
load(path);

updateEpsilon = 0.005;
minWith = 'minVWithL';

extraArgs.stopConverge = 0;
extraArgs.ignoreBoundary = 1; 
extraArgs.targets = lx;

t0 = 0;
dt = 0.05;
tMax = 100; % tMax = 100 means compute infinite-horizon solution
times = t0:dt:tMax; 

% Setup environment bounds.
lowEnv = [0;0];
upEnv = [10;7];
obsShape = 'rectangle';
lowRealObs = [4;1];
upRealObs = [7;4];
plt = Plotter(lowEnv, upEnv, lowRealObs, upRealObs, obsShape);

[dataOut, tau, extraOuts] = ...
    HJIPDE_solve_local(data0, lxOld, lx, updateEpsilon, times, ...
    schemeData, minWith, extraArgs, plt);

%% Run the normal solver.

% Compute the solution.
%   HJIPDE_solve        -- typical solver.
%   HJIPDE_solve_local  -- solver that implements new ``local'' scheme.
% updateEpsilon = 0.01;
% extra.visualize.valueSet = 1;
% extra.visualize.initialValueSet = 1;
% extra.visualize.figNum = 1;
% extra.visualize.deleteLastPlot = true; 
% 
% extra.visualize.plotData.plotData = 1;
% extra.visualize.plotData.plotDims = [1 1 0];
% extra.visualize.plotData.projpt = pi/2;
% 
% extra.visualize.plotColorVS0 = [1,0,0.7]; % color of inital value set
% extra.visualize.plotColorVS = [1,0,0]; % color of final value set
% extra.visualize.plotColorTS = [0,0,1]; % color of new cost function
% extra.visualize.plotColorlxOld = [0,0,0]; % color of prior cost function

%legend('old V(x)', 'old l(x)', 'new l(x)', 'new V(x)');
%[dataOut, tau, extraOuts] = ...
%    HJIPDE_solve(lx, times, scheme, minWith, extra);

