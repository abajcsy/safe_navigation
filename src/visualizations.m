%% Visualizes stuff in a pretty way.

%% Choose which method you wanna visualize.
%   HJI, globalQ, localQ
method = 'HJI';
warm = false;
inheritVals = 'lx';

%% Grab the data from both local updates.
path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data/';
% Load the compute grid.
load(strcat(path, 'grid.mat'));
load(strcat(path, 'trajectory.mat'));
if strcmp(method, 'HJI')
    if warm
        filePath = strcat(path, 'warmHJI_reduced.mat');
        load(filePath);
        valueFuns = valueFunCellArr;
        color = [0.5,0.5,0.5];
    else
        filePath = strcat(path, 'groundTruth_reduced.mat');
        load(filePath);
        valueFuns = reducedValueFunCellArr;
        color = [0,0,0];
    end
elseif strcmp(method, 'globalQ')
    if warm
        filePath = strcat(path, 'warmGlobalQ_reduced.mat');
        load(filePath);
        valueFuns = valueFunCellArr;
        color = [0.1,0.1,0.6];
    else
        filePath = strcat(path, 'globalQ_reduced.mat');
        load(filePath);
        valueFuns = valueFunCellArr;
        color = [0.1,0.1,1.0];
    end
elseif strcmp(method, 'localQ')
	if strcmp(inheritVals, 'lx')
        filePath = strcat(path, 'hackySignedDistanceWarmStart_reduced.mat');
        load(filePath);
        valueFuns = reducedValueFunCellArr;
        color = [0.6,0.1,0.1];
    else
        filePath = strcat(path, 'pureWarmStart_reduced.mat');
        load(filePath);
        valueFuns = reducedValueFunCellArr;
        color = [1.0,0.1,0.1];
    end
else
    error('Unsupported method type.');
end

%% Visualize

figure(1)
hold on;
for i=1:length(valueFuns)
    currFun = valueFuns{i};
    extraArgs.LineWidth = 2.0;
    currState = trajectory{i}{1};
    [gPlot, plotData] = proj(grid, currFun, [0 0 1], currState(3));
    set = visSetIm(gPlot, plotData, color, 0, extraArgs);
    pause(0.02);
    if i ~= length(valueFuns)
        delete(set);
    end
end

