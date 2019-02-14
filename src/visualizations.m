%% Visualizes stuff in a pretty way.
clf

%% Choose which method you wanna visualize.
%   HJI, localQ
method = 'localQ';
warm = true;

%% Grab the data from both local updates.
path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data/';
% Load the compute grid.
load(strcat(path, 'grid.mat'));
load(strcat(path, 'trajectory.mat'));
if strcmp(method, 'HJI')
    if warm
        title('Warm Start HJI-VI', 'Interpreter','latex', 'fontsize',16);
        filePath = strcat(path, 'HJIwarm.mat');
        load(filePath);
        color = [0.1,0.1,1.0];
    else
        title('HJI-VI', 'Interpreter','latex', 'fontsize',16);
        filePath = strcat(path, 'HJI.mat');
        load(filePath);
        color = [0,0,0];
    end
elseif strcmp(method, 'localQ')
    title('Local Update Algorithm', 'Interpreter','latex', 'fontsize',16);
    filePath = strcat(path, 'localQwarm.mat');
    load(filePath);
    color = [1.0,0.1,0.1];
else
    error('Unsupported method type.');
end

valueFuns = valueFunCellArr;

%% Visualize

% Setup environment bounds.
lowEnv = [0;0];
upEnv = [10;7];

% Create lower and upper bounds on rectangle.
lowRealObs = [4;1];
upRealObs = [7;4];
obsShape = 'rectangle';
plt = Plotter(lowEnv, upEnv, lowRealObs, upRealObs, obsShape);

figure(1)
dt = 0.05;
hold on;
for i=1:length(valueFuns)
    
    % plot value function
    currFun = valueFuns{i};
    extraArgs.LineWidth = 1.5;
    currState = trajectory{i}{1};
    [gPlot, plotData] = proj(grid, currFun, [0 0 1], currState(3));
    
    e = plt.plotEnvironment();    
    s = plt.plotSensing(gPlot, fovCellArr{i});
    c = plt.plotCar(currState);
    
    set = visSetIm(gPlot, plotData, color, 0, extraArgs);
    pause(dt);
    if i ~= length(valueFuns)
        delete(set);
        delete(e);
        delete(s);
        delete(c);
    end
end

