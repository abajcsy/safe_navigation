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
if strcmp(method, 'HJI')
    if warm
        title('Warm Start HJI-VI', 'Interpreter','latex', 'fontsize',16);
        filePath = strcat(path, 'HJIwarmcamera_dawkins.mat');
        load(filePath);
        color = [0.1,0.1,1.0];
        params = dubinsWarmCameraExp1();
    else
        title('HJI-VI', 'Interpreter','latex', 'fontsize',16);
        filePath = strcat(path, 'HJIcamera_dawkins.mat');
        load(filePath);
        color = [0,0,0];
        params = dubinsHJICameraExp1();
    end
elseif strcmp(method, 'localQ')
    title('Local Update Algorithm', 'Interpreter','latex', 'fontsize',16);
    filePath = strcat(path, 'localQwarmcamera20193326_200220.mat');
    load(filePath);
    color = [1.0,0.1,0.1];
    params = dubinsLocalQCameraExp1();
else
    error('Unsupported method type.');
end

valueFuns = valueFunCellArr;

%% Visualize

offsetX = (grid.max(1) - grid.min(1))/grid.N(1);
offsetY = (grid.max(2) - grid.min(2))/grid.N(2);

boundLow = [grid.min(1)+offsetX, grid.min(2)+offsetY, grid.min(3)];
boundUp = [grid.max(1)-offsetX, grid.max(2)-offsetY, grid.max(3)];

plt = Plotter(params.lowEnv, params.upEnv, boundLow, boundUp, params.obstacles);

hold on;
idx = 1;
currUpdateT = updateTimeArr(idx);
for i=1:length(states)
    if i == currUpdateT
        % plot value function
        currFun = valueFuns{idx};
        idx = idx + 1;
        currUpdateT = updateTimeArr(idx);
    end
    
    % grab important variables.
    x = states{i};
    path = paths{i};
    occuMap = occuMaps{i};
    [g2D, ~] = proj(grid, currFun, [0 0 1], x(3));
    
    % update plot.
    plt.updatePlot(x, params.xgoal, currFun, ...
                grid, g2D, occuMap, path);
    
    pause(params.dt);
    
end

