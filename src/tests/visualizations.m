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
    filePath = strcat(path, 'localQwarmcamera_abajcsy.mat');
    load(filePath);
    color = [1.0,0.1,0.1];
    params = dubinsLocalQCameraExp1();
else
    error('Unsupported method type.');
end

valueFuns = valueFunCellArr;

%% Visualize
plt = Plotter(params.lowEnv, params.upEnv, params.obstacles);

figure(1)
hold on;
for i=1:length(valueFuns)
    
    % plot value function
    currFun = valueFuns{i};
    extraArgs.LineWidth = 1.5;
    currState = states{i};
    path = paths{i};
    [gPlot, plotData] = proj(grid, currFun, [0 0 1], currState(3));
    
    e = plt.plotEnvironment();    
    signed_dist_map = occuMaps{i};
    s = plt.plotSensing(gPlot, signed_dist_map);
    c = plt.plotCar(currState);
    
    set = visSetIm(gPlot, plotData, color, 0, extraArgs);
    pause(params.dt);
    if i ~= length(valueFuns)
        delete(set);
        delete(e);
        delete(s);
        delete(c);
    end
end

