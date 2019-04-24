%% This script creates a figures to explain the localQ method.
clf
clear all

% Load params.
params = car3DLocalQCameraSpline();
%params = car4DLocalQCameraRRT();

% Load the safe sets.
load('/home/abajcsy/hybrid_ws/src/safe_navigation/data/localQwarm_spline_camera_hand.mat');
%load('/home/abajcsy/hybrid_ws/src/safe_navigation/data/localQwarm_rrt_camera_4D_hand.mat');

%% Colors.
lightGreyBlue = [152, 158, 170]/255.;     
darkGreyBlue = [114, 119, 130]/255.;  
brightBlue = [0, 88, 255]/255.;        

lightGreyRed = [183, 130, 130]/255.;    
darkGreyRed = [135, 97, 107]/255.;   
brightRed = [234, 0, 62]/255.;   
trueRed = [255, 0, 0]/255.;

lightGreyPurple = [181, 156, 186]/255.;
darkGreyPurple = [118, 100, 122]/255.;
brightPurple = [212, 0, 255]/255.;

lightGreyOrange = [242, 208, 162]/255.;
darkGreyOrange = [191, 156, 109]/255.;
brightOrange = [255, 148, 0]/255.;

mediumGrey = [132, 132, 132]/255.;
darkGrey = [73, 73, 73]/255.;

nextLxColor = mediumGrey;
nextLxOutline = darkGrey; 
nextVxColor = trueRed; 
nextStateIdx = 22;
nextVxIdx = 3;

initVxColor = brightOrange;
initStateIdx = 12;
initVxIdx = 2;

%% -------- Do the localQ update and visualize how it works ------- %%
f3 = figure(3);
set(gcf,'color','w');
clf
hold on

% Plot the (initial) value function.
%seth = visSetIm(initGrid2D, initData2D, initVxColor, 0, extraArgs);

% Grab the value function and state.
state = states{nextStateIdx};
nextValueFun = valueFunCellArr{nextVxIdx};
initState = states{initStateIdx};
initLx = lxCellArr{initVxIdx};
nextLx = lxCellArr{nextVxIdx};

if params.dynSys.nx == 3
    % Grab 2D slice of the next value function.
    [nextGrid2D, nextData2D] = proj(params.grid, nextValueFun, [0 0 1], [state(3)]);

    % Get 2D slice of the initial cost function. 
    [initGrid, init_obs] = proj(params.grid, initLx, [0 0 1], initState(3));

    % Get 2D slice of the next cost function. 
    [nextGrid, next_belief_obstacle] = proj(params.grid, nextLx, [0 0 1], state(3));
elseif params.dynSys.nx == 4
    % Grab 2D slice of the next value function.
    [nextGrid2D, nextData2D] = proj(params.grid, nextValueFun, [0 0 1 1], [state(3), state(4)]);

    % Get 2D slice of the initial cost function. 
    [initGrid, init_obs] = proj(params.grid, initLx, [0 0 1 1], [initState(3), initState(4)]);

    % Get 2D slice of the next cost function. 
    [nextGrid, next_belief_obstacle] = proj(params.grid, nextLx, [0 0 1 1], [state(3), state(4)]);
else
	error('I cannot run visualizations with a %dD system!', params.dynSys.nx);
end

% Plot new belief obstacle.
[c, h]= contourf(nextGrid2D.xs{1}, nextGrid2D.xs{2}, -next_belief_obstacle, [0,0], ...
    'linecolor', nextLxOutline, 'linewidth', 1.5);
colormap(nextLxColor);

carh = plotCar(state, 'k', 1, false, []);

% Plotting stuff.
xlim([params.lowEnv(1) params.upEnv(1)]);
ylim([params.lowEnv(2) params.upEnv(2)]);
xticks([]);
yticks([]);

% Setup data.
data0 = valueFunCellArr{initVxIdx};
lxOld = lxCellArr{initVxIdx};
lx = lxCellArr{nextVxIdx};

% Parameter choices.
updateEpsilon = 0.01;
minWith = 'minVWithL';
tau = 0:params.dt:10; 
schemeData.grid = params.grid;
schemeData.dynSys = params.dynSys;
schemeData.accuracy = 'high'; % Set accuracy.
schemeData.uMode = params.uMode;
if ~isempty(params.dMode)
    schemeData.dMode = params.dMode;
end
HJIextraArgs.ignoreBoundary = 0; 
HJIextraArgs.quiet = true;
HJIextraArgs.stopConverge = 0;
HJIextraArgs.targets = lx;
if params.dynSys.nx == 3
    schemeData.hamFunc = @dubins3Dham_localQ;
    schemeData.partialFunc = @dubins3Dpartial_localQ;
elseif params.dynSys.nx == 4
    schemeData.hamFunc = @plane4Dham_localQ;
    schemeData.partialFunc = @plane4Dpartial_localQ;
else
	error('I cannot run visualizations with a %dD system!', params.dynSys.nx);
end
                    
% Solve.
[data, tau, extraOuts] = ...
    HJIPDE_solve_localQ_visualize(data0, lxOld, lx, updateEpsilon, ...
        tau, schemeData, minWith, HJIextraArgs, initVxColor, nextVxColor, ...
        f3, state, next_belief_obstacle, nextGrid2D, init_obs);

%% Plots dubins car point and heading.
% Inputs:
%   x [vector]  - 3D/4D state of dubins car
% Ouput:
%   c   - handle for figure
function c = plotCar(x, carColor, alpha, isUnsafe, carh)

c = quiver(x(1), x(2), cos(x(3)), sin(x(3)), 'o', 'Color', carColor, ...
    'MarkerSize', 5, 'MarkerEdgeColor', carColor, ...
    'MarkerFaceColor', carColor, 'MaxHeadSize', 5.0, ...
    'ShowArrowHead', 'on', 'AutoScaleFactor', 0.3, ...
    'LineWidth', 1.4); 

end