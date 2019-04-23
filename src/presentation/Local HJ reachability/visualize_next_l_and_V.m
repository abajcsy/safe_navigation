%% This script creates a figures to explain the localQ method.
clf
clear all

% Load params.
%params = car4DLocalQCameraSLAM();
%params = car3DLocalQLidarRRT();
params = car3DLocalQCameraSpline();

% Load the safe sets.
load('/home/abajcsy/hybrid_ws/src/safe_navigation/data/localQwarm_spline_camera_hand.mat');

%% Colors.
lightGreyBlue = [152, 158, 170]/255.;     
darkGreyBlue = [114, 119, 130]/255.;  
brightBlue = [0, 88, 255]/255.;        

lightGreyRed = [183, 130, 130]/255.;    
darkGreyRed = [135, 97, 107]/255.;   
brightRed = [234, 0, 62]/255.;      

lightGreyPurple = [181, 156, 186]/255.;
darkGreyPurple = [118, 100, 122]/255.;
brightPurple = [212, 0, 255]/255.;

mediumGrey = [132, 132, 132]/255.;
darkGrey = [73, 73, 73]/255.;

nextLxColor = mediumGrey;
nextLxOutline = darkGrey; 
nextVxColor = brightRed;
nextStateIdx = 22;
nextVxIdx = 3;

initVxColor = brightBlue;
initStateIdx = 12;
initVxIdx = 2;

%% ---------- Next state setup. ----------- %%
f2 = figure(2);
set(f2, 'position', [0,0,970,728]); 
clf
hold on
state = states{nextStateIdx}; %stateCellArr{nextIdx};
nextValueFun = valueFunCellArr{nextVxIdx}; %setCellArr{nextIdx};

% Plot the car.
carh = [];
carh = plotCar(state, 'k', 1.0, false, carh);

% Grab 2D slice of the next value function.
[nextGrid2D, nextData2D] = proj(params.grid, nextValueFun, [0 0 1], [state(3)]);

% Plot the sensing region.
% sensing_region = safeOccuMaps{nextStateIdx};
% [c, h]= contourf(nextGrid2D.xs{1}, nextGrid2D.xs{2}, sensing_region, [0,0]);
% x = c(1,2:end);
% y = c(2,2:end);
% delete(h);
% h = fill(x,y,[0,0.2,1],'FaceAlpha',0.3, 'EdgeColor', 'none');

% Get the old cost function.
initLx = lxCellArr{initVxIdx};
initState = states{initStateIdx};
[initGrid2D, init_obs] = proj(params.grid, initLx, [0 0 1], initState(3));

% Plot the new belief obstacle & value function.
extraArgs.LineWidth = 2;
[c, h]= contour(initGrid2D.xs{1}, initGrid2D.xs{2}, -init_obs, [0,0], ...
    'linecolor', [0.7,0.7,0.7], 'linewidth', 1.5);

% Get new cost function. 
nextLx = lxCellArr{nextVxIdx};
[nextGrid, next_belief_obstacle] = proj(params.grid, nextLx, [0 0 1], state(3));

% Plot the new belief obstacle & value function.
extraArgs.LineWidth = 2;
[c, h]= contourf(nextGrid2D.xs{1}, nextGrid2D.xs{2}, -next_belief_obstacle, [0,0], ...
    'linecolor', nextLxOutline, 'linewidth', 1.5);
colormap(nextLxColor);

% Plot the (initial) value function.
initValueFun = valueFunCellArr{initVxIdx};
[initGrid2D, initData2D] = proj(params.grid, initValueFun, [0 0 1], states{initStateIdx}(3));
%seth = visSetIm(initGrid2D, initData2D, initVxColor, 0, extraArgs);

% Plot the (next) value function.
extraArgs.LineWidth = 3;
nsh = visSetIm(nextGrid2D, nextData2D, nextVxColor, 0, extraArgs);

% Plotting stuff.
xlim([params.lowEnv(1) params.upEnv(1)]);
ylim([params.lowEnv(2) params.upEnv(2)]);
xticks([]);
yticks([]);
xlabel('$p_x$', 'Interpreter', 'latex', 'fontsize', 30);
ylabel('$p_y$', 'Interpreter', 'latex', 'fontsize', 30);

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