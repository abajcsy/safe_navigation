%% This script creates a figures to explain the localQ method.
clf
clear all

% Load params.
%params = car3DLocalQLidarRRT();
params = car3DLocalQCameraSpline();

% Load the safe sets.
load('/home/abajcsy/hybrid_ws/src/safe_navigation/data/localQwarm_spline_camera_hand.mat');
%load('/home/abajcsy/hybrid_ws/src/safe_navigation/data/localQwarm_rrt_lidar_hand.mat');

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

initLxColor = mediumGrey; %lightGreyRed;
initLxOutline = darkGrey;
initVxColor = brightBlue;
initStateIdx = 12;
initVxIdx = 2;

%% ---------- Initial state setup. ----------- %%
f1 = figure(1);
hold on 
% Plotting stuff.
xlim([params.lowEnv(1) params.upEnv(1)]);
ylim([params.lowEnv(2) params.upEnv(2)]);
xlabel('$p_x$', 'Interpreter', 'latex', 'fontsize', 20);
ylabel('$p_y$', 'Interpreter', 'latex', 'fontsize', 20);
xticks([]);
yticks([]);

state = states{initStateIdx}; 
initValueFun = valueFunCellArr{initVxIdx}; 

% Plot the car.
carh = [];
plotCar(state, 'k', 1.0, false, carh);

% Grab 2D slice of the initial value function.
[initGrid2D, initData2D] = proj(params.grid, initValueFun, [0 0 1], state(3));

% Grab the initial belief obstacle.
initLx = lxCellArr{initVxIdx};
[initGrid, init_belief_obstacle] = proj(params.grid, initLx, [0 0 1], state(3));

% Plot the sensing region.
% sensing_region = safeOccuMaps{initStateIdx};
% [c, h]= contourf(initGrid2D.xs{1}, initGrid2D.xs{2}, sensing_region, [0,0]);
% x = c(1,2:end);
% y = c(2,2:end);
% delete(h);
% h = fill(x,y,[0,0.2,1],'FaceAlpha',0.3, 'EdgeColor', 'none');

% Plot the belief obstacle & value function.
ax1 = axes;
extraArgs.LineWidth = 2;
[c, h]= contourf(ax1, initGrid2D.xs{1}, initGrid2D.xs{2}, -init_belief_obstacle, [0,0], ...
    'linecolor', initLxOutline, 'linewidth', 1.5);
colormap(ax1, initLxColor);
ax1.Visible = 'off';
xticks([]);
yticks([]);

% Plot the value function.
ax2 = axes;
extraArgs.LineWidth = 2;
[c, h]= contour(ax2, initGrid2D.xs{1}, initGrid2D.xs{2}, initData2D, [0,0], ...
    'linecolor', initVxColor, 'linewidth', 2);
ax2.Visible = 'off';
xticks([]);
yticks([]);

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