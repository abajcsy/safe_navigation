clear all
close all

% ------- Neural Net Trajs -------- %
safe_traj = 'localQwarm_nn_camera_sbpd_safetytrue_traj.mat';
unsafe_traj = 'localQwarm_nn_camera_sbpd_safetyfalse_traj.mat';

% ------- Neural Net Data -------- %
localQ_nn_safe_camera_filename = 'localQwarm_nn_camera_sbpd_safetytrue.mat';
localQ_nn_unsafe_camera_filename = 'localQwarm_nn_camera_sbpd_safetyfalse.mat';

% Path info.
path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data/';
traj_path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data_traj/';

%% Grab file and params.

% Load traj.
filename = unsafe_traj;
filePath = strcat(traj_path, filename);
load(filePath);
params = car4DLocalQCameraNN();

% Load the ground truth map.
load(strcat(path,'sbpdTrueOccuMap.mat'));

% Load data.
if strcmp(filename, unsafe_traj)
    filePath = strcat(path, localQ_nn_unsafe_camera_filename);
else
    filePath = strcat(path, localQ_nn_safe_camera_filename);
end
load(filePath);

orange = [242./255., 160./255., 29./255.];
turquoise = [0, 166, 204]/255.;
lightGreen = [132, 216, 56]/255.;
dLightGreen = [80, 150, 16]/255.;
darkGreen = [0, 137, 41]/255.;
dDarkGreen = [42, 89, 0]/255.;

% extract points where safe control was applied.
safeCtrl_indicies = find(appliedUOpt == true);
xSafeCtrl = xtraj(safeCtrl_indicies);
ySafeCtrl = ytraj(safeCtrl_indicies);

%% Plot.
f = figure(1);
hold on

% visualize goal region.
c = [0.1,0.8,0.5,0.5];
pos = [params.xgoal(1)-params.goalEps, params.xgoal(2)-params.goalEps, params.goalEps*2, params.goalEps*2];
rectangle('Position',pos,'Curvature',1.0,'FaceColor',c,'LineStyle','none');
scatter(params.xgoal(1),params.xgoal(2),[],[0.0,0.8,0.5],'filled');

% Setup the figure axes to represent the entire environment
xlim([params.lowEnv(1) params.upEnv(1)]);
ylim([params.lowEnv(2) params.upEnv(2)]);

% Plot environment
[grid2D, ~] = proj(params.grid, params.grid.xs{1}, [0 0 1 1], [0 0]);
e = contourf(grid2D.xs{1}, grid2D.xs{2}, -occuMap, [0 0]);
colormap gray
 
h = plot(xtraj, ytraj, '-', 'Color', lightGreen, 'LineWidth', 3); 
% Plot points where safety was applied.
plot(xSafeCtrl, ySafeCtrl, 'ro', 'MarkerSize', 3,'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');

% Plot the car
plotCar([xtraj(1);ytraj(1);thetatraj(1)], 'k');
plotCar([xtraj(end);ytraj(end);thetatraj(end)], dLightGreen);

set(gca,'TickLabelInterpreter','latex')
set(gcf, 'Color', 'w');
% setup size of figure and tick mark stuff.
set(gca,'XTick',[params.lowEnv(1) 18 params.upEnv(1)], 'fontsize', 12);
set(gca,'YTick',[params.lowEnv(2) 33 params.upEnv(2)], 'fontsize', 12);
xlabel('$p_x$', 'Interpreter','latex', 'fontsize', 16);
ylabel('$p_y$', 'Interpreter','latex', 'fontsize', 16);

widthMeters = abs(params.lowEnv(1) - params.upEnv(1));
heightMeters = abs(params.lowEnv(2) - params.upEnv(2));
set(gcf, 'Position',  0.8*[100, 100, widthMeters*100*0.6, heightMeters*100*0.6])

% Save the figure
saveas(f, './hybrid_ws/src/safe_navigation/imgs/sbpd_crash.png')
saveas(f, './hybrid_ws/src/safe_navigation/imgs/sbpd_crash.fig')
saveas(f, './hybrid_ws/src/safe_navigation/imgs/sbpd_crash.pdf')

%% Plots dubins car point and heading.
% Inputs:
%   x [vector]  - 3D/4D state of dubins car
% Ouput:
%   c   - handle for figure
function c = plotCar(x, carColor)
    c = quiver(x(1), x(2), cos(x(3)), sin(x(3)), 'o', 'Color', carColor, ...
                'MarkerSize', 3, 'MarkerEdgeColor', carColor, ...
                'MarkerFaceColor', carColor, 'MaxHeadSize', 5.0, ...
                'ShowArrowHead', 'on', 'AutoScaleFactor', 0.4, ...
                'LineWidth', 0.9); 
%     c{1} = plot(x(1), x(2), 'o','MarkerSize', 5, ...
%         'MarkerEdgeColor', carColor, 'MarkerFaceColor', carColor);
% 
%     % Plot heading.
%     center = x(1:2);
% 
%     if length(x) >= 3
%         % Rotation matrix.
%         R = [cos(x(3)) -sin(x(3)); 
%              sin(x(3)) cos(x(3))];
%         % Heading pt.
%         hpt = [0.4; 0];
%         hptRot = R*hpt + center;
%         p2 = plot([center(1) hptRot(1)], [center(2) hptRot(2)], 'Color', carColor, 'LineWidth', 1.5);
%         c{2} = p2;
%     end
end