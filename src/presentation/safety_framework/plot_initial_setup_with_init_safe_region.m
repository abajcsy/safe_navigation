clear all;
close all;

% Data.
localQ_rrt_camera_filename = '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/localQwarm_spline_camera_hand.mat';
load(localQ_rrt_camera_filename);

% Grab the parameters and data.
g = params.grid;
fontsize = 25;
lx = lxCellArr{2};

%% Create the initial setup
f = figure;
set(f, 'position', [0, 0, 530, 300]);
set(gcf,'Color','w') 
hold on;

% Plot the initial state and heading
x = params.xinit;
l(1) = quiver(x(1), x(2), 0, 0.6, 'ko', 'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'MaxHeadSize', 30.0, 'ShowArrowHead', 'on');

% Plot the camera sensing
fillC = [0.5,0.5,1.0];
[grid2D, lx2D] = proj(g, lx, [0 0 1], 0);
[c, h] = contourf(grid2D.xs{1}, grid2D.xs{2}, -lx2D, [0, 0]);
x = c(1,2:end);
y = c(2,2:end);
delete(h);
h = fill(x,y,fillC,'FaceAlpha',0.3, 'EdgeColor', 'none');

% Plot the goal region
c = [0.0,0.8,0.5,0.5];
xgoal = params.xgoal;
pos = [xgoal(1)-params.goalEps, xgoal(2)-params.goalEps, params.goalEps*2, params.goalEps*2];
rectangle('Position',pos,'Curvature',1.0,'FaceColor',c,'LineStyle','none');
l(2) = scatter(xgoal(1),xgoal(2),[],[0.0,0.8,0.5],'filled');

% Plot the obstacle
obstacles = params.obstacles;
lowObs = obstacles{1}{1};
upObs = obstacles{1}{2};
width = upObs(1) - lowObs(1);
height = upObs(2) - lowObs(2);
obsCoord = [lowObs(1), lowObs(2), width, height];
e = rectangle('Position', obsCoord, ...
    'FaceColor', [0.5,0.5,0.5], 'EdgeColor', [0.2,0.2,0.2]);
l(3) = plot(0, 0, 'Color', [0.5,0.5,0.5], 'LineStyle','-');

% Figure properties
ylim([params.lowEnv(2)+g.dx(2) params.upEnv(2)-g.dx(2)]);
xlim([params.lowEnv(1)+g.dx(1) params.upEnv(1)-g.dx(1)]);
set(gca, 'xtick', []);
set(gca, 'ytick', []);
xlabel('$p_x$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
ylabel('$p_y$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
box on;

% Save the figure
saveas(f, '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/figures_and_videos/safety_framework/initial_setup_with_safe_region.png')
