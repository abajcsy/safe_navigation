clear all;
close all;

% Data.
localQ_rrt_camera_filename = '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/localQwarm_spline_camera_hand.mat';
load(localQ_rrt_camera_filename);

% Grab the parameters and data.
g = params.grid;
fontsize = 25;
lx = lxCellArr{2};
% lx = valueFunCellArr{2};

% Create the figure
f = figure;
set(f, 'position', [0, 0, 530, 300]);
set(gcf,'Color','w') 
hold on;

% Plot the initial state and heading
x = params.xinit;
l(1) = quiver(x(1), x(2), 0, 0.6, 'ko', 'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'MaxHeadSize', 30.0, 'ShowArrowHead', 'on');

% Belief obstacle
[grid2D, lx2D] = proj(g, lx, [0 0 1], 0);
[~, c] = contourf(grid2D.xs{1}, grid2D.xs{2}, -lx2D, [0, 0]);
colormap([0.5, 0.5, 0.5])

% Figure properties
ylim([params.lowEnv(2)+g.dx(2) params.upEnv(2)-g.dx(2)]);
xlim([params.lowEnv(1)+g.dx(1) params.upEnv(1)-g.dx(1)]);
set(gca, 'xtick', []);
set(gca, 'ytick', []);
xlabel('$p_x$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
ylabel('$p_y$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
box on;

% Save the figure
saveas(f, '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/figures_and_videos/safety_framework/belief_obs_1.png')
