clear all;
close all;
clc;

% Data.
localQ_rrt_camera_filename = '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/localQwarm_spline_camera_hand.mat';
load(localQ_rrt_camera_filename);

% Grab the parameters and data.
g = params.grid;
fontsize = 25;
states = cell2mat(states);

% Create the figure
f = figure;
set(f, 'position', [0, 0, 530, 300]);
set(gcf,'Color','w') 
hold on;

videoFilename = '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/figures_and_videos/safety_framework/sensing_region_progression.mp4';
vout = VideoWriter(videoFilename,'MPEG-4');
vout.Quality = 100;
vout.FrameRate = 30;
vout.open;

num_steps = 150;
start_step = 2;
fillC = [0.5,0.5,1.0];

% Belief obstacle
lx = lxCellArr{start_step};
[grid2D, lx2D] = proj(g, lx, [0 0 1], 0);
[c, h] = contourf(grid2D.xs{1}, grid2D.xs{2}, -lx2D, [0, 0]);
x = c(1,2:end);
y = c(2,2:end);
delete(h);
h1 = fill(x,y,fillC,'FaceAlpha',0.1, 'EdgeColor', 'none');

% Figure properties
ylim([params.lowEnv(2)+g.dx(2) params.upEnv(2)-g.dx(2)]);
xlim([params.lowEnv(1)+g.dx(1) params.upEnv(1)-g.dx(1)]);
set(gca, 'xtick', []);
set(gca, 'ytick', []);
xlabel('$p_x$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
ylabel('$p_y$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
box on;

current_frame = getframe(gcf); %gca does just the plot
writeVideo(vout,current_frame);

for i=start_step:start_step+num_steps  
  % Belief occupancy map
  occu2D = safeOccuMaps{i};
  [c, h] = contourf(grid2D.xs{1}, grid2D.xs{2}, -occu2D, [0, 0]);
  x = c(1,2:end);
  y = c(2,2:end);
  delete(h);
  h2 = fill(x,y,fillC,'FaceAlpha',0.1, 'EdgeColor', 'none');
  
  % Plot the quiver
  x = states(:, i);
  l = quiver(x(1), x(2), 0.6*cos(x(3)), 0.6*sin(x(3)), 'ko', 'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'MaxHeadSize', 30.0, 'ShowArrowHead', 'on');
  
  % Plot the state trajectory
  h = plot(states(1, start_step:i), states(2, start_step:i), 'linewidth', 3, 'color', [0.3, 0.3, 0.3]);
  
  % Figure properties
  ylim([params.lowEnv(2)+g.dx(2) params.upEnv(2)-g.dx(2)]);
  xlim([params.lowEnv(1)+g.dx(1) params.upEnv(1)-g.dx(1)]);
  set(gca, 'xtick', []);
  set(gca, 'ytick', []);
  xlabel('$p_x$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
  ylabel('$p_y$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
  box on;
  
  current_frame = getframe(gcf); %gca does just the plot
  writeVideo(vout,current_frame);
  delete(h);
%   delete(l)
  
%   delete(c1);
%   delete(c2);
end
vout.close;
