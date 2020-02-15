clear all;
close all;
clc;

% Data.
localQ_spline_camera_filename = '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/localQwarm_spline_camera_hand.mat';
localQ_spline_camera_traj_filename = '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/localQwarm_spline_camera_hand_traj.mat';
load(localQ_spline_camera_filename);
load(localQ_spline_camera_traj_filename);

% Grab the parameters and data.
g = params.grid;
fontsize = 25;
states = cell2mat(states);

% Create the figure
f = figure;
set(f, 'position', [0, 0, 530, 300]);
set(gcf,'Color','w') 
hold on;

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

videoFilename = '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/figures_and_videos/safety_framework/overall_progression.mp4';
vout = VideoWriter(videoFilename,'MPEG-4');
vout.Quality = 100;
vout.FrameRate = 10;
vout.open;

for i=1:104  
  % Belief obstacle
  lx = lxCellArr{i};
  [grid2D, lx2D] = proj(g, lx, [0 0 1], 0);
  [~, c1] = contourf(grid2D.xs{1}, grid2D.xs{2}, -lx2D, [0, 0]);
  colormap([0.5, 0.5, 0.5])
  
  % BRS
  Vx = valueFunCellArr{i};
  [grid2D, Vx2D] = proj(g, Vx, [0 0 1], states(3, 1+(i-1)*10));
  [~, c2] = contour(grid2D.xs{1}, grid2D.xs{2}, -Vx2D, [0, 0], 'r', 'linewidth', 3);
  
  % Plot the state trajectory
  for j=1+(i-1)*10:1+i*10
    if appliedUOpt(j) == 1
      color= 'r';
      marker_color = 'r';
    else
      color = [170/255, 170/255, 170/255];
      marker_color = 'k';
    end
    h = plot(states(1, j), states(2, j), 'o', 'MarkerSize', 5, 'MarkerEdgeColor', color, 'MarkerFaceColor', color);
  end
  
  % Plot the state and heading
  x = states(:, 1+i*10);
  l = quiver(x(1), x(2), 0.6*cos(x(3)), 0.6*sin(x(3)), 'ko', 'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'MaxHeadSize', 30.0, 'ShowArrowHead', 'on');

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
  delete(c1);
  delete(c2);
  delete(l);
end
vout.close;