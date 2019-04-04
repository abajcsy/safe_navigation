% Clear old figure plotting and variables.
clf 
clc 
clear 
close all;

% Load the occupancy map
load('/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/brs_Vx_4D.mat');
im = imread('/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/experimental_setup.png');

% Initial and final sets
data0 = data(:, :, :, :, 1);
brs = data(:, :, :, :, end);
fontsize = 68;
ticksize = 20;

% %% Plot BRS
% f = figure(1);
% hold on
% % % Initial set
% % [grid2D, data2D] = proj(g, data0, [0 0 1 1], [pi/2 0]);
% % contour(grid2D.xs{1}, grid2D.xs{2}, data2D, [0, 0]);
% % Final set
% [grid2D, data2DBRS] = proj(g, brs, [0 0 1 1], [pi/2 0.5]);
% contour(grid2D.xs{1}, grid2D.xs{2}, data2DBRS, [0, 0], 'r', 'linewidth', 5);
% axis off;

% %% Plot the value function video strating from 2D
% figure, 
% hold on
% [grid2D, data2DBRS] = proj(g, brs, [0 0 1 1], [pi/2 0.5]);
% [grid2D, data2D] = proj(g, data0, [0 0 1 1], [pi/2 0.5]);
% [~, c] = contourf(grid2D.xs{1}, grid2D.xs{2}, -data2D, [0, 0]);
% colormap([0.5, 0.5, 0.5])
% contour(grid2D.xs{1}, grid2D.xs{2}, data2DBRS, [0, 0], 'r', 'linewidth', 5);
% h = visFuncIm(grid2D, data2DBRS, 'r', 0.5);
% c = camlight;
% % axis off
% 
% videoFilename = 'val_func.mp4';
% vout = VideoWriter(videoFilename,'MPEG-4');
% vout.Quality = 100;
% vout.FrameRate = 40;
% vout.open;
% 
% azrange = linspace(0,80,100);
% elrange = linspace(90,30,100);
% for ii = 1:length(azrange)
%    az = azrange(ii);
%    el = elrange(ii);
%    view(az,el)
%    current_frame = getframe(gcf); %gca does just the plot
%    writeVideo(vout,current_frame);
% end
% vout.close;

% Plot the lx function video strating from 2D
f = figure('Position', [0, 0, 1920, 1080]), 
set(gcf,'Color','w') 
hold on
[grid2D, data2D] = proj(g, data0, [0 0 1 1], [pi/2 0.5]);
[~, c] = contourf(grid2D.xs{1}, grid2D.xs{2}, -data2D, [0, 0]);
colormap([0.5, 0.5, 0.5])
c = camlight;
c.Position = [119.5625 119.2780 15.3565];
set(gca, 'xtick', []);
set(gca, 'ytick', []);
set(gca, 'ztick', [-0.5, 0, 0.5]);
set(gca, 'fontsize', ticksize);
xlabel('$p_x$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
ylabel('$p_y$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
zlabel('$V(0, x)$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
% axis off

videoFilename = '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/figures_and_videos/full_HJI/lx_func.mp4';
vout = VideoWriter(videoFilename,'MPEG-4');
vout.Quality = 100;
vout.FrameRate = 40;
vout.open;

current_frame = getframe(gcf); %gca does just the plot
writeVideo(vout,current_frame);
saveas(f, '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/figures_and_videos/full_HJI/lx_func.png');

h = visFuncIm(grid2D, data2D, [0.5, 0.5, 0.5], 0.5);
c = camlight;
c.Position = [119.5625 119.2780 15.3565];

azrange = linspace(0,-80,100);
elrange = linspace(90,50,100);
for ii = 1:length(azrange)
   az = azrange(ii);
   el = elrange(ii);
   view(az,el)
   current_frame = getframe(gcf); %gca does just the plot
   writeVideo(vout,current_frame);
end
vout.close;


%% Show the value function propagation
f = figure('Position', [0, 0, 1920, 1080]), 
set(gcf,'Color','w') 
hold on
[grid2D, data2D] = proj(g, data0, [0 0 1 1], [pi/2 0.5]);
[~, c] = contourf(grid2D.xs{1}, grid2D.xs{2}, -data2D, [0, 0]);
colormap([0.5, 0.5, 0.5])

videoFilename = '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/figures_and_videos/full_HJI/Vx_func.mp4';
vout = VideoWriter(videoFilename,'MPEG-4');
vout.Quality = 100;
vout.FrameRate = 5;
vout.open;

data_size = size(data);
colorR = linspace(0.5, 1, data_size(5));
colorG = linspace(0.5, 0, data_size(5));
colorB = linspace(0.5, 0, data_size(5));

for ii = 1:data_size(5)
  [grid2D, data2DBRS] = proj(g, data(:, :, :, :, ii), [0 0 1 1], [pi/2 0.5]);
  h = visFuncIm(grid2D, data2DBRS, [colorR(ii), colorG(ii), colorB(ii)], 0.5);
  [~, s] = contour(grid2D.xs{1}, grid2D.xs{2}, data2DBRS, [0, 0], 'linecolor', [colorR(ii), colorG(ii), colorB(ii)], 'linewidth', 3);
  c = camlight;
  c.Position = [119.5625 119.2780 15.3565];
  set(gca, 'xtick', []);
  set(gca, 'ytick', []);
  set(gca, 'ztick', [-0.5, 0, 0.5]);
  set(gca, 'zlim', [-1, 1]);
  set(gca, 'fontsize', ticksize);
  xlabel('$p_x$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
  ylabel('$p_y$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
  zlabel(strcat('$V(', num2str(0.02*(ii-1), '%4.2f'), ', x)$'), 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
  view(-80, 50)

  current_frame = getframe(gcf); %gca does just the plot
  writeVideo(vout,current_frame);
  delete(h);
  delete(c);
  delete(s);
end
vout.close;


%% Plot the value function video strating from 2D
f = figure('Position', [0, 0, 1920, 1080]), 
set(gcf,'Color','w') 
hold on
[grid2D, data2DBRS] = proj(g, brs, [0 0 1 1], [pi/2 0.5]);
[grid2D, data2D] = proj(g, data0, [0 0 1 1], [pi/2 0.5]);
[~, c] = contourf(grid2D.xs{1}, grid2D.xs{2}, -data2D, [0, 0]);
colormap([0.5, 0.5, 0.5])
contour(grid2D.xs{1}, grid2D.xs{2}, data2DBRS, [0, 0], 'r', 'linewidth', 3);
h = visFuncIm(grid2D, data2DBRS, 'r', 0.5);
data_size = size(data);
c = camlight;
c.Position = [119.5625 119.2780 15.3565];
set(gca, 'xtick', []);
set(gca, 'ytick', []);
set(gca, 'ztick', [-0.5, 0, 0.5]);
set(gca, 'zlim', [-1, 1]);
set(gca, 'fontsize', ticksize);
xlabel('$p_x$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
ylabel('$p_y$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
zlabel(strcat('$V(', num2str(0.02*(data_size(5)-1)), ', x)$'), 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
% axis off

videoFilename = '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/figures_and_videos/full_HJI/Vx_2D.mp4';
vout = VideoWriter(videoFilename,'MPEG-4');
vout.Quality = 100;
vout.FrameRate = 40;
vout.open;

azrange = linspace(-80,0,100);
elrange = linspace(50,90,100);
for ii = 1:length(azrange)
   az = azrange(ii);
   el = elrange(ii);
   view(az,el)
   current_frame = getframe(gcf); %gca does just the plot
   writeVideo(vout,current_frame);
end
delete(h);
current_frame = getframe(gcf); %gca does just the plot
writeVideo(vout,current_frame);
saveas(f, '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/figures_and_videos/full_HJI/Vx_func.png');
vout.close;