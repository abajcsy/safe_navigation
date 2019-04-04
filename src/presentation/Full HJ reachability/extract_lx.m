% Clear old figure plotting and variables.
clf 
clc 
clear 
close all;

% Load the occupancy map
load('/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/slamTrueOccuMap.mat');

% The correct occupancy map
lowX = 2030;
highX = 2115;
lowY = 1975;
highY = 2060;
res = 0.05;
occupancy_grid_trimmed = rawOccuMap(lowX:highX, lowY:highY);

% Create a grid for the occupancy map
gridLow = [lowX*res;lowY*res];
gridUp = [highX*res;highY*res];
N = size(occupancy_grid_trimmed); 
grid = createGrid(gridLow, gridUp, N);

signed_distance_trimmed = compute_fmm_map(grid, occupancy_grid_trimmed); 
safety_threshold = 0.23;    % Tuned for SLAM
lx_2D = signed_distance_trimmed - safety_threshold;

save('/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/slam_lx_2D.mat', 'lx_2D', 'grid');

% occupancy_grid = sign(signed_distance_trimmed);
% figure,
% imshow(flipud(fliplr(occupancy_grid)));