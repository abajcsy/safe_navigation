clear all
close all;
%---------------------------------------------------------------------------
% Create the computation grid.
N = 31;
g.dim = 2;
g.min = [14; 26];
g.max = [24; 36];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate};
g.N = [N; N];
g = processGrid(g);

% Load the SBPD grid
load('sbpdOccuMap.mat');
occupancy_map = sbpdOccuMap';

% Trim the occupancy grid
trimmed_occupancy_grid = generate_computation_grid(g, occupancy_map, 0.05, [0, 0, 26.2, 55.65]);
figure, imshow(flipud(trimmed_occupancy_grid'));
