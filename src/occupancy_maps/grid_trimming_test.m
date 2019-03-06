clear all
close all;
%---------------------------------------------------------------------------
% Create the computation grid.
g.dim = 2;
g.min = [1; 3];
g.max = [7; 9];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate};
g.N = [31; 31];
g = processGrid(g);

% Create a random occupancy grid
map_bounds_4 = [0, 0, 10, 10];
map_dx = 0.05;
occupancy_map = zeros(200, 200);
occupancy_map(50:100, 100:150) = 1;

% Trim the occupancy grid
trimmed_occupancy_grid = generate_computation_grid(g, occupancy_map, map_dx, map_bounds_4);


% Visualize
figure,
RI = imref2d(size(occupancy_map));
RI.XWorldLimits = [0 10];
RI.YWorldLimits = [0 10];
imshow(occupancy_map, RI);

figure,
RI = imref2d(size(trimmed_occupancy_grid));
RI.XWorldLimits = [1 7];
RI.YWorldLimits = [3 9];
imshow(trimmed_occupancy_grid, RI);

