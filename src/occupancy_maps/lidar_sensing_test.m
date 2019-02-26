clear all
close all;
%---------------------------------------------------------------------------
% Create grid
% How many grid cells?
Nx = 51;

% Create the computation grid.
g.dim = 2;
g.min = [  -1; -1];
g.max = [ +1; +1];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate};
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx];

g = processGrid(g);

% Obstacles (-ive inside and +ive outside)
Ro = 0.2;
obs1 = (g.xs{1}-0.5).^2 + g.xs{2}.^2 - Ro^2;
obs2 = (g.xs{1}+0.5).^2 + g.xs{2}.^2 - Ro^2;
obs = shapeUnion(obs1, obs2);

% Sensor details
sensing_radius = 0.8;

% Vehicle position
pos = [0.; 0.];

occupancy_grid = generate_lidar_sensing_region(g, obs, sensing_radius, pos);

% Visualize
contour(g.xs{1},g.xs{2},occupancy_grid,-2:0.1:-0.5)
hold on
scatter(pos(1), pos(2))
contour(g.xs{1},g.xs{2},obs,[0 0],'color', [0 0 0])