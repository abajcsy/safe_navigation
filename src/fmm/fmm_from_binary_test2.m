clear all
close all;
%---------------------------------------------------------------------------
% Create grid
% How many grid cells?
Nx = 200;

% Create the computation grid.
g.dim = 2;
g.min = [  -1; -1];
g.max = [ +1; +1];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate};
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx];

g = processGrid(g);

% Target
Rt = 0.2;
target = g.xs{1}.^2 + g.xs{2}.^2 - Rt^2;

% Make target positive in the first and the third quadrant
target = -target .* (2*(g.xs{1} > 0.01) - 1) .* (2*(g.xs{2} > 0.01) - 1) .* (target < 0) ;

% Convert target into a binary
target = sign(target);

% Speed profile
speed = ones(g.N(1), g.N(2));

% Compute value
u1 = compute_value(g, target, speed);
u2 = compute_value(g, -target, speed);
u = u1 .* (target >= 0) - u2 .* (target < 0);

% Visualize
contour(g.xs{1},g.xs{2},u,-0.2:0.1:6)
hold on
contour(g.xs{1},g.xs{2},target,[0 0],'color', [0 0.5 0],'linewidth',2)