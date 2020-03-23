clear all
%---------------------------------------------------------------------------
% Create grid
% How many grid cells?
Nx = 7;
% Create the computation grid.
g.dim = 2;
g.min = [  -3; -3];
g.max = [ +3; +3];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate};
% Roughly equal dx in x and y (so different N).
g.N = [ Nx; Nx];

g = processGrid(g);

% Target
Rt = 0.2;
%target = g.xs{1}.^2 + g.xs{2}.^2 - Rt^2;

% Convert target into a binary
%target = sign(target);
target = [-1, 1, 1, 1, 1, -1, -1;
            1, 1, 1, 1, 1, -1, -1;
            1, 1, 1, 1, 1, -1, -1;
            -1, -1, -1, -1, 1, 1, 1;
            -1, -1, -1, -1, 1, 1, 1;
            1, 1, 1, 1, 1, 1, 1;
            1, 1, 1, 1, 1, -1, -1];
% target = [ 1, 1, -1; 1, 1, -1; 1, -1, -1];
% Speed profile
speed = ones(g.N(1), g.N(2));

% Compute value
u = compute_value(g, target, speed);

obstacles = target == -1;
formatSpec = "%d, ";
fprintf(formatSpec, obstacles');

% Visualize
contour(g.xs{1},g.xs{2},u,0:0.1:6)
hold on
contour(g.xs{1},g.xs{2},target,[0 0],'color', [0 0.5 0],'linewidth',2)