clear all
close all;
%---------------------------------------------------------------------------
% Create grid
% How many grid cells?
Nx = 11;

% Create the computation grid.
g.dim = 2;
g.min = [  -2.5; -2.5];
g.max = [ +2.5; +2.5];
g.bdry = { @addGhostExtrapolate; @addGhostExtrapolate};
% Roughly equal dx in x and y (so different N).


g.N = [ Nx; Nx];

g = processGrid(g);

% Target
Rt = 0.2;
target = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
          1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1;
          1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1;
          1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1;
          1, 1, -1, -1, -1, 1, 1, 1, 1, 1, 1;
          1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
          1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
          1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1;
          1, 1, 1, 1, 1, 1, 1, -1, -1, -1, 1;
          1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
          1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;];

% Speed profile
speed = ones(g.N(1), g.N(2));

% Compute value
u1 = compute_value(g, target, speed);
u2 = compute_value(g, -target, speed);
u = u1 .* (target >= 0) - u2 .* (target < 0);
obstacles = target == -1;
fprintf("Input:\n");
formatSpec = "%d, ";
fprintf(formatSpec, obstacles');
fprintf("\n");

formatSpec = "%.5f, ";
fprintf("Output:\n");
fprintf(formatSpec, u');
fprintf("\n");

% Visualize
%contour(g.xs{1},g.xs{2},u,-0.2:0.1:6)
%hold on
%contour(g.xs{1},g.xs{2},target,[0 0],'color', [0 0.5 0],'linewidth',2)