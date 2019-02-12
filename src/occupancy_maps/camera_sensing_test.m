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
half_fov = pi/6;

% Vehicle position
pos = [0.; -0.5];
heading = 2*pi/3;
% pos = 2*rand(2) - 1;

% Initialize the occupancy grid to be occupied everywhere
occupancy_grid = -ones(size(g.xs{1}));

% Compute the angle of each points on the grid to the vehicle position
angle_to_source = atan2(g.xs{2} - pos(2), g.xs{1} - pos(1));

% Find the points that are inside the field of view and set them to free
indicies_inside_fov = find(abs(angle_to_source - heading) < half_fov);
occupancy_grid(indicies_inside_fov) = 1;

% Compute the indicies inside the obstacle and set them to occupied
obs_indicies = find(obs <= 0);
occupancy_grid(obs_indicies) = -1;

% Now compute the indicies of all the points for which we have to do ray
% tracing
obs_indicies_of_interest = intersect(indicies_inside_fov, obs_indicies);
if ~isempty(obs_indicies_of_interest)
  angle_for_obs = angle_to_source(obs_indicies_of_interest) - heading;
  max_stretch = min(max(angle_for_obs), half_fov);
  min_stretch = max(min(angle_for_obs), -half_fov);
  indicies_to_trace = find(((angle_to_source - heading) <= max_stretch) .* ...
    ((angle_to_source - heading) >= min_stretch));

  % Now let's do the ray tracing
  num_pts = size(indicies_to_trace);
  occupancy_status = ones(num_pts);
  pos_x = g.xs{1}(indicies_to_trace);
  pos_y = g.xs{2}(indicies_to_trace);
  flattened_grid_x = repmat(pos_x, [1, num_pts]);
  flattened_grid_y = repmat(pos_y, [1, num_pts]);

  num_alphas = 10;
  alphas = linspace(0, 1, num_alphas);

  for i=1:num_alphas
    alpha = alphas(i);

    % Find the points to check
    points_x = (1-alpha)*pos(1) + alpha*pos_x;
    points_y = (1-alpha)*pos(2) + alpha*pos_y;

    % Find the array indices corresponding to these points
    points_x = repmat(points_x', [num_pts, 1]);
    points_y = repmat(points_y', [num_pts, 1]);
    error = sqrt((flattened_grid_x - points_x).^2 + (flattened_grid_y - points_y).^2);
    [~, array_indicies_to_check] = min(error, [], 1);
    obs_status = sign(obs(indicies_to_trace(array_indicies_to_check)));
    occupancy_status = 0.5*((obs_status + occupancy_status) - abs(obs_status - occupancy_status));
  end
  occupancy_grid(indicies_to_trace) = occupancy_status;
end
  
% Visualize
contour(g.xs{1},g.xs{2},occupancy_grid,-2:0.1:-0.5)
hold on
scatter(pos(1), pos(2))
contour(g.xs{1},g.xs{2},obs,[0 0],'color', [0 0 0])