function occupancy_grid = generate_lidar_sensing_region(grid, obs, sensing_radius, vehicle_pos)

% ----- How to use this function -----
%
% Inputs:
%   grid           - A 2D grid structure over which the occupancy map 
%                     needs to be computed. 
%   obs            - A 2D occupancy map with (+1) for unoccupied 
%                     regions and (-1) for occupied.
%   sensing_radius - The sensing radius of the lidar.
%   vehicle_pos    - The vehicle vehicle_position vector.
% 
% Outputs:
%   u              - Signed distance over the grid.

% Initialize the occupancy grid to be occupied everywhere
occupancy_grid = -ones(size(grid.xs{1}));

% Compute the angle of each points on the grid to the vehicle vehicle_position
angle_to_source = atan2(grid.xs{2} - vehicle_pos(2), grid.xs{1} - vehicle_pos(1));

% Compute the distance of each point from the vehicle
dist_to_vehicle = (grid.xs{1}-vehicle_pos(1)).^2 + ...
  (grid.xs{2} - vehicle_pos(2)).^2 - sensing_radius^2;

% Find the points that are inside the field of view and set them to free
indicies_inside_fov = find(dist_to_vehicle < 0);
occupancy_grid(indicies_inside_fov) = 1;

% Compute the indicies inside the obstacle and set them to occupied
obs_indicies = find(obs <= 0);
occupancy_grid(obs_indicies) = -1;

% Now compute the indicies of all the points for which we have to do ray
% tracing
obs_indicies_of_interest = intersect(indicies_inside_fov, obs_indicies);

% Line scales for ray tracing
num_alphas = 10;
alphas = linspace(0, 1, num_alphas);

if ~isempty(obs_indicies_of_interest)
  angle_for_obs = angle_to_source(obs_indicies_of_interest);
  max_stretch = max(angle_for_obs);
  min_stretch = min(angle_for_obs);
  indicies_to_trace = find((angle_to_source <= max_stretch) .* (angle_to_source >= min_stretch) .* (dist_to_vehicle < 0));

  % Now let's do the ray tracing
  num_pts = size(indicies_to_trace);
  occupancy_status = ones(num_pts);
  vehicle_pos_x = grid.xs{1}(indicies_to_trace);
  vehicle_pos_y = grid.xs{2}(indicies_to_trace);
  flattened_grid_x = repmat(vehicle_pos_x, [1, num_pts]);
  flattened_grid_y = repmat(vehicle_pos_y, [1, num_pts]);

  for i=1:num_alphas
    alpha = alphas(i);

    % Find the points to check
    points_x = (1-alpha)*vehicle_pos(1) + alpha*vehicle_pos_x;
    points_y = (1-alpha)*vehicle_pos(2) + alpha*vehicle_pos_y;

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

end