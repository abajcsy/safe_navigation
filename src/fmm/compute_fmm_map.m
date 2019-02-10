function u = compute_fmm_map(grid, occupancy_map)

% ----- How to use this function -----
%
% Inputs:
%   grid           - A 2D grid structure over which signed distance
%                     needs to be computed. 
%   occupancy_map  - A 2D occupancy map with (+1) for unoccupied 
%                     regions and (-1) for occupied.
% 
% Outputs:
%   u              - Signed distance over the grid.

% Make grid symmetric if required
if (grid.min(1) ~= -grid.max(1)) || (grid.min(2) ~= -grid.max(2))
  grid_center = 0.5*(grid.min + grid.max);
  grid.min = grid.min - grid_center;
  grid.max = grid.max - grid_center;
  grid.xs{1} = grid.xs{1} - grid_center(1);
  grid.xs{2} = grid.xs{2} - grid_center(2);
end

% Speed profile
speed = ones(grid.N(1), grid.N(2));

% Compute value
u1 = compute_value(grid, occupancy_map, speed);
u2 = compute_value(grid, -occupancy_map, speed);
u = u1 .* (occupancy_map >= 0) - u2 .* (occupancy_map < 0);
