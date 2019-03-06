function occupancy_grid_trimmed = generate_computation_grid(grid, occupancy_func_mn, occupancy_map_dx, occupancy_map_bounds_4)

% ----- How to use this function -----
%
% Inputs:
%   grid           - A 2D grid structure over which the occupancy map 
%                     function needs to be computed. 
%   occupancy_func_mn   - A mxn occupancy map with (+1) for unoccupied 
%                     regions and (-1) for occupied, which needs to be
%                     trimmed.
%   occupancy_map_dx    - The discretization step for the occupancy map.
%   occupancy_map_bounds_4   - The [xMin, yMin, xMax, yMax] bounds of the
%                               occpancy grid.
% 
% Outputs:
%  trimmed_occcupancy_map    - Trimmed occupancy function defined over the
%                                grid.


% Crop the grid
dx = occupancy_map_dx;
min_indices = floor((grid.min' - occupancy_map_bounds_4(1:2))/dx)+1;
max_indices = ceil((grid.max' - occupancy_map_bounds_4(1:2))/dx)+1;
occupancy_grid_trimmed = occupancy_func_mn(min_indices(1):max_indices(1), ...
                                           min_indices(2):max_indices(2));
                                         
% Create the computation grid of the finer resolution occupancy map
grid_size = size(occupancy_grid_trimmed); 
g.dim = grid.dim;
g.min = grid.min;
g.max = grid.max;
g.bdry = grid.bdry;
g.N = grid_size';
g = processGrid(g);

% Change the grid resolution
occupancy_grid_trimmed = interp2(g.xs{1}', g.xs{2}', occupancy_grid_trimmed', ...
                                 grid.xs{1}, grid.xs{2});
occupancy_grid_trimmed = sign(occupancy_grid_trimmed);
end