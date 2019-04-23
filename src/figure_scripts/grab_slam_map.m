function grab_slam_map()
    % Load up all the experimental parameters.
    params = car4DLocalQCameraSLAM();

    fprintf('Loading ground-truth SLAM occupancy map.\n');
    repo = what('safe_navigation');
    slamPath = strcat(repo.path, '/data/slamTrueOccuMap.mat');  
    load(slamPath);
    origin = origin - [2.,2.]; 

    % Extract the ground-truth occupancy map based on our 
    % compute grid.
    trueOccuMap = getTrueOccuMap(rawOccuMap, params.grid, origin, res, realWidth, realHeight);

    filename = 'resizedSLAMTrueOccuMap_023.mat';
    filePath = strcat(repo.path, '/data/', filename);  
    save(filePath, 'trueOccuMap');
end

%% Extracts ground-truth map relative to our compute grid.
function trueOccuMap = getTrueOccuMap(rawMap, grid, origin, res, realWidth, realHeight)
    % Note: only works for 4D system right now.
    if grid.dim == 4
        [grid2D, ~] = proj(grid, grid.xs{1}, [0 0 1 1], [0 0]);
    else
        error('I am not programmed to use safety module with %D system.\n', grid.dim);
    end
    mapBounds = [origin(1), origin(2), origin(1) + realWidth, origin(2) + realHeight];

    % Crop and interpolate the raw occupancy map from SBPD or SLAM
    % into the correct size for the computation grid.
    trueOccuMap = ...
        generate_computation_grid(grid2D, rawMap, ...
        res, mapBounds);
end