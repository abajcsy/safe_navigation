%% Saves out trajectories.
clear all

% -------- RRT --------- %
HJI_rrt_lidar_filename = 'HJI_rrt_lidar_hand.mat';
HJIwarm_rrt_lidar_filename = 'HJIwarm_rrt_lidar_hand.mat';
localQ_rrt_lidar_filename = 'localQwarm_rrt_lidar_hand.mat';

HJI_rrt_camera_filename = 'HJI_rrt_camera_hand.mat';
HJIwarm_rrt_camera_filename = 'HJIwarm_rrt_camera_hand.mat';
localQ_rrt_camera_filename = 'localQwarm_rrt_camera_hand.mat';

% -------- Spline -------- %
HJI_spline_lidar_filename = 'HJI_spline_lidar_hand.mat';
HJIwarm_spline_lidar_filename = 'HJIwarm_spline_lidar_hand.mat';
localQ_spline_lidar_filename = 'localQwarm_spline_lidar_hand.mat';

HJI_spline_camera_filename = 'HJI_spline_camera_hand.mat';
HJIwarm_spline_camera_filename = 'HJIwarm_spline_camera_hand.mat';
localQ_spline_camera_filename = 'localQwarm_spline_camera_hand.mat';

% Path info.
path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data/';
traj_path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data_traj/';

% ------- Neural Net -------- %
localQ_nn_safe_camera_filename = 'localQwarm_nn_camera_sbpd_safetytrue.mat';
localQ_nn_unsafe_camera_filename = 'localQwarm_nn_camera_sbpd_safetyfalse.mat';

% Specify the file.
filename = localQ_nn_safe_camera_filename; 
%params = car3DWarmCameraRRT();       % NOTE: have to change this too!
params = car4DLocalQCameraNN();

% Load the file.
filePath = strcat(path, filename);
load(filePath);
grid = params.grid;
if ~strcmp(filename, localQ_nn_unsafe_camera_filename)
    valueFuns = valueFunCellArr;
end

% Store everything
xtraj = [];
ytraj = [];
thetatraj = [];
veltraj = [];
appliedUOpt = [];

idx = 1;
currUpdateT = 1;
ctrlHoriz = 20;
for i=1:length(states)
    if ~strcmp(filename, localQ_nn_unsafe_camera_filename)
        % determine if we need to grab the safe set.
        if ~strcmp(filename, localQ_nn_safe_camera_filename)
            currUpdateT = updateTimeArr(idx);
        end
        if i == currUpdateT
            % plot value function
            currFun = valueFuns{idx};
            idx = idx + 1;
            currUpdateT = currUpdateT + ctrlHoriz;
            if idx > length(updateTimeArr)
                idx = length(updateTimeArr);
            end
        end
    end
    
    % grab important variables.
    x = states{i};
    
    if ~strcmp(filename, localQ_nn_unsafe_camera_filename)
        % see if we applied optimal control
        usedUOpt = isOnBoundary(x,grid,currFun,params.safetyTol);
    else
        usedUOpt = false;
    end
    
    % Store everything.
    xtraj(end+1) = x(1);
    ytraj(end+1) = x(2);
    thetatraj(end+1) = x(3);
    % NN has 4D states.
    if strcmp(filename, localQ_nn_unsafe_camera_filename) || ...
        strcmp(filename, localQ_nn_safe_camera_filename)
        veltraj(end+1) = x(4);
    end
    appliedUOpt(end+1) = usedUOpt; 
end

% save out.
strings = strsplit(filename, '.');
new_filename = strcat(traj_path, strings{1}, '_traj.mat');
if strcmp(filename, localQ_nn_unsafe_camera_filename) || ...
        strcmp(filename, localQ_nn_safe_camera_filename)
    save(new_filename, 'xtraj', 'ytraj', 'thetatraj','veltraj', 'appliedUOpt');
else
    save(new_filename, 'xtraj', 'ytraj', 'thetatraj', 'appliedUOpt');
end

function onBoundary = isOnBoundary(x,grid,valueFun,tol)
    % Grab the value at state x from the most recent converged 
    % value function.
    value = eval_u(grid, valueFun, x);

    % If the value is close to zero, we are close to the safety
    % boundary.
    if value < tol 
        onBoundary = true;
    else
        onBoundary = false;
    end
end