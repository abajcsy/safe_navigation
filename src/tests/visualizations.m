%% Visualizes stuff in a pretty way.
clf

%% Choose which method you wanna visualize.
%   HJI, localQ
method = 'HJI';
warm = false;
planner = 'rrt';
sensor = 'lidar';
envType = 'hand';

%% Grab the data from both local updates.
path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data/';

% filename = method;
% if warm
%     filename = strcat(filename, 'warm');
% end
% filename = strcat(filename, '_', planner, '_', sensor, '_', envType, '.mat');
% filePath = strcat(path, filename);
% load(filePath);

% if strcmp(method, 'HJI')
%     if warm
%         title('Warm Start HJI-VI', 'Interpreter','latex', 'fontsize',16);
%         color = [0.1,0.1,1.0];
%         params = dubinsWarmCameraExp1();
%     else
%         title('HJI-VI', 'Interpreter','latex', 'fontsize',16);
%         filename = strcat('HJI', sensor, '_dawkins.mat');
%         color = [0,0,0];
%         params = dubinsHJICameraExp1();
%     end
% elseif strcmp(method, 'localQ')
%     title('Local Update Algorithm', 'Interpreter','latex', 'fontsize',16);
%     color = [1.0,0.1,0.1];
%     params = dubinsLocalQCameraExp1();
% else
%     error('Unsupported method type.');
% end

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

% Choose which file we want to visualize.
filename = HJI_spline_camera_filename;
% @car3DLocalQCameraRRT, ...
% @car3DWarmCameraRRT, ...
% @car3DHJICameraRRT, ...
% @car3DLocalQLidarRRT, ...
% @car3DWarmLidarRRT, ...
% @car3DHJILidarRRT};
% {@car3DLocalQLidarSpline, ...
% @car3DWarmLidarSpline, ...
% @car3DHJILidarSpline, ...
% @car3DLocalQCameraSpline, ...
% @car3DWarmCameraSpline, ...
% @car3DHJICameraSpline};
params = car3DHJICameraSpline();

filePath = strcat(path, filename);
load(filePath);
grid = params.grid;
valueFuns = valueFunCellArr;

%% Visualize
offsetX = (grid.max(1) - grid.min(1))/grid.N(1);
offsetY = (grid.max(2) - grid.min(2))/grid.N(2);

boundLow = [grid.min(1)+offsetX, grid.min(2)+offsetY, grid.min(3)];
boundUp = [grid.max(1)-offsetX, grid.max(2)-offsetY, grid.max(3)];

plt = Plotter(params.lowEnv, params.upEnv, boundLow, boundUp, params.envType, params.obstacles, params.goalEps);

hold on;
idx = 1;
for i=1:length(states)
    currUpdateT = updateTimeArr(idx);
    if i == currUpdateT
        % plot value function
        currFun = valueFuns{idx};
        idx = idx + 1;
        if idx > length(updateTimeArr)
            idx = length(updateTimeArr);
        end
    end
    
    % grab important variables.
    x = states{i};
    path = paths{i};
    occuMap = safeOccuMaps{i};
    [g2D, ~] = proj(grid, currFun, [0 0 1], x(3));
    
    % see if we applied optimal control
    usedUOpt = isOnBoundary(x,grid,currFun,params.safetyTol);
    
    % update plot.
    plt.updatePlot(x, params.xgoal, currFun, ...
                grid, g2D, occuMap, path, usedUOpt);
    
    pause(params.dt);
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

