function simulation_2()
%% Should we compute the trajectory?
compTraj = false;

%% Grid

% everything is x,y,theta or x,y unless specified (only specified is view
% and shape data)
n = 81;
grid_min = [-5; -5; -3]; % Lower corner of computation domain
grid_max = [5; 5; 3];    % Upper corner of computation domain
N = [n; n; 7];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions


%% time vector
t0 = 0;
tMax = 2.0;
dt = 0.05;
tau = t0:dt:tMax;
%% problem parameters
% input bounds
speed = 2;
wMax =  [-1, 1];
x_positions = {[-3., 1.1], [-2.5, 2.], [-1.3, 2.8], [0., 3.], [1.5, 3.], [3., 3.]};
xinit = [x_positions{1}, 0];
dMax = [0.0, 0.0, 0.0];
% Define dynamic system
% obj = DubinsCar(x, wMax, speed, dMax)
dCar = DubinsCar(xinit, wMax, speed, dMax); %do dStep3 here

%% Control Parameters
% control trying to min or max value function?
uMode = 'max';
dMode = 'min';
% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'veryHigh'; %set accuracy
schemeData.uMode = uMode;
schemeData.dMode = dMode;

%% target set
obst = {[-2, 0, 3, 2]};
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
% low_x, low_y, up_x, up_y
% obst = {[-3, -3, 3, -2.6]};
% shape: 
% y goes down to up. -3 to 3. 
% x goes left to right. -3 to 3.
%     -1    -1    -1    -1    -1    -1    -1
%      1     1     1     1     1     1     1
%      1     1     1     1     1     1     1
%      1     1     1     1     1     1     1
%      1     1     1     1     1     1     1
%      1     1     1     1     1     1     1
%      1     1     1     1     1     1     1
% data0 = shapeRectangleByCorners(grid, )

% shape is y then x.
target = zeros(g.shape);
shape = ones(n, n);
view = -ones(n, n); % -1 obstacle
grid_disc = (grid_max - grid_min) ./ (N - 1);
for i = 1:length(obst)
    [low_x, low_y] = IndicesToCoords(obst{i}(1), obst{i}(2), grid_min(1), grid_min(2), grid_disc(1), grid_disc(2));
    [up_x, up_y] = IndicesToCoords(obst{i}(3), obst{i}(4), grid_min(1), grid_min(2), grid_disc(1), grid_disc(2));
    for y = low_y:up_y
        for x = low_x:up_x
            shape(y, x) = -1;
        end
    end
end 
[gFMM, ~] = proj(g, target, [0, 0, 1], 0);  
% unionL_2D_FMM = compute_fmm_map(gFMM, shape); 
% signed_dist = repmat(unionL_2D_FMM, 1, 1, g.shape(3));

xpos = x_positions{1};
viewRadius = 2.5;
view = UpdateViewMap(xpos, view, viewRadius, N, shape, grid_min, grid_disc);
unionL_2D_FMM = compute_fmm_map(gFMM, view); 
signed_dist = repmat(unionL_2D_FMM, 1, 1, g.shape(3));

%% Compute value function
data0 = signed_dist;
HJIextraArgs.targets = data0;
HJIextraArgs.visualize = true; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update
HJIextraArgs.stopConverge = true;
HJIextraArgs.convergeThreshold = 1e-4;

% Warm start solve
[data, tau2, ~] = ...
  HJIPDE_solve_warm(data0, [], data0, tau, schemeData, 'minVWithL', true, HJIextraArgs);
valueFun = data; tau = tau2; 

% Create plotting info
exampleName = 'test_grid';
paths = sprintf('../beacls_ros_features/gen_sample_traj/%s/', exampleName);
mkdir(strcat(paths, 'view/'));
mkdir(strcat(paths, 'unionL_2D_FMM/'));
mkdir(strcat(paths, 'HJIPDE/zero_level_set/'));
mkdir(strcat(paths, 'HJIPDE/positive_set/'));
mkdir(strcat(paths, 'warm_start/zero_level_set/'));
mkdir(strcat(paths, 'warm_start/positive_set/'));
mkdir(strcat(paths, 'values/'));

% Plot time step 1
time_step = 1;
plot_and_save_env(x_positions, shape, paths, g);
plot_and_save_value_funs(valueFun, view, unionL_2D_FMM, signed_dist, paths, time_step, viewRadius, true);
plot_and_save_value_funs(valueFun, view, unionL_2D_FMM, signed_dist, paths, time_step, viewRadius, false);
lxOld = signed_dist; 
for i = 2:length(x_positions)
    % update robot position
    xpos = x_positions{i};
    view = UpdateViewMap(xpos, view, viewRadius, N, shape, grid_min, grid_disc);
    unionL_2D_FMM = compute_fmm_map(gFMM, view); 
    signed_dist = repmat(unionL_2D_FMM, 1, 1, g.shape(3));
    
    % warm start and HJIPDE solve
    lx = signed_dist;
    data0 = data(:, :, :, end);
    HJIextraArgs.targets = lx;
    [data, tau2, ~] = HJIPDE_solve_warm(data0, lxOld, lx, tau, schemeData, 'minVWithL', true, HJIextraArgs);
    [hjipde_data, tau3, ~] =   HJIPDE_solve(lx, tau, schemeData, 'minVWithL', HJIextraArgs);    
    lxOld = lx; 
    
    % plotting and save level sets.
    plot_and_save_value_funs(data, view, unionL_2D_FMM, signed_dist, paths, i, viewRadius, false);
    plot_and_save_value_funs(hjipde_data, view, unionL_2D_FMM, signed_dist, paths, i, viewRadius, true);
end 

%% Compute optimal trajectory from some initial state
if compTraj
  pause
  
  %set the initial state
  xinit = [2, 1, -pi];
  
  %check if this initial state is in the BRS/BRT
  %value = eval_u(g, data, x)
  value = eval_u(g,data(:,:,:,end),xinit);
  
  if value <= 0 %if initial state is in BRS/BRT
    % find optimal trajectory
    
    dCar.x = xinit; %set initial state of the dubins car

    TrajextraArgs.uMode = uMode; %set if control wants to min or max
    TrajextraArgs.visualize = true; %show plot
    TrajextraArgs.fig_num = 2; %figure number
    
    %we want to see the first two dimensions (x and y)
    TrajextraArgs.projDim = [1 1 0]; 
    
    %flip data time points so we start from the beginning of time
    dataTraj = flip(data,4);
    
    % [traj, traj_tau] = ...
    % computeOptTraj(g, data, tau, dynSys, extraArgs)
    [traj, traj_tau] = ...
      computeOptTraj(g, dataTraj, tau2, dCar, TrajextraArgs);
  else
    error(['Initial state is not in the BRS/BRT! It have a value of ' num2str(value,2)])
  end
end
end

function [x, y] = IndicesToCoords(nx, ny, min_x, min_y, x_disc, y_disc)
    x = round((nx - min_x) / x_disc) + 1; % 1-index
    y = round((ny - min_y) / y_disc) + 1; % 1-index
end 

function [x, y] = CoordsToIndices(nx, ny, min_x, min_y, x_disc, y_disc)
    x = min_x + x_disc * (nx - 1); % 1-index
    y = min_y + y_disc * (ny - 1); % 1-index
end 

function updatedView = UpdateViewMap(xpos, view, viewRadius, N, shape, grid_min, grid_disc) 
    updatedView = view;
    for ny = 1:N
        for nx = 1:N
            [x, y] = CoordsToIndices(nx, ny, grid_min(1), grid_min(2), grid_disc(1), grid_disc(2));
            dist = (x - xpos(1))^2 + (y - xpos(2))^2;
            if dist <= (viewRadius * viewRadius) && shape(ny, nx) == 1
                updatedView(ny, nx) = 1; 
            end 
        end
    end 
end


function plot_and_save_value_funs(valueFun, view, unionL_2D_FMM, signed_dist, paths, time, sensing_radius, hjipde)
    % plot view
    f=figure(1);
    clf;
    contour(view);
    xlabel('x');
    ylabel('y');
    title(sprintf('robot view at time %d with radius %.3f', time, sensing_radius));
    saveas(f, strcat(paths, sprintf('view/view_%d.png', time)));
    
    % plot unionL_2D_FMM
    f=figure(1);
    clf;
    contour(unionL_2D_FMM);
    xlabel('x');
    ylabel('y');
    title(sprintf('unionL2DFMM at time %d with radius %.3f', time, sensing_radius));
    saveas(f, strcat(paths, sprintf('unionL_2D_FMM/unionL_2D_FMM_%d.png', time)));
    contour(unionL_2D_FMM);
    
    % plot level set hyperparameters
    projpt = 4;
    timestep = 41;
    EPS = 0.25;
    if hjipde 
        alg = 'HJIPDE/';    
    else 
        alg = 'warm_start/';
    end 
    
    % plot zero level set
    f=figure(1);
    clf;
    zero_level_set = abs(valueFun(:, :, projpt, timestep)) <= EPS;
    total_points = numel(zero_level_set);
    num_points = sum(sum(zero_level_set));
    contour(zero_level_set);
    xlabel('x');
    ylabel('y');
    title(sprintf('%s zero level set at time %d containing %d/%d points', alg, time, num_points, total_points));
    saveas(f, strcat(paths, sprintf('%szero_level_set/zero_level_set_%d.png', alg, time)));
    
    % plot positive level set
    f=figure(1);
    clf;
    positive_set = valueFun(:, :, projpt, timestep) >= 0;
    num_points = sum(sum(positive_set));
    contour(positive_set);
    xlabel('x');
    ylabel('y');
    title(sprintf('%s positive set at time %d containing %d/%d points', alg, time, num_points, total_points));
    saveas(f, strcat(paths, sprintf('%s/positive_set/positive_set_%d.png', alg, time)));
    
    % save values
    views = view'; % save transpose of views because matrix is in reverse order.
    save(strcat(paths, sprintf('values/t%d.mat', time)), 'valueFun', 'signed_dist', 'views');
end

function plot_and_save_env(x_positions, shape, paths, g)
    % plot trajectory
    f=figure(1);
    clf;
    n = length(x_positions);
    x = zeros(n, 1);
    y = zeros(n, 1);
    for i = 1:n
        x(i) = x_positions{i}(1);
        y(i) = x_positions{i}(2);
    end
    scatter(x, y);
    axis([g.min(1) g.max(1) g.min(2) g.max(2)]);
    xlabel('x');
    ylabel('y');
    title('trajectory');
    saveas(f, strcat(paths, 'trajectory.png'));
    
    % plot shape
    f=figure(1);
    clf;
    contour(shape);
    xlabel('x');
    ylabel('y');
    title('obstacle');
    saveas(f, strcat(paths, 'obstacle.png'));
end
