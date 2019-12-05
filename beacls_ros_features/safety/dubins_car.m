function dubins()
% 1. Run Backward Reachable Set (BRS) with a goal
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = false <-- no trajectory
% 2. Run BRS with goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'none' <-- Set (not tube)
%     compTraj = true <-- compute optimal trajectory
% 3. Run Backward Reachable Tube (BRT) with a goal, then optimal trajectory
%     uMode = 'min' <-- goal
%     minWith = 'minVWithTarget' <-- Tube (not set)
%     compTraj = true <-- compute optimal trajectory
% 4. Add disturbance
%     dStep1: define a dMax (dMax = [.25, .25, 0];)
%     dStep2: define a dMode (opposite of uMode)
%     dStep3: input dMax when creating your DubinsCar
%     dStep4: add dMode to schemeData
% 5. Change to an avoid BRT rather than a goal BRT
%     uMode = 'max' <-- avoid
%     dMode = 'min' <-- opposite of uMode
%     minWith = 'minVWithTarget' <-- Tube (not set)
%     compTraj = false <-- no trajectory
% 6. Change to a Forward Reachable Tube (FRT)
%     add schemeData.tMode = 'forward'
%     note: now having uMode = 'max' essentially says "see how far I can
%     reach"
% 7. Add obstacles
%     add the following code:
%     obstacles = shapeCylinder(g, 3, [-1.5; 1.5; 0], 0.75);
%     HJIextraArgs.obstacles = obstacles;
% 8. Add random disturbance (white noise)
%     add the following code:
%     HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];

%% Should we compute the trajectory?
compTraj = false;

%% Grid
grid_min = [-10; -10; -pi]; % Lower corner of computation domain
grid_max = [10; 10; pi];    % Upper corner of computation domain
N = [161; 81; 11];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions


%% time vector
t0 = 0;
tMax = 2.0;
dt = 0.025;
tau = t0:dt:tMax;

%% problem parameters
% input bounds
speed = 1;
wMax = 1;
% Define dynamic system
% obj = DubinsCar(x, wMax, speed, dMax)
dCar = DubinsCar([0, 0, 0], wMax, speed); %do dStep3 here


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
R = 5;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
data0 = shapeCylinder(g, 3, [0; 0; 0], R);
% also try shapeRectangleByCorners, shapeSphere, etc.

%% additive random noise
%do Step8 here
%HJIextraArgs.addGaussianNoiseStandardDeviation = [0; 0; 0.5];
% Try other noise coefficients, like:
%    [0.2; 0; 0]; % Noise on X state
%    [0.2,0,0;0,0.2,0;0,0,0.5]; % Independent noise on all states
%    [0.2;0.2;0.5]; % Coupled noise on all states
%    {zeros(size(g.xs{1})); zeros(size(g.xs{1})); (g.xs{1}+g.xs{2})/20}; % State-dependent noise

%% If you have obstacles, compute them here

%% Compute value function
HJIextraArgs.targets = data0;
HJIextraArgs.visualize = true; %show plot
HJIextraArgs.fig_num = 1; %set figure number
HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update

%[data, tau, extraOuts] = ...
% HJIPDE_solve_warm(data0, lxOld, lx, tau, schemeData, compMethod, warmStart, extraArgs)
[data, tau2, ~] = ...
  HJIPDE_solve_warm(data0, [], data0, tau, schemeData, 'minVWithL', true, HJIextraArgs);
t1_initial_data_m = data;
t1_initial_time_m = tau2;
t1_initial_grid_m = g;
save(sprintf('../compare/mt1_%s.mat', mfilename), 't1_initial_data_m', 't1_initial_time_m', 't1_initial_grid_m');

R = 1;
lxOld = data0; 
lx = shapeCylinder(g, 3, [0; 0; 0], R);
data0 = data(:, :, :, end);
HJIextraArgs.targets = lx;
[data2, tau3, ~] = HJIPDE_solve_warm(data0, lxOld, lx, tau, schemeData, 'minVWithL', true, HJIextraArgs);
t2_initial_data_m = data2;
t2_initial_time_m = tau3;
t2_initial_grid_m = g;
save(sprintf('../compare/mt2_%s.mat', mfilename), 't2_initial_data_m', 't2_initial_time_m', 't2_initial_grid_m');


% Plotting
% (3D system) Grab slice at theta.
figure(3)
theta = -pi;
[gPlot, dataPlot] = proj(g, data2(:,:,:,end), [0 0 1], theta);
visExtraArgs.LineWidth = 2.0;
h = visSetIm(gPlot, dataPlot, 'r', 0, visExtraArgs);
clf;
% Debug Outputs 
% fprintf("Value Functions: "); 
% for i=1:81
%     fprintf("%f ", d2(11 * 41 * 21 * (i - 1) + 1));
% end 
% fprintf("\n");
% fprintf("Data Functions: "); 
% for i=1:10
%     fprintf("%f ", data0(i)); 
% end
% fprintf("\n"); 

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
