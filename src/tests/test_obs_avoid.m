function test_obs_avoid()

%% Clear figure
clf

%% Grid
grid_min = [0; 0; -pi]; % Lower corner of computation domain
grid_max = [5; 5; pi];    % Upper corner of computation domain
N = [41; 41; 41];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);
% Use "g = createGrid(grid_min, grid_max, N);" if there are no periodic
% state space dimensions

%% target set
R = 1;
% data0 = shapeCylinder(grid,ignoreDims,center,radius)
%data0 = shapeCylinder(g, 3, [5; 2.5; 0], R);
lowObs = [4;1;-inf];
upObs = [5;4;inf];
data0 = shapeRectangleByCorners(g,lowObs,upObs);
%data0_2 = shapeRectangleByCorners(g,[0.5;4;-inf],[5;5;-inf]);
%data0 = shapeUnion(data0_1,data0_2);

%% time vector
t0 = 0;
tMax = 5;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
speed = 1;
wMax = 1;
% do dStep1 here

% control trying to min or max value function?
uMode = 'max';
% do dStep2 here


%% Pack problem parameters

% Define dynamic system
xinit = [0, 2.5, 0];
dCar = DubinsCar(xinit, wMax, speed); %do dStep3 here

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = dCar;
schemeData.accuracy = 'high'; %set accuracy
schemeData.uMode = uMode;
%do dStep4 here

%% Compute value function
% Convergence information
HJIextraArgs.stopConverge = 1;
HJIextraArgs.convergeThreshold=.01;

% HJIextraArgs.visualize.valueSet = 1;
% HJIextraArgs.visualize.initialValueSet = 1;
% HJIextraArgs.visualize.figNum = 1; %set figure number
% HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
% HJIextraArgs.visualize.viewGrid = false;
% 
% % uncomment if you want to see a 2D slice
% HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; % Plot x, y
% HJIextraArgs.visualize.plotData.projpt = [0]; % Project at theta = 0

%HJIextraArgs.visualize.plotData.plotDims = [1 0 0]; % Plot x dimension
%HJIextraArgs.visualize.plotData.projpt = {0,'min'}; % y = 0, theta unioned
%HJIextraArgs.visualize.viewAngle = [0,90]; % view 2D

%[data, tau, extraOuts] = ...
% HJIPDE_solve(data0, tau, schemeData, minWith, extraArgs)
minWith = 'zero';
[data, tau2, extraOuts] = ...
  HJIPDE_solve(data0, tau, schemeData, minWith, HJIextraArgs);

%% Plot area where we are doing computations
hold on

% plot initial set
initialSet = data(:,:,:,1);
plotLevelSet(g, initialSet, xinit(3));

% plot the converged set for the initial condition angle
finalSet = data(:,:,:,end);
convergedSetHandle = plotLevelSet(g, finalSet, xinit(3));

% draw rectangle which represents sensed (x,y) states
sensing = rectangle('Position',[grid_min(1),grid_min(2),5,5], ...
    'FaceColor',[0.5 .5 .5 .2], 'Linestyle', 'none');

% draw *actual* obstacle (beyond compute radius)
obs = rectangle('Position',[4,1,3,3], 'Linewidth', 0.5, 'LineStyle', '--');

% setup the figure axes to represent the entire environment
xlim([0 10]);
ylim([0 7]);
%daspect([1 2 1]);

%% Simulate dubins car moving around environment and the safe set changing
T = 100; % total simulation timesteps
x = [xinit(1); xinit(2); xinit(3)];
for t=1:T
    if t >= 1 && t <= 20
        u = -0.5;
    elseif t >= 20 && t <= 80
        u = 0.8;
    else 
        u = -0.8;
    end
    % apply control to dynamics
    dx = dynamics(dCar,t,x,u);
    x = x + dx*dt;
    
    % plot the state of the car (point)
    plot(x(1), x(2), 'ko','MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
    drawnow   
    
    % plot sensing radius
    viscircles([x(1), x(2)], 1.5, 'Color', [1,0,0,0.5]);
    
    % delete old set's visualization
    delete(convergedSetHandle);
    
    % plot level set corresponding to the current theta
    convergedSetHandle = plotLevelSet(g, finalSet, x(3));
end
end

%% Plot slice of value function at specific theta
%  data -- (converged or at a specific time) value function
%  output -- plots level set in (x,y) for fixed theta.
function handle = plotLevelSet(grid, data, theta)
    % grab slice at theta
    [gPlot, dataPlot] = proj(grid, data, [0 0 1], [theta]);
    
    % visualize final set
    handle = visSetIm(gPlot, dataPlot, 'k', 0);
end