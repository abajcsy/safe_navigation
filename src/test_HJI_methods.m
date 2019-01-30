function [datas, taus, h]= test_HJI_methods(...
    discountMode, discountFactor, discountAnneal)
%% Grid
grid_min = [-3; -3; -pi]; % Lower corner of computation domain
grid_max = [3; 3; pi];    % Upper corner of computation domain
N = [71; 71; 71];         % Number of grid points per dimension
pdDims = 3;               % 3rd dimension is periodic
g = createGrid(grid_min, grid_max, N, pdDims);

%% target set

% Define l(x)
target = -shapeRectangleByCorners(g,[-2;-2;-inf],[2;2;inf]);
HJIextraArgs.targets = target;

% Make funky initial value function
data0_1 = shapeCylinder(g,3,[-2;0;0],1);
data0_2 = shapeCylinder(g,3,[1;0;0],.5);
data0 = shapeUnion(data0_1,data0_2);

% If you have obstacles, compute them here
%obstacles = shapeRectangleByCorners(g, [-2 0 -inf], [2 2 inf]);

%% time vector
t0 = 0;
tMax = 100;
dt = 0.05;
tau = t0:dt:tMax;

%% problem parameters

% input bounds
speed = 1;
wMax = 1.5;

% control trying to min or max value function?
uMode = 'max';

%% Pack problem parameters

% Put grid and dynamic systems into schemeData
schemeData.grid = g;
schemeData.dynSys = DubinsCar([0;0;0], wMax, speed);
schemeData.accuracy = 'veryHigh'; %set accuracy
schemeData.uMode = uMode;

%% Visualization Parameters

% Visualize the value function
visualize.valueFunction = 1;

% visualization setup
visualize.figNum = 1; %set figure number
visualize.deleteLastPlot = 1; %delete previous plot as you update
visualize.holdOn = 1; % don't clear the figure when starting
visualize.viewAxis = [grid_min(1) grid_max(1) -2 2];
visualize.xTitle =  'x';
visualize.yTitle =  'V';

% Which dimensions to plot
visualize.plotData.plotDims = [1 0 0]; % Plot x dimension
visualize.plotData.projpt = {0,'min'}; % y = 0, theta unioned

%% Pack extra arguments

% pack visualization stuff
HJIextraArgs.visualize = visualize;

% Convergence information
HJIextraArgs.stopConverge = 1;
HJIextraArgs.convergeThreshold=.01;

% Ignore changes along boundary when checking for convergence
HJIextraArgs.ignoreBoundary = 1; 

% Keep only the most recent value function (saves on memory)
HJIextraArgs.keepLast = 1;

% Add obstacles if you want
%HJIextraArgs.obstacles = obstacles;

% Make a video if you want
HJIextraArgs.makeVideo = 0;

% Title video if you want, otherwise will be titled with time stamp
%HJIextraArgs.videoFilename = 'V_funky.mp4';

%% Compute correct solution

% want to visualize l(x)
HJIextraArgs.visualize.targetFunction = 1;
HJIextraArgs.visualize.plotColorTF = 'g';

% Minimization method
minWith = 'zero';

% Color of value function
HJIextraArgs.visualize.plotColorVF = 'y';

% Solution
[dataCorrect, tauCorrect, extraOuts] = ...
  HJIPDE_solve(HJIextraArgs.targets, tau, schemeData, minWith, HJIextraArgs);
h.VF_Correct = extraOuts.hVF;

%% Set discount factor
if nargin < 1
    % if you write 'Jaime' does Jaime method of discounting, all other
    % strings/options default to 'Kene'
    HJIextraArgs.discountMode = 'Jaime'; 
else
    HJIextraArgs.discountFactor = discountMode;
end

if nargin < 2
    HJIextraArgs.discountFactor = .999;
else
    HJIextraArgs.discountFactor = discountFactor;
end

if nargin < 3
    % for annealing set to 'hard' or 1 to do hard, 'soft' to do soft
    HJIextraArgs.discountAnneal = 0;
else
    HJIextraArgs.discountAnneal = discountAnneal;
end

%% Method 1: min V with l(x)

% don't need to re-plot l(x)
HJIextraArgs.visualize.targetFunction = 0;

% Minimization method
minWith = 'minVWithTarget';

% Color of value function
HJIextraArgs.visualize.plotColorVF = 'r';

% Solution
[dataTarget, tauTarget, extraOuts] = ...
  HJIPDE_solve(data0, tau, schemeData, minWith, HJIextraArgs);
h.VF_target = extraOuts.hVF;

%% Method 2: min Ham with zero

% remove l(s) from extraArgs to reset
HJIextraArgs = rmfield(HJIextraArgs,'targets');

% Minimization method
minWith = 'zero';

% Color of value function
HJIextraArgs.visualize.plotColorVF = 'b';

% Solution
[dataZero, tauZero, extraOuts] = ...
    HJIPDE_solve(data0, tau, schemeData, minWith, HJIextraArgs);
h.VF_zero = extraOuts.hVF;

%% Method 3: min with initial value function

% Minimization method
minWith = 'minVWithV0';

% Color of value function
HJIextraArgs.visualize.plotColorVF = 'c';

% Solution
[dataV0, tauV0, extraOuts] = ...
  HJIPDE_solve(data0, tau, schemeData, minWith, HJIextraArgs);
h.VF_V0 = extraOuts.hVF;

%% Method 4: min with previous value function

% Minimization method
minWith = 'minVOverTime';

% Color of value function
HJIextraArgs.visualize.plotColorVF = 'k';

% Solution
[dataVLast, tauVLast, extraOuts] = ...
  HJIPDE_solve(data0, tau, schemeData, minWith, HJIextraArgs);
h.VF_VLast = extraOuts.hVF;

%% Save Everything

datas = {dataCorrect, dataTarget, dataZero, dataV0, dataVLast};
taus = {tauCorrect, tauTarget, tauZero, tauV0, tauVLast};

end