%% Analyzes performance metrics for three methods: HJI, warm HJI, local
%  Metrics:
%       - average total compute time
%       - average number of states updated
%       - average number of overly conservative states
% 
%  Variables (to load):
%       - valueFunCellArr (cell arr) 
%       - lxCellArr       (cell arr) 
%       - QSizeCellArr    (cell arr) 
%       - solnTimes       (array) 
clear all

%% Choose which method you wanna analyze.
%   HJI, localQ
method = 'localQ';
warm = true;
sensor = 'lidar';

%% Grab the data.
path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data/';
% if strcmp(method, 'HJI')
%     if warm
%         methodName = 'Warm Start HJI-VI';
%         filename = strcat('HJIwarm', sensor, '_dawkins.mat');
%         filePath = strcat(path, filename);
%         load(filePath);
%         color = [0.1,0.1,1.0];
%     else
%         methodName = 'HJI-VI';
%         filename = strcat('HJI', sensor, '_dawkins.mat');
%         filePath = strcat(path, filename);
%         load(filePath);
%         color = [0,0,0];
%     end
% elseif strcmp(method, 'localQ')
%     methodName = 'Local Update Algorithm';
%     filename = strcat('localQwarm', sensor, '_dawkins.mat');
%     filePath = strcat(path, filename);
%     load(filePath);
%     color = [1.0,0.1,0.1];
% else
%     error('Unsupported method type.');
% end

HJI_rrt_lidar_filename = 'HJI_rrt_lidar_hand.mat';
HJIwarm_rrt_lidar_filename = 'HJIwarm_rrt_lidar_hand.mat';
localQ_rrt_lidar_filename = 'localQwarm_rrt_lidar_hand.mat';

HJI_rrt_camera_filename = 'HJI_rrt_camera_hand.mat';
HJIwarm_rrt_camera_filename = 'HJIwarm_rrt_camera_hand.mat';
localQ_rrt_camera_filename = 'localQwarm_rrt_camera_hand.mat';

%% Choose which file we want to analyze and what is the ground truth.
filename = HJIwarm_rrt_lidar_filename; 
groundTruthFilename = HJI_rrt_lidar_filename;

% load all the files
filePath = strcat(path, filename);
load(filePath);
params = car3DWarmLidarRRT();       % NOTE: have to change this too!

%% Compute avg num states updated
totalPerStep = [];
for i=1:length(QSizeCellArr)
    totalThisStep = sum(QSizeCellArr{i});
    totalPerStep = [totalPerStep, totalThisStep];
end
avgNumStatesUpdated = mean(totalPerStep);


%% Printing 
%fprintf('-------- %s (%s)--------\n', methodName, sensor);
fprintf('-------- %s (%s)--------\n', filename);
fprintf("avg total compute time (s): %f\n", mean(solnTimes));
fprintf("median total compute time (s): %f\n", median(solnTimes));
fprintf("avg num states updated: %f\n", avgNumStatesUpdated);

%% Compute num conservative states.
grid = params.grid;
conservEpsilon = 0.01;

% Save the current value functions and load the ground truth ones.
otherValueFuns = valueFunCellArr;
filePath = strcat(path, groundTruthFilename);
load(filePath);
gtValueFuns = valueFunCellArr;

% Compute how many conservative states we had at each step through env.
numConservStates = howConservative(otherValueFuns, gtValueFuns, grid, conservEpsilon); 
fprintf("avg num conservative states: %f\n", mean(numConservStates));
fprintf("percent conservative states: %f\n", mean(numConservStates)/prod(grid.N)*100);

%% Checks how conservative the other value functions are wrt ground truth
function numConservStates = howConservative(otherValueFuns, gtValueFuns, grid, epsilon)
    numConservStates = [];
    numVxToCompare = 60;
    for i=1:numVxToCompare
        localFun = otherValueFuns{i};
        gtFun = gtValueFuns{i};

        % We want indicies to be empty (for local update to be more
        % conservative)
        locStates = (localFun < -epsilon).*(gtFun > epsilon);
        indicies = find(locStates);
        numConservStates(end+1) = length(indicies);
    end
end