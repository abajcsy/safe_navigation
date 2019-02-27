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
method = 'HJI';
warm = false;
sensor = 'lidar';

%% Grab the data.
path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data/';
if strcmp(method, 'HJI')
    if warm
        methodName = 'Warm Start HJI-VI';
        filename = strcat('HJIwarm', sensor, '_dawkins.mat');
        filePath = strcat(path, filename);
        load(filePath);
        color = [0.1,0.1,1.0];
    else
        methodName = 'HJI-VI';
        filename = strcat('HJI', sensor, '_dawkins.mat');
        filePath = strcat(path, filename);
        load(filePath);
        color = [0,0,0];
    end
elseif strcmp(method, 'localQ')
    methodName = 'Local Update Algorithm';
    filename = strcat('localQwarm', sensor, '_dawkins.mat');
    filePath = strcat(path, filename);
    load(filePath);
    color = [1.0,0.1,0.1];
else
    error('Unsupported method type.');
end

%% Compute avg num states updated
totalPerStep = [];
for i=1:length(QSizeCellArr)
    totalThisStep = sum(QSizeCellArr{i});
    totalPerStep = [totalPerStep, totalThisStep];
end
avgNumStatesUpdated = mean(totalPerStep);


%% Printing 
fprintf('-------- %s (%s)--------\n', methodName, sensor);
fprintf("avg total compute time (s): %f\n", mean(solnTimes)); 
fprintf("avg num states updated: %f\n", avgNumStatesUpdated);

%% Compute num conservative states.
load('grid.mat');
if strcmp(method, 'HJI') && ~warm
    numConservStates = 0;
else
    otherValueFuns = valueFunCellArr;
    filename = strcat('HJI', sensor, '_dawkins.mat');
    filePath = strcat(path, filename);
    load(filePath);
    gtValueFuns = valueFunCellArr;
    epsilon = 0.01;
    numConservStates = howConservative(otherValueFuns, gtValueFuns, grid, epsilon); 
end
fprintf("avg num conservative states: %f\n", mean(numConservStates));


%% Checks how conservative the other value functions are wrt ground truth
function numConservStates = howConservative(otherValueFuns, gtValueFuns, grid, epsilon)
    numConservStates = [];
    for i=1:length(otherValueFuns)
        localFun = otherValueFuns{i};
        gtFun = gtValueFuns{i};

        % We want indicies to be empty (for local update to be more
        % conservative)
        locStates = (localFun < -epsilon).*(gtFun > epsilon);
        indicies = find(locStates);
        numConservStates(end+1) = length(indicies);
    end
end