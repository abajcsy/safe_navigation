%% Generates solution that HJI would have produced if it visited all the
%  states that HJI-warm or localQ method did. Saves this out for
%  comparison.
clear all

path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data/';
HJIwarm_rrt_camera_filename = 'HJIwarm_rrt_camera_hand.mat';

filename = HJIwarm_rrt_camera_filename;
filePath = strcat(path, filename);
load(filePath);
gtParams = car3DHJICameraRRT();

% Time vector.
timeDisc = 0:gtParams.dt:gtParams.tMax; 

% Put grid and dynamic systems into schemeData.
schemeData.grid = gtParams.grid;
schemeData.dynSys = gtParams.dynSys;
schemeData.accuracy = 'high'; % Set accuracy.
schemeData.uMode = gtParams.uMode;
if ~isempty(gtParams.dMode)
    schemeData.dMode = gtParams.dMode;
end

% Save out sequence of value functions as system moves through
% space as well as the cost functions and the max Q size (if using Q method). 
gtValueFunCellArr = {};

% since we have a finite compute grid, we may not want to 
% trust values near the boundary of grid
HJIextraArgs.ignoreBoundary = 0; 
HJIextraArgs.stopConverge = 1;
HJIextraArgs.convergeThreshold = gtParams.updateEpsilon;

idx = 1;
lxOld = [];
minWith = 'minVWithL';
warmStart = false;

for i=1:length(states)
    currUpdateT = updateTimeArr(idx);
    if i == currUpdateT
        % grab cost function used at this step.
        lCurr = lxCellArr{idx};
        data0 = lCurr;
        HJIextraArgs.targets = lCurr;

        [dataOut, tau, extraOuts] = ...
            HJIPDE_solve_warm(data0, lxOld, lCurr, ...
            timeDisc, schemeData, minWith, ...
            warmStart, HJIextraArgs); 
        
        gtValueFunCellArr{end+1} = dataOut(:,:,:,:,end);
        
        idx = idx + 1;
        if idx > length(updateTimeArr)
            idx = length(updateTimeArr);
        end
    end
end

save('groundTruth_HJIwarm_rrt_camera_hand.mat', 'gtValueFunCellArr');