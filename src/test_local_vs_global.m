%% Compare local update solution with ground truth solution.

% % Grab the data from both local updates.
% path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data/';
% trueSDPath = strcat(path, 'trueSignedDistanceWarmStart_reduced.mat');
% hackySDPath = strcat(path, 'hackySignedDistanceWarmStart_reduced.mat');
% gtPath = strcat(path, 'groundTruth_reduced.mat');
% 
% % Local Method 1 -- True Signed Distance
% load(trueSDPath);
% trueValueFuns = reducedValueFunCellArr;
% trueMaxQSize = maxQSize;
% 
% % Local Method 2 -- "Hacky" Signed Distance
% load(hackySDPath);
% hackyValueFuns = reducedValueFunCellArr;
% hackyMaxQSize = maxQSize;
% 
% % Ground Truth 
% load(gtPath);
% gtValueFuns = reducedValueFunCellArr;
% 
% % grab the grid
% load(strcat(path, 'grid.mat'));

%% Are we visiting fewer states? Q comparisons

% Average num of states visited vs. total num states visited in standard
% appraoch
groundTruthQSize = 31*31*21;
trueAvgQSize = mean(trueMaxQSize);
hackyAvgQSize = mean(hackyMaxQSize);

fprintf('----- True Signed Distance Metrics -----\n');
fprintf('Avg Q Size: %f, fraction of ground truth: %f\n', ... 
    trueAvgQSize, trueAvgQSize/groundTruthQSize);

fprintf('----- Hacky Signed Distance Metrics -----\n');
fprintf('Avg Q Size: %f, fraction of ground truth: %f\n', ... 
    hackyAvgQSize, hackyAvgQSize/groundTruthQSize);
fprintf('\n');

%% Are the local updates conservative? BRT comparisons

% Local Method 1 -- True Signed Distance
fprintf('----- True Signed Distance -----\n');
checkIfConservative(trueValueFuns, gtValueFuns, grid);

% Local Method 2 -- "Hacky" Signed Distance
fprintf('----- Hacky Signed Distance -----\n');
checkIfConservative(hackyValueFuns, gtValueFuns, grid);

%% How conservative are the local updates? BRT comparisons


%% Checks if other value functions are conservative wrt ground-truth value funcs.
function [isConserv, numUnsafeVx, unsafeStates] = checkIfConservative(otherValueFuns, gtValueFuns, grid)
    
    unsafeStates = [];
    numUnsafeVx = 0;
    
    epsilon = 0.005;
    isConserv = true;
    
    for i=1:length(otherValueFuns)
        localFun = otherValueFuns{i};
        gtFun = gtValueFuns{i};
        
        % We want indicies to be empty (for local update to be more
        % conservative)
        locUnsafeStates = (localFun > epsilon).*(gtFun < -epsilon);
        indicies = find(locUnsafeStates);
        if ~isempty(indicies)
            %hold on;
            %XReal = grid.xs{1}(indicies);
            %YReal = grid.xs{2}(indicies);
            TReal = grid.xs{3}(indicies);
            %scatter3(XReal, YReal, TReal);
            
            hold on;
           
            % plot xytheta states that are NOT conservative
            xythetaSet = visSetIm(grid, locUnsafeStates, 'r', 0);
            
            % plot the zero level set at a particular heading 
            [gSlice, dataSlice] = proj(grid, localFun, [0 0 1], TReal(1));
            xySet = visSetIm(gSlice, dataSlice, 'k', 0);
            
            % nice plotting stuff
            xlim([0,10]); xlabel('x');
            ylim([0,7]); ylabel('y');
            zlim([-pi,pi]); zlabel('theta');
            view(0,90);
            
            
            % keep track of the average number of points that are
            % not conservative 
            unsafeStates(end+1) = length(indicies);
            isConserv = false;
            % keep track of number of value functions with unsafe pts
            numUnsafeVx = numUnsafeVx+1;
            delete(xythetaSet);
            delete(xySet);
        end
    end
    
    if isConserv
        fprintf('Update is IS conservative!\n');
    else
        fprintf('Update is NOT conservative:\n');
        fprintf('   --> num unsafe Vxs: %f \n', numUnsafeVx);
        fprintf('   --> avg num unsafe states: %f \n', sum(unsafeStates)/length(unsafeStates));
    end
end
