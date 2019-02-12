%% Compare local update solution with ground truth solution.
clf

% % Grab the data from both local updates.
path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data/';
% trueSDPath = strcat(path, 'trueSignedDistanceWarmStart_reduced.mat');
% hackySDPath = strcat(path, 'hackySignedDistanceWarmStart_reduced.mat');
% pureWarmPath = strcat(path, 'pureWarmStart_reduced.mat');
gtPath = strcat(path, 'groundTruth_reduced.mat');
%hjiWarmPath = strcat(path, 'warmHJI_reduced.mat');
%globalQPath = strcat(path, 'globalQ_reduced.mat');
warmGlobalQPath = strcat(path, 'warmGlobalQ_reduced.mat');

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
% % Local Method 3 -- Pure Warm Start
% load(pureWarmPath);
% pureValueFuns = reducedValueFunCellArr;
% pureMaxQSize = maxQSize;

% % Ground Truth 
load(gtPath);
gtValueFuns = reducedValueFunCellArr;

% Warm-start HJI
%load(hjiWarmPath);
%hjiWarmValueFuns = valueFunCellArr;

% Global Q Method
% load(globalQPath);
% globalQValueFuns = valueFunCellArr;

% Warm global Q Method
load(warmGlobalQPath);
warmGlobalQValueFuns = valueFunCellArr;
warmGlobalQSize = maxQSize;

% % grab the grid
load(strcat(path, 'grid.mat'));

%% Are we visiting fewer states? Q comparisons

% Average num of states visited vs. total num states visited in standard
% appraoch
groundTruthQSize = 31*31*21;
% trueAvgQSize = mean(trueMaxQSize);
% hackyAvgQSize = mean(hackyMaxQSize);
% pureAvgQSize = mean(pureMaxQSize);
warmGlobalAvgQSize = mean(warmGlobalQSize);

% 
% fprintf('----- True Signed Distance Metrics -----\n');
% fprintf('   --> avg Q Size: %f, fraction of ground truth: %f\n', ... 
%     trueAvgQSize, trueAvgQSize/groundTruthQSize);
% 
% fprintf('----- Hacky Signed Distance Metrics -----\n');
% fprintf('   --> avg Q Size: %f, fraction of ground truth: %f\n', ... 
%     hackyAvgQSize, hackyAvgQSize/groundTruthQSize);
% 
% fprintf('----- Pure Warm Start Metrics -----\n');
% fprintf('   --> avg Q Size: %f, fraction of ground truth: %f\n', ... 
%     pureAvgQSize, pureAvgQSize/groundTruthQSize);
% fprintf('\n');
% 
% fprintf('----- Global Q Metrics -----\n');
% fprintf('   --> avg Q Size: %f, fraction of ground truth: %f\n', ... 
%     globalAvgQSize, globalQSize/groundTruthQSize);
% fprintf('\n');

fprintf('----- (Warm) Global Q Metrics -----\n');
fprintf('   --> avg Q Size: %f, fraction of ground truth: %f\n', ... 
    warmGlobalAvgQSize, warmGlobalAvgQSize/groundTruthQSize);
fprintf('\n');

%% Are the local updates conservative? BRT comparisons

% Local Method 1 -- True Signed Distance
%fprintf('----- True Signed Distance -----\n');
%checkIfConservative(trueValueFuns, gtValueFuns, grid);

% Local Method 2 -- "Hacky" Signed Distance
%fprintf('----- Hacky Signed Distance -----\n');
%checkIfConservative(hackyValueFuns, gtValueFuns, grid);

% Local Method 3 -- Pure Warm Start
%fprintf('----- Pure Warm Start -----\n');
%checkIfConservative(pureValueFuns, gtValueFuns, grid);

% % Warm-started HJIPDE_solve()
% fprintf('----- Warm-started HJIPDE -----\n');
% checkIfConservative(hjiWarmValueFuns, gtValueFuns, grid);

% Global Q Method
% fprintf('----- Global Q -----\n');
% checkIfConservative(globalQValueFuns, gtValueFuns, grid);

% (Warm) Global Q Method
fprintf('----- (Warm) Global Q -----\n');
checkIfConservative(warmGlobalQValueFuns, gtValueFuns, grid);


%% How conservative are the local updates? BRT comparisons
% Local Method 1 -- True Signed Distance
% figure(1)
% fprintf('----- True Signed Distance -----\n');
% trueColor = [0.8,0.1,0.1];
% howConservative(trueValueFuns, gtValueFuns, grid, trueColor);
% legend('true');
% 
% % Local Method 2 -- "Hacky" Signed Distance
% figure(2)
% fprintf('----- Hacky Signed Distance -----\n');
% hackyColor = [0.1,0.8,0.1];
% howConservative(hackyValueFuns, gtValueFuns, grid, hackyColor);
% legend('hacky');
% 
% % Local Method 3 -- Pure Warm Start
% figure(3)
% fprintf('----- Pure Warm Start -----\n');
% pureColor = [0.1,0.1,0.8];
% howConservative(pureValueFuns, gtValueFuns, grid, pureColor);
% legend('pure');

%legend('true', 'hacky', 'pure');

% fprintf('----- Warm-started HJIPDE -----\n');
% hjiWarmColor = [0.6,0.1,0.8];
% howConservative(hjiWarmValueFuns, gtValueFuns, grid, hjiWarmColor);
% legend('warm HJI');

% fprintf('----- Global Q Method -----\n');
% globalQColor = [0.1,0.6,0.8];
% howConservative(globalQValueFuns, gtValueFuns, grid, globalQColor);
% legend('global Q');

fprintf('----- (Warm) Global Q Method -----\n');
warmGlobalQColor = [0.1,0.6,0.8];
howConservative(warmGlobalQValueFuns, gtValueFuns, grid, warmGlobalQColor);
legend('global Q (warm)');

%% Checks how conservative the other value functions are wrt ground truth
function howConservative(otherValueFuns, gtValueFuns, grid, color)
    epsilon = 0.005;
    hold on;

    title('Conservative States');
    numConservStates = [];
    for i=1:length(otherValueFuns)
        localFun = otherValueFuns{i};
        gtFun = gtValueFuns{i};

        % We want indicies to be empty (for local update to be more
        % conservative)
        locStates = (localFun < -epsilon).*(gtFun > epsilon);
        indicies = find(locStates);
        numConservStates(end+1) = length(indicies);
        
%         % plot xytheta states that are NOT conservative
%         extraArgs.applyLight = false;
%         unsafeStatesSet = visSetIm(grid, locStates, color, 0, extraArgs);
% 
%         % nice plotting stuff
%         xlim([0,10]); xlabel('x');
%         ylim([0,7]); ylabel('y');
%         zlim([-pi,pi]); zlabel('theta');
%         view(0,90);
%         pause(0.02);
% 
%         % delete visualization shit
%         if i ~= length(otherValueFuns)
%             delete(unsafeStatesSet);
%         end
    end
    fprintf('   --> avg num conservative states: %f \n', ...
        mean(numConservStates));
end


%% Checks if other value functions are conservative wrt ground-truth value funcs.
function [isConserv, numUnsafeVx, unsafeStates] = checkIfConservative(otherValueFuns, gtValueFuns, grid)
    
    unsafeStates = [];
    numUnsafeVx = 0;
    
    epsilon = 0.005;
    isConserv = true;
    
    title('Unconservative States');
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
            
%             hold on;
%            
%             % plot xytheta states that are NOT conservative
%             localFunSet = visSetIm(grid, localFun, [0.1,0.1,0.1], 0);
%             unsafeStatesSet = visSetIm(grid, locUnsafeStates, 'r', 0);
%             
%             % plot the zero level set at a particular heading 
%             %[gSlice, dataSlice] = proj(grid, localFun, [0 0 1], TReal(1));
%             %xySet = visSetIm(gSlice, dataSlice, 'k', 0);
%             
%             % nice plotting stuff
%             xlim([0,10]); xlabel('x');
%             ylim([0,7]); ylabel('y');
%             zlim([-pi,pi]); zlabel('theta');
%             view(0,90);
%             pause(0.1);


            % keep track of the average number of points that are
            % not conservative 
            unsafeStates(end+1) = length(indicies);
            isConserv = false;
            % keep track of number of value functions with unsafe pts
            numUnsafeVx = numUnsafeVx+1;
            
%             % delete visualization shit
%             if i ~= length(otherValueFuns)
%                 delete(localFunSet);
%                 delete(unsafeStatesSet);
%                 %delete(xySet);
%             end
        end
    end
    
    if isConserv
        fprintf('Update is IS conservative!\n');
    else
        fprintf('Update is NOT conservative:\n');
        fprintf('   --> num unsafe Vxs: %f \n', numUnsafeVx);
        fprintf('   --> avg num unsafe states: %f \n', mean(unsafeStates));
    end
end