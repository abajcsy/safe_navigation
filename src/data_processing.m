%% Loads a sequence of value functions and grabs just the final solution
%  and saves out a sequence of V(x,0)'s as the car moves through env. 

path = '/home/ext_drive/somilb/safe_navigation_data/';
hackySDVals = strcat(path, 'hackySignedDistanceWarmStart.mat');
trueSDVals = strcat(path, 'trueSignedDistanceWarmStart.mat');

% Load the sequence of values.
load(trueSDVals);

% Need to grab valueFunCellArr, maxQSize
reducedValueFunCellArr = {};
for i=1:201
    reducedValueFunCellArr{i} = valueFunCellArr{i}(:,:,:,end);
end

% Save the reduced value func array.  
save('trueSDWarmStart_reduced.mat', 'reducedValueFunCellArr', 'maxQSize');