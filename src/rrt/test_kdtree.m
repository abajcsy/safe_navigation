
load('grid.mat');

% create tree
xinit = [7,2];
numDims = 2;
tree = KDTree(xinit, numDims);

% insert some nodes
tree.insert([5,4]);
tree.insert([9,6]);
tree.insert([2,3]);
tree.insert([4,7]);
tree.insert([8,1]);

% find the closest point in tree.
searchPt = [5,8];
[bestDist, bestPt, closestNode] = tree.findNNEuclid(searchPt);