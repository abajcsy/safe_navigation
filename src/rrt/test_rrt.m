clf
clear all

% create grid.
gridLow = [0,0];
gridUp = [5,5];
N = [20,20];
pdDims = [];
grid = createGrid(gridLow, gridUp, N, pdDims);

% create obstacle map
occuGrid = ones(N(1),N(2));
occuGrid(3,3) = -1;
occuGrid(3,4) = -1;
occuGrid(3,5) = -1;

% create rrt obj
rrt = RRT(grid, occuGrid);

%% check building rrt
xinit = [0,0];
xgoal = [5,5];
dx = 0.05;
showTree = true;
rrt.plotOccuGrid();
nodes = rrt.build(xinit, xgoal, dx, showTree);

%% Check getting shortest path
path = nodes.getPath(xgoal);
rrt.plotPath(path);

%% check finding NN in tree
% t = KDTree(xinit, 2);
% t.insert([5.0,0.0]);
% t.insert([3.75,0.0]);
% [bestDist, bestPt] = t.findNNEuclid([2.5,0.0]);


%% check state to grid index conversion
% x = [1,1];
% gridIdx = rrt.stateToGrid(x);
% assert(isequal(gridIdx, [2,2]));

%% check collision-checking
% gridIdx = [3,3];
% colFree = rrt.collisionCheck(gridIdx);
% assert(colFree == false);
% gridIdx = [1,1];
% colFree = rrt.collisionCheck(gridIdx);
% assert(colFree == true);

%% check generating random collision-free state
% xrand = rrt.getRandState();

% check dubins path generation
% dubins = DubinsPath(dynSys, occuGrid);
% xgoal = [0;2;pi/2];
% [~,~, pathLen] = dubins.getDubinsPath(xinit, xgoal);
% assert(sum(pathLen) == 2);
% 
% xinit = [0;0;0];
% xgoal = [3;0;0];
% [~,~, pathLen] = dubins.getDubinsPath(xinit, xgoal);
% assert(sum(pathLen) == 3);