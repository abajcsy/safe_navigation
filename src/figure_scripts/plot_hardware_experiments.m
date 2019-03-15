clf
clear all

%% Load map
load('/home/abajcsy/hybrid_ws/src/safe_navigation/data/resizedSLAMTrueOccuMap.mat');
slamMap = trueOccuMap;

%% Load params.
params = car4DLocalQCameraSLAM();

%% Plot ground-truth SLAM environment.
hold on
[grid2D, ~] = proj(params.grid, params.grid.xs{1}, [0 0 1 1], [0 0]);
contourf(grid2D.xs{1}, grid2D.xs{2}, -slamMap, [0,0]);

%% Plot goal.
c = [0.1,0.8,0.5,0.5];
pos = [params.xgoal(1)-params.goalEps, params.xgoal(2)-params.goalEps, params.goalEps*2, params.goalEps*2];
rectangle('Position',pos,'Curvature',1.0,'FaceColor',c,'LineStyle','none');
scatter(params.xgoal(1),params.xgoal(2),[],[0.0,0.8,0.5],'filled');

%% 