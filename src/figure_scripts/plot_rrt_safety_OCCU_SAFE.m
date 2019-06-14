%% Visualize safety and trajectory and occupancy map for spline + camera.
clf
clear all

% Data.
localQ_rrt_camera_filename = 'localQwarm_rrt_camera_hand.mat';
traj_filename = 'localQwarm_rrt_camera_hand_traj.mat';
traj_path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data_traj/';
path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data/';

% Grab the parameters and traj.
params = car3DLocalQCameraRRT();      
filePath = strcat(traj_path, traj_filename);
load(filePath);

% Grab the data.
filePath = strcat(path, localQ_rrt_camera_filename);
load(filePath);

%% Plot.
f = figure(1);
hold on

% Setup size of figure and tick mark stuff.
xlim([params.lowEnv(1) 5]);
ylim([params.lowEnv(2) 5]);
set(gca,'TickLabelInterpreter','latex')
set(gcf, 'Color', 'w');
set(gca,'XTick',[]);
set(gca,'YTick',[]);
box on

%% Plot car and sensing during part of traj.
[grid2D, ~] = proj(params.grid, valueFunCellArr{1}, [0 0 1], 0);

% Indicies of timesteps to show occupancy maps for.
nextIdx = 180; 
xnext = states{nextIdx};
    
% Create the belief map up till this point.
% nextCostIdx = find(updateTimeArr == (nextIdx+1));
% nextLx = lxCellArr{1};
% for i=2:nextCostIdx
%     currLx = lxCellArr{i};
%     nextLx = max(currLx, nextLx);
% end
% [~, nextMap] = proj(params.grid, nextLx, [0 0 1], 0); 

nextMap = safeOccuMaps{1};
for i=2:nextIdx
    currMap = safeOccuMaps{i};
    nextMap = max(currMap, nextMap);
end

% Plot obstacle map.
colormap gray
[cn, ~] = contourf(grid2D.xs{1}, grid2D.xs{2}, -nextMap, [0,0]);

%% Plot Safe Set.

% Next value function function.
% nextCostIdx = find(updateTimeArr == (nextIdx+1));
% nextVx = valueFunCellArr{nextCostIdx};
lxOld = [];
lCurr = repmat(nextMap, 1, 1, params.grid.N(3));

% Put grid and dynamic systems into schemeData.
schemeData.grid = params.grid;
schemeData.dynSys = params.dynSys;
schemeData.accuracy = 'high'; % Set accuracy.
schemeData.uMode = params.uMode;
schemeData.dMode = params.dMode;
timeDisc = 0:params.dt:params.tMax; 

firstHJIextraArgs.ignoreBoundary = 0; 
firstHJIextraArgs.quiet = true;
firstHJIextraArgs.stopConverge = 1;
firstHJIextraArgs.convergeThreshold = params.updateEpsilon;
firstHJIextraArgs.targets = lCurr;
data0 = lCurr;
firstWarmStart = false;
minWith = 'minVWithL';

% Solve. 
[dataOut, tau, extraOuts] = ...
HJIPDE_solve_warm(data0, lxOld, lCurr, ...
  timeDisc, schemeData, minWith, ...
  firstWarmStart, firstHJIextraArgs);

% Project and plot.
nextVx = dataOut(:,:,:,end);
[~, nextVxMap] = proj(params.grid, nextVx, [0 0 1], xnext(3)); 
[cn, hn] = contour(grid2D.xs{1}, grid2D.xs{2}, -nextVxMap, [0,0], 'r', 'LineWidth', 3.);

% Get safety control.
%uOpt = getSafetyControl(params.grid, params.dynSys, params.uMode, xnext, nextVx);

%% Plot sequence of states
carColorSeq = {[0.9,0.9,0.9], [0.7,0.7,0.7], [0.5,0.5,0.5], [0.2,0.2,0.2], [0.1,0.1,0.1], [0,0,0]};
alphaSeq = [0.1, 0.2, 0.3, 0.6, 0.7, 1.0]; %[0.1, 0.15, 0.2, 0.25, 0.3, 1.0]; 

% How frequently to plot everything
plotFreq = 30;
plotHoriz = 230; 
idx = 1;
colorIdx = 1;
fillC = [132, 134, 255]/255.; 
for i=1:plotHoriz
    currT = updateTimeArr(idx);
    xcurr = states{i};
    
    if mod(i,plotFreq) == 0 || i == 2      
        % plot car and sensing
        carColor = carColorSeq{colorIdx};
        plotCar(xcurr, carColor);
        
        colorIdx = colorIdx+1;
        if colorIdx > length(carColorSeq)
            colorIdx = length(carColorSeq);
        end
    end
end

widthMeters = abs(params.lowEnv(1) - 5);
heightMeters = abs(params.lowEnv(2) - 5);
set(gcf, 'Position',  0.9*[100, 100, widthMeters*100*0.6, heightMeters*100*0.6])

% Save the figure
saveas(f, './hybrid_ws/src/safe_navigation/imgs/rrt_occu_safe.png')
saveas(f, './hybrid_ws/src/safe_navigation/imgs/rrt_occu_safe.fig')
saveas(f, './hybrid_ws/src/safe_navigation/imgs/rrt_occu_safe.pdf')

%% Plots dubins car point and heading.
% Inputs:
%   x [vector]  - 3D/4D state of dubins car
% Ouput:
%   c   - handle for figure
function c = plotCar(x, carColor)
    c{1} = plot(x(1), x(2), 'o','MarkerSize', 5, ...
        'MarkerEdgeColor', carColor, 'MarkerFaceColor', carColor);

    % Plot heading.
    center = x(1:2);

    if length(x) >= 3
        % Rotation matrix.
        R = [cos(x(3)) -sin(x(3)); 
             sin(x(3)) cos(x(3))];
        % Heading pt.
        hpt = [0.3; 0];
        hptRot = R*hpt + center;
        p2 = plot([center(1) hptRot(1)], [center(2) hptRot(2)], 'Color', carColor, 'LineWidth', 1.5);
        c{2} = p2;
    end
end

%% Plots sensing radius centered around car position (x)
% Inputs:
%   grid  - 
%   signed_distance_map - 
% Ouput:
%   h   - handle for figure
function plotSensing(grid, signed_distance_map)
    [c, h]= contourf(grid.xs{1}, grid.xs{2}, signed_distance_map, [0,0]);
    x = c(1,2:end);
    y = c(2,2:end);
    delete(h);
    h = fill(x,y,[0,0.2,1],'FaceAlpha',0.3, 'EdgeColor', 'none');

%     posIdx = find(signed_distance_map > 0);
%     h = scatter(grid.xs{1}(posIdx),grid.xs{2}(posIdx), 30, ...
%         'MarkerFaceColor', [0,0.2,1], 'MarkerFaceAlpha', 0.3, 'MarkerEdgeColor', 'none');
end

%% Gets optimal control.
function uOpt = getSafetyControl(grid, dynSys, uMode, x, vx)
    deriv = computeGradients(grid, vx);
    % value of the derivative at that particular state
    current_deriv = eval_u(grid, deriv, x);
    % NOTE: need all 5 arguments (including NaN's) to get 
    % correct optimal control!
    uOpt = dynSys.optCtrl(NaN, x, current_deriv, uMode, NaN);
end