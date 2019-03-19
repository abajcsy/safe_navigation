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
hold on

% visualize goal region.
c = [0.1,0.8,0.5,0.5];
pos = [params.xgoal(1)-params.goalEps, params.xgoal(2)-params.goalEps, params.goalEps*2, params.goalEps*2];
rectangle('Position',pos,'Curvature',1.0,'FaceColor',c,'LineStyle','none');
scatter(params.xgoal(1),params.xgoal(2),[],[0.0,0.8,0.5],'filled');

% Setup the figure axes to represent the entire environment
% xlim([params.lowEnv(1) params.upEnv(1)]);
% ylim([params.lowEnv(2) params.upEnv(2)]);

xlim([params.lowEnv(1) 5]);
ylim([params.lowEnv(2) 5]);

set(gca,'TickLabelInterpreter','latex')
set(gcf, 'Color', 'w');
% setup size of figure and tick mark stuff.
set(gca,'XTick',[params.lowEnv(1) params.upEnv(1)]);
set(gca,'YTick',[params.lowEnv(2) params.upEnv(2)]);
xlabel('$p_x$', 'Interpreter','latex', 'fontsize', 12);
ylabel('$p_y$', 'Interpreter','latex', 'fontsize', 12);
box on

%% Plot environment.
for i=1:length(params.obstacles)
    lowObs = params.obstacles{i}{1};
    upObs = params.obstacles{i}{2};
    width = upObs(1) - lowObs(1);
    height = upObs(2) - lowObs(2);
    obsCoord = [lowObs(1), lowObs(2), width, height];
    %e = rectangle('Position', obsCoord, 'Linewidth', 2.0, 'LineStyle', '--'); 
    e = rectangle('Position', obsCoord, ...
        'FaceColor', [0.5,0.5,0.5], 'EdgeColor', [0.4,0.4,0.4]); 
end

%% Plot car and sensing during part of traj.
[grid2D, ~] = proj(params.grid, valueFunCellArr{1}, [0 0 1], 0);

carColorSeq = {[0.9,0.9,0.9], [0.7,0.7,0.7], [0.5,0.5,0.5], [0.2,0.2,0.2], [0,0,0]};
alphaSeq = [0.1, 0.15, 0.2, 0.25, 1.0]; 

% indicies of timesteps to show occupancy maps for.
prevIdx = 280;
nextIdx = 180;
xprev = states{prevIdx};
xnext = states{nextIdx};

% grab cost functions at each state.
prevCostIdx = find(updateTimeArr == (prevIdx+1));
nextCostIdx = find(updateTimeArr == (nextIdx+1));
prevVx = valueFunCellArr{prevCostIdx};
nextVx = valueFunCellArr{nextCostIdx};

[~, prevMap] = proj(params.grid, prevVx, [0 0 1], xprev(3)); 
[~, nextMap] = proj(params.grid, nextVx, [0 0 1], xnext(3)); 
%[cp, hp] = contour(grid2D.xs{1}, grid2D.xs{2}, -prevMap, [0,0], 'r', 'LineWidth', 2.);
[cn, hn] = contour(grid2D.xs{1}, grid2D.xs{2}, -nextMap, [0,0], 'r', 'LineWidth', 2.);

% plot car
%plotCar(xprev, [0.5,0.5,0.5]);
plotCar(xnext, 'k');

% get safety control.
uOpt = getSafetyControl(params.grid, params.dynSys, params.uMode, xnext, nextVx);

%% Gets optimal control.
function uOpt = getSafetyControl(grid, dynSys, uMode, x, vx)
    deriv = computeGradients(grid, vx);
    % value of the derivative at that particular state
    current_deriv = eval_u(grid, deriv, x);
    % NOTE: need all 5 arguments (including NaN's) to get 
    % correct optimal control!
    uOpt = dynSys.optCtrl(NaN, x, current_deriv, uMode, NaN);
end

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
        hpt = [0.5; 0];
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