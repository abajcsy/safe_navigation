clf
clear all

%% Load map
load('/home/abajcsy/hybrid_ws/src/safe_navigation/data/resizedSLAMTrueOccuMap.mat');
slamMap = trueOccuMap;

%% Load params.
params = car4DLocalQCameraSLAM();

%% Plot ground-truth SLAM environment.
f = figure(1);
hold on
[grid2D, ~] = proj(params.grid, params.grid.xs{1}, [0 0 1 1], [0 0]);
contourf(grid2D.xs{1}, grid2D.xs{2}, -slamMap, [0,0]);

%% Plot goal.
c = [0.1,0.8,0.5,0.5];
pos = [params.xgoal(1)-params.goalEps, params.xgoal(2)-params.goalEps, params.goalEps*2, params.goalEps*2];
rectangle('Position',pos,'Curvature',1.0,'FaceColor',c,'LineStyle','none');
scatter(params.xgoal(1),params.xgoal(2),[],[0.0,0.8,0.5],'filled');

%% Load the safe set sequence and odometry.
load('/home/abajcsy/hybrid_ws/src/safe_navigation/data/setAndOdom3.mat');

%% Colors
blue = [8./255., 75./255., 135./255.]; 
lightBlue = [47./255., 141./255., 224./255.]; 
green = [22./255., 191./255., 121./255.];
lightPurple = [171./255., 134./255., 206./255.];
purple = [109./255., 71./255., 198./255.]; 
pinkPurple = [116, 50, 147]/255.;
orange = [242./255., 160./255., 29./255.];

powderBlue = [146, 170, 232]/255.;
turquoise = [0, 166, 204]/255.;
purpleBlue = [71, 118, 247]/255.;

greenLight = [92, 191, 0]/255.;
magenta = [127, 0, 178]/255.;
pink = [211, 91, 201]/255;
darkPink = [132, 9, 122]/255.;
brightPink = [219, 43, 168]/255.;

jungleGreen = [82, 193, 32]/255.;
darkGreen = [29, 94, 0]/255.;

%% Setup alpha.
alphaArr = linspace(0.3, 1, (length(setCellArr)/2));
idx = 1;
%% Plot the SAFE state trajectory.
for i=1:2:length(setCellArr)
    state = stateCellArr{i};
    valueFun = setCellArr{i};
    onBoundary = isOnBoundary(state,params.grid,valueFun,params.safetyTol);
    alpha = alphaArr(idx);
    idx = idx + 1;
    if idx > length(alphaArr)
        idx = length(alphaArr);
    end
    if onBoundary
        cSafe = plotCar(state, 'r', 1, false);
    else
        cSafe = plotCar(state, [0, 137, 41]/255., 1, false);
    end
end

%% Load the UNSAFE traj.
load('/home/abajcsy/hybrid_ws/src/safe_navigation/data/pos_heading.mat');
states = data;
states = states - [0.05,0,0];
%% Plot the UNSAFE state trajectory.
step = 30;
alphaArr = linspace(0.15, 1, (length(states)/step));
idx = 1;
for i=1:step:length(states)
    state = states(i,:);
    alpha = alphaArr(idx);
    idx = idx + 1;
    if idx > length(alphaArr)
        idx = length(alphaArr);
    end
    cUnsafe = plotCar(state, [132, 216, 56]/255., alpha, true);
end

colormap gray

% plot initial point
plotCar(stateCellArr{1}, 'k', 1, false);
pSafe = scatter(-2,-2, 1, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', [0, 137, 41]/255.);
pUnsafe = scatter(-2,-2, 1, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', [132, 216, 56]/255.);
legend([pSafe, pUnsafe], {'Safety', 'No safety'}, 'Interpreter','latex', 'fontsize', 10, 'Location', 'northwest');

%% Plot shit.
xlim([params.lowEnv(1) 4]);
ylim([params.lowEnv(2) 2]);

set(gca,'TickLabelInterpreter','latex')
set(gcf, 'Color', 'w');
% setup size of figure and tick mark stuff.
set(gca,'XTick',[params.lowEnv(1) 1 4], 'fontsize', 12);
set(gca,'YTick',[params.lowEnv(2) -1 2], 'fontsize', 12);
xlabel('$p_x$', 'Interpreter','latex', 'fontsize', 16);
ylabel('$p_y$', 'Interpreter','latex', 'fontsize', 16);

widthMeters = abs(params.lowEnv(1) - 4);
heightMeters = abs(params.lowEnv(2) - 2);
set(gcf, 'Position',  [100, 100, widthMeters*100*0.6, heightMeters*100*0.6])

% Save the figure
saveas(f, './hybrid_ws/src/safe_navigation/imgs/slam_traj.png')
saveas(f, './hybrid_ws/src/safe_navigation/imgs/slam_traj.fig')
saveas(f, './hybrid_ws/src/safe_navigation/imgs/slam_traj.pdf')

%% Plots dubins car point and heading.
% Inputs:
%   x [vector]  - 3D/4D state of dubins car
% Ouput:
%   c   - handle for figure
function c = plotCar(x, carColor, alpha, isUnsafe)
    if isUnsafe
        c = quiver(x(1), x(2), cos(x(3)), sin(x(3)), 'o', 'Color', carColor, ...
            'MarkerSize', 3, 'MarkerEdgeColor', carColor, ...
            'MarkerFaceColor', carColor, 'MaxHeadSize', 5.0, ...
            'ShowArrowHead', 'off', 'AutoScaleFactor', 0.2, ...
            'LineWidth', 0.9); 
        c.Color(4) = alpha;
    else
    
        c{1} = scatter(x(1), x(2), 15, ...
            'MarkerEdgeColor', 'none', 'MarkerFaceColor', carColor, 'MarkerEdgeAlpha', alpha, 'MarkerFaceAlpha', alpha);

        % Plot heading.
        center = x(1:2);

        if length(x) >= 3
            % Rotation matrix.
            R = [cos(x(3)) -sin(x(3)); 
                 sin(x(3)) cos(x(3))];
            % Heading pt.
            hpt = [0.2; 0];
            hptRot = R*hpt + center;
            c{2} = plot([center(1) hptRot(1)], [center(2) hptRot(2)], 'Color', carColor, 'LineWidth', 1);
            c{2}.Color(4) = alpha;
        end
    end
end

%% Checks if is on boundary
function onBoundary = isOnBoundary(x,grid,valueFun,tol)
    % Grab the value at state x from the most recent converged 
    % value function.
    value = eval_u(grid, valueFun, x);

    % If the value is close to zero, we are close to the safety
    % boundary.
    if value < tol 
        onBoundary = true;
    else
        onBoundary = false;
    end
end