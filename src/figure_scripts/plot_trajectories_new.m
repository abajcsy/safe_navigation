%% Files.
clear all
clf

% -------- RRT --------- %
HJI_rrt_lidar_filename = 'HJI_rrt_lidar_hand_traj.mat';
HJIwarm_rrt_lidar_filename = 'HJIwarm_rrt_lidar_hand_traj.mat';
localQ_rrt_lidar_filename = 'localQwarm_rrt_lidar_hand_traj.mat';

HJI_rrt_camera_filename = 'HJI_rrt_camera_hand_traj.mat';
HJIwarm_rrt_camera_filename = 'HJIwarm_rrt_camera_hand_traj.mat';
localQ_rrt_camera_filename = 'localQwarm_rrt_camera_hand_traj.mat';

% -------- Spline -------- %
HJI_spline_lidar_filename = 'HJI_spline_lidar_hand_traj.mat';
HJIwarm_spline_lidar_filename = 'HJIwarm_spline_lidar_hand_traj.mat';
localQ_spline_lidar_filename = 'localQwarm_spline_lidar_hand_traj.mat';

HJI_spline_camera_filename = 'HJI_spline_camera_hand_traj.mat';
HJIwarm_spline_camera_filename = 'HJIwarm_spline_camera_hand_traj.mat';
localQ_spline_camera_filename = 'localQwarm_spline_camera_hand_traj.mat';

traj_path = '/home/abajcsy/hybrid_ws/src/safe_navigation/data_traj/';

% Grab the parameters.
params = car3DHJICameraRRT();       % NOTE: have to change this too!

%%
experiments = {{HJI_rrt_lidar_filename,HJI_rrt_camera_filename}, ...
               {HJI_spline_lidar_filename,HJI_spline_camera_filename}, ...
               {HJIwarm_rrt_lidar_filename,HJIwarm_rrt_camera_filename}, ...
               {HJIwarm_spline_lidar_filename,HJIwarm_spline_camera_filename}, ...
               {localQ_rrt_lidar_filename,localQ_rrt_camera_filename}, ...
               {localQ_spline_lidar_filename,localQ_spline_camera_filename}};


%% Setup big visualization loop.
for i=1:length(experiments)
    %% Load.
    curr_files = experiments{i};
    
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
    
    %% Construct data
    % Load Lidar
    filenameS = curr_files{1}; 
    filePathS = strcat(traj_path, filenameS);
    load(filePathS);
    lidar_xtraj = xtraj;
    lidar_ytraj = ytraj;
    lidar_thetatraj = thetatraj;
    lidar_appliedUOpt = appliedUOpt;

    % extract points where safe control was applied.
    li_safe_indicies = find(lidar_appliedUOpt == true);
    lidar_xSafe = lidar_xtraj(li_safe_indicies);
    lidar_ySafe = lidar_ytraj(li_safe_indicies);
    lidar_thetaSafe = lidar_thetatraj(li_safe_indicies);

    % Load Camera.
    filenameR = curr_files{2}; 
    filePathR = strcat(traj_path, filenameR);
    load(filePathR);
    camera_xtraj = xtraj;
    camera_ytraj = ytraj;
    camera_thetatraj = thetatraj;
    camera_appliedUOpt = appliedUOpt;
    % extract points where safe control was applied.
    cam_safe_indicies = find(camera_appliedUOpt == true);
    camera_xSafe = camera_xtraj(cam_safe_indicies);
    camera_ySafe = camera_ytraj(cam_safe_indicies);
    camera_thetaSafe = camera_thetatraj(cam_safe_indicies);
    
    %% Choose colors
    lidar_color = pinkPurple; %purple;
    camera_color = turquoise;

    %% Create subplot and environment.
    subplot(3,2,i); 
    hold on

    % visualize goal region.
    c = [0.1,0.8,0.5,0.5];
    pos = [params.xgoal(1)-params.goalEps, params.xgoal(2)-params.goalEps, params.goalEps*2, params.goalEps*2];
    rectangle('Position',pos,'Curvature',1.0,'FaceColor',c,'LineStyle','none');
    scatter(params.xgoal(1),params.xgoal(2),[],[0.0,0.8,0.5],'filled');

    % visualize the start region.
    plot(params.xinit(1),params.xinit(2),'o', 'MarkerSize', 4, ...
        'MarkerEdgeColor', 'none', 'MarkerFaceColor', [0.6,0.6,0.6]);

    % setup size of figure and tick mark stuff.
    set(gca,'XTick',[params.lowEnv(1) params.upEnv(1)]);
    set(gca,'YTick',[params.lowEnv(2) params.upEnv(2)]);
    widthMeters = abs(params.lowEnv(1) - params.upEnv(1));
    heightMeters = abs(params.lowEnv(2) - params.upEnv(2));
    %set(gcf, 'Position',  0.7*[100, 100, widthMeters*100*0.6, heightMeters*100*0.6])
    axis tight;

    % Setup the figure axes to represent the entire environment
    xlim([params.lowEnv(1) params.upEnv(1)]);
    ylim([params.lowEnv(2) params.upEnv(2)]);
    
    if i == 1
        title('RRT', 'Interpreter','latex', 'fontsize', 12);
        ylabel('HJI-VI', 'Interpreter','latex', 'fontsize', 12);
    end
    if i == 2
        title('Spline', 'Interpreter','latex', 'fontsize', 12);
    end
    if i == 3
        ylabel('HJI-VI Warm', 'Interpreter','latex', 'fontsize', 12);
    end
    if i == 5
        ylabel('Local', 'Interpreter','latex', 'fontsize', 12);
    end
    %if i == 1 || i == 3 || i == 5
    %   ylabel('$p_y$', 'Interpreter','latex', 'fontsize', 12);
    %end
    %if i == 5 || i == 6 
    %    xlabel('$p_x$', 'Interpreter','latex', 'fontsize', 12);
    %end
    %set(gca,'TickLength',[0 0]);
    %xticks(1:2:9);
    %yticks(0:2:5);
    set(gca,'TickLabelInterpreter','latex')
    set(gcf, 'Color', 'w');
    set(gca,'YTickLabel',[]);
    set(gca,'XTickLabel',[]);
    box on

    %% Plot Lidar.
    step = 60;
    num = floor(length(lidar_xtraj)/step);
    alphavals = linspace(0.1, 1, num);
    idx = 1;
    for i=1:step:length(lidar_xtraj)
        
        % Check if state is on safety boundary
        xcurr = [lidar_xtraj(i); lidar_ytraj(i); lidar_thetatraj(i)];
        onBoundary = lidar_appliedUOpt(i);
        
        % If on boundary, color red.
        c = lidar_color;
        if onBoundary
            c = 'r';
        end
        
        % Plot the car.
        lih = plotCar(xcurr, c, alphavals(idx));
        idx = idx + 1;
        if idx > length(alphavals)
            idx = length(alphavals);
        end
    end
    
    % plot all the safe states for lidar.
    for i=1:length(lidar_xSafe)
        % Plot the car.
        xcurr = [lidar_xSafe(i); lidar_ySafe(i); lidar_thetaSafe(i)];
        lih = plotCar(xcurr, 'r', 1.0);
    end
    
    %% Plot Camera.
    num = floor(length(lidar_xtraj)/step);
    alphavals = linspace(0.1, 1, num);
    idx = 1;
    for i=1:step:length(camera_xtraj)
        % Check if state is on safety boundary
        xcurr = [camera_xtraj(i); camera_ytraj(i); camera_thetatraj(i)];
        onBoundary = camera_appliedUOpt(i);
        
        % If on boundary, color red.
        c = camera_color;
        if onBoundary
            c = 'r';
        end
       
        % Plot the car.
        camh = plotCar([camera_xtraj(i); camera_ytraj(i); camera_thetatraj(i)], c, alphavals(idx));
        idx = idx + 1;
        if idx > length(alphavals)
            idx = length(alphavals);
        end
    end
    
    step = 15;
    num = floor(length(camera_xSafe)/step);
    alphavals = linspace(0.01, 1, num);
    idx = 1;
    % plot all the safe states for lidar.
    for i=1:step:length(camera_xSafe)
        % Plot the car.
        xcurr = [camera_xSafe(i); camera_ySafe(i); camera_thetaSafe(i)];
        lih = plotCar(xcurr, 'r', alphavals(idx));
        idx = idx + 1;
        if idx > length(alphavals)
            idx = length(alphavals);
        end
    end

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
end

lih_bar = plot([0,0], [1,1], '-', 'Color', lidar_color, 'LineWidth', 3);
camh_bar = plot([0,0], [1,1], '-', 'Color', camera_color, 'LineWidth', 3);
legend([lih_bar camh_bar],{'LiDAR','Camera'}, 'Interpreter','latex', 'fontsize', 10, 'Orientation','horizontal', 'Position', [0.5 0 0.1 0.05]);
%legend([lih camh],{'LiDAR','Camera'}, 'Interpreter','latex', 'fontsize', 10, 'Orientation','horizontal', 'Position', [0.5 0 0.1 0.05]);
legend boxoff 

%% Checks if state x is at the safety boundary. If it is, returns
%  the optimal safety control to take. 
function [uOpt, onBoundary] = checkAndGetSafetyControl(x, dynSys, grid, vx, uMode, tol)
    % Grab the value at state x from the most recent converged 
    % value function.
    value = eval_u(grid, vx, x);

    % If the value is close to zero, we are close to the safety
    % boundary.
    if value < tol 
        deriv = computeGradients(grid, vx);
        % value of the derivative at that particular state
        current_deriv = eval_u(grid, deriv, x);
        % NOTE: need all 5 arguments (including NaN's) to get 
        % correct optimal control!
        uOpt = dynSys.optCtrl(NaN, x, current_deriv, uMode, NaN); 
        onBoundary = true;
        if iscell(uOpt)
            uOpt = cell2mat(uOpt);
        end
    else
        uOpt = zeros(length(x), 1);
        onBoundary = false;
    end
end

%% Plots dubins car point and heading.
% Inputs:
%   x [vector]  - 3D/4D state of dubins car
% Ouput:
%   c   - handle for figure
function c = plotCar(x, carColor, alpha)
    c{1} = scatter(x(1), x(2), 10, ...
        'MarkerEdgeColor', carColor, 'MarkerFaceColor', carColor, 'MarkerEdgeAlpha', alpha, 'MarkerFaceAlpha', alpha);

    % Plot heading.
    center = x(1:2);

    if length(x) >= 3
        % Rotation matrix.
        R = [cos(x(3)) -sin(x(3)); 
             sin(x(3)) cos(x(3))];
        % Heading pt.
        hpt = [0.5; 0];
        hptRot = R*hpt + center;
        c{2} = plot([center(1) hptRot(1)], [center(2) hptRot(2)], 'Color', carColor, 'LineWidth', 1);
        c{2}.Color(4) = alpha;
    end
end