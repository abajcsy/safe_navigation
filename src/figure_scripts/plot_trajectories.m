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
    lidar_color = turquoise;
    % extract points where safe control was applied.
    li_safe_indicies = find(lidar_appliedUOpt == true);
    lidar_xSafe = lidar_xtraj(li_safe_indicies);
    lidar_ySafe = lidar_ytraj(li_safe_indicies);

    % Load Camera.
    filenameR = curr_files{2}; 
    filePathR = strcat(traj_path, filenameR);
    load(filePathR);
    camera_xtraj = xtraj;
    camera_ytraj = ytraj;
    camera_thetatraj = thetatraj;
    camera_appliedUOpt = appliedUOpt;
    camera_color = orange;
    % extract points where safe control was applied.
    cam_safe_indicies = find(camera_appliedUOpt == true);
    camera_xSafe = camera_xtraj(cam_safe_indicies);
    camera_ySafe = camera_ytraj(cam_safe_indicies);

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
%     idx = 1;
%     prevIdx = 100;
%     if ~isempty(li_safe_indicies)
%         safetyIdx = li_safe_indicies(1);
%     end
%     si = 1;
%     amount = length(lidar_xtraj)/3.;
%     
%     for i=1:10:length(lidar_xtraj)
% %         if i < amount
% %             alpha = 0.3;
% %         elseif i >= amount && i < amount*2
% %             alpha = 0.7;
% %         else
% %             alpha = 1;
% %         end
%         alpha = prevIdx/length(lidar_xtraj);
%         idx = idx + 1;
%         if idx > 10
%            idx = 1;
%            prevIdx = i;
%         end
%         lih = scatter(lidar_xtraj(i), lidar_ytraj(i), 5, ...
%            'MarkerFaceColor', lidar_color, 'MarkerEdgeColor', lidar_color, 'MarkerFaceAlpha',alpha, 'MarkerEdgeAlpha',alpha); 
%         %lih = plot(lidar_xtraj(i:i+1), lidar_ytraj(i:i+1), '-', 'MarkerEdgeColor', 'none', 'Color', lidar_color, 'LineWidth', 5); 
%         %lih.Color(4) = alpha;
% %         if ~isempty(li_safe_indicies)
% %             safetyIdx = li_safe_indicies(si);
% %             if i == safetyIdx
% %                 scatter(lidar_xSafe(si), lidar_ySafe(si), 5, ...
% %                     'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r', ...
% %                     'MarkerFaceAlpha',alpha, 'MarkerEdgeAlpha',alpha);
% %                 si = si+1;
% %             end
% %         end
%     end
    lih = plot(lidar_xtraj, lidar_ytraj, '-', 'Color', lidar_color, 'LineWidth', 3); 
    % Plot points where safety was applied.
    plot(lidar_xSafe, lidar_ySafe, 'ro', 'MarkerSize', 3,'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
    % Plot car at final point
    %plotCar(obj, [spline_xtraj(end); spline_ytraj(end); spline_thetatraj(end)]);

    %% Plot Camera.
%     idx = 1;
%     prevIdx = 1;
%     safetyIdx = cam_safe_indicies(1);
%     si = 1;
%     for i=10:length(camera_xtraj)
%         alpha = prevIdx/length(camera_xtraj);
%         idx = idx + 1;
%         if idx > 10
%             idx = 1;
%             prevIdx = i;
%         end
%         camh = scatter(camera_xtraj(i), camera_ytraj(i), 5, ...
%             'MarkerFaceColor', camera_color, 'MarkerEdgeColor', camera_color, 'MarkerFaceAlpha',alpha, 'MarkerEdgeAlpha',alpha); 
% 
%         %alpha = i/length(camera_xtraj);
%         %camh = scatter(camera_xtraj(i), camera_ytraj(i), 5, ...
%         %    'MarkerFaceColor', camera_color, 'MarkerEdgeColor', camera_color, 'MarkerFaceAlpha',alpha, 'MarkerEdgeAlpha',alpha); 
% %         safetyIdx = cam_safe_indicies(si);
% %         if i == safetyIdx
% %             scatter(camera_xSafe(si), camera_ySafe(si), 5, ...
% %                 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r', ...
% %                 'MarkerFaceAlpha',alpha, 'MarkerEdgeAlpha',alpha);
% %             si = si+1;
% %             if si > length(cam_safe_indicies)
% %                 si = length(cam_safe_indicies);
% %             end
% %         end
%     end
    camh = plot(camera_xtraj, camera_ytraj, '-', 'Color', camera_color, 'LineWidth', 3);
    % Plot points where safety was applied.
    plot(camera_xSafe, camera_ySafe, 'ro', 'MarkerSize', 3,'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'r');
    % Plot car at final point
    %plotCar(obj, [rrt_xtraj(end); rrt_ytraj(end); rrt_thetatraj(end)]);

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

%lih_bar = plot([0,0], [1,1], '-', 'Color', lidar_color, 'LineWidth', 3);
%camh_bar = plot([0,0], [1,1], '-', 'Color', camera_color, 'LineWidth', 3);
%legend([lih_bar camh_bar],{'LiDAR','Camera'}, 'Interpreter','latex', 'fontsize', 10, 'Orientation','horizontal', 'Position', [0.5 0 0.1 0.05]);
legend([lih camh],{'LiDAR','Camera'}, 'Interpreter','latex', 'fontsize', 10, 'Orientation','horizontal', 'Position', [0.5 0 0.1 0.05]);
legend boxoff 