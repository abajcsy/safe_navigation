clear all;
close all;

% Grab the parameters.
params = car3DHJICameraRRT();       
fontsize = 25;

%% Create the initial setup
f = figure;
set(f, 'position', [0, 0, 530, 300]);
hold on;

% Plot the initial state and heading
x = params.xinit;
l(1) = quiver(x(1), x(2), 0, 1, 'ko', 'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'MaxHeadSize', 10.0, 'ShowArrowHead', 'on');

% Plot the camera sensing
theta = pi/6;
c = [0.0,0.0,1.0,0.3];
thresY = 10;
thresX = thresY * tan(theta);
vertices = [x(1) x(2); (x(1) - thresX) (x(2) + thresY); (x(1) + thresX) (x(2) + thresY); x(1) x(2)];
h = fill(vertices(:, 1), vertices(:, 2), c(1:3),'LineStyle','none');
set(h,'facealpha',c(4))

% Plot the goal region
c = [0.0,0.8,0.5,0.5];
xgoal = params.xgoal;
pos = [xgoal(1)-params.goalEps, xgoal(2)-params.goalEps, params.goalEps*2, params.goalEps*2];
rectangle('Position',pos,'Curvature',1.0,'FaceColor',c,'LineStyle','none');
l(2) = scatter(xgoal(1),xgoal(2),[],[0.0,0.8,0.5],'filled');

% Plot the obstacle
obstacles = params.obstacles;
lowObs = obstacles{1}{1};
upObs = obstacles{1}{2};
width = upObs(1) - lowObs(1);
height = upObs(2) - lowObs(2);
obsCoord = [lowObs(1), lowObs(2), width, height];
e = rectangle('Position', obsCoord, ...
    'FaceColor', [0.5,0.5,0.5], 'EdgeColor', [0.2,0.2,0.2]);
l(3) = plot(0, 0, 'Color', [0.5,0.5,0.5], 'LineStyle','-');

% Figure properties
ylim([params.lowEnv(2) params.upEnv(2)]);
xlim([params.lowEnv(1) params.upEnv(1)]);
set(gca, 'xtick', []);
set(gca, 'ytick', []);
xlabel('$p_x$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
ylabel('$p_y$', 'interpreter', 'latex', 'fontsize', fontsize, 'fontweight','bold')
box on;

% Save the figure
saveas(f, '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation/data/Presentation/figures_and_videos/safety_framework/initial_setup.png')
