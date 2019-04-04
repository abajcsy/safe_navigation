clear all;
close all;

% Grab the parameters.
params = car3DHJICameraRRT();       

%% Create the camera sensing
f = figure;
hold on;

% Plot the initial state and heading
x = params.xinit;
obstacles = params.obstacles;
lowObs = obstacles{1}{1};
upObs = obstacles{1}{2};
% h1 = plot(x(1), x(2), 'ko','MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
% l(1) = quiver(x(1), x(2), 0, 1, 'ko', 'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'MaxHeadSize', 10.0, 'ShowArrowHead', 'on');

% Plot the camera sensing
theta = pi/6;
c = [0.0,0.0,1.0,0.3];
thresY = 10;
thresX = thresY * tan(theta);
vertices = [x(1) x(2); (x(1) - thresX) (x(2) + thresY); (x(1) + thresX) (x(2) + thresY); x(1) x(2)];
h = fill(vertices(:, 1), vertices(:, 2), c(1:3),'LineStyle','none');
set(h,'facealpha',c(4))

% Plot the initial state and heading
x = params.xinit;
l(1) = quiver(x(1), x(2), 0, 1, 'ko', 'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'MaxHeadSize', 10.0, 'ShowArrowHead', 'on');

% Plot the obstacle
width = upObs(1) - lowObs(1);
height = upObs(2) - lowObs(2);
obsCoord = [lowObs(1), lowObs(2), width, height];
e = rectangle('Position', obsCoord, ...
    'FaceColor', [0.5,0.5,0.5], 'EdgeColor', [0.2,0.2,0.2]);
l(3) = plot(0, 0, 'Color', [0.5,0.5,0.5], 'LineStyle','-');

% Setup size of figure and tick mark stuff.
ylim([params.lowEnv(2) params.upEnv(2)]);
xlim([params.lowEnv(1) params.upEnv(1)]);
font = 14; line = 0.2;
set(gca,'XTick',[ ]);
set(gca,'YTick',[ ]);
widthMeters = abs(params.lowEnv(1) - params.upEnv(1));
heightMeters = abs(params.lowEnv(2) - params.upEnv(2));
set(gcf, 'Position',  0.25*[100, 100, widthMeters*100*0.6, heightMeters*100*0.6])
box on

% Save the figure
saveas(f, './figures/initial_setup/camera_sensing.fig')
saveas(f, './figures/initial_setup/camera_sensing.png')
saveas(f, './figures/initial_setup/camera_sensing.pdf')
