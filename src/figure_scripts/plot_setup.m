clear all;
close all;

% Grab the parameters.
params = car3DHJICameraRRT();       

% % create plotter.
% plt = Plotter(params.lowEnv, params.upEnv, [], [], ...
%     params.envType, params.obstacles, params.goalEps);
% 
% plt.updatePlot(params.xinit, params.xgoal, [], params.grid, [], ...
%                 [], [], false);


%% Create the initial setup
f = figure;
hold on;

% Plot the initial state and heading
x = params.xinit;
% h1 = plot(x(1), x(2), 'ko','MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
l(1) = quiver(x(1), x(2), 0, 1, 'ko', 'MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k', 'MaxHeadSize', 10.0, 'ShowArrowHead', 'on');

% Plot the goal region
c = [0.0,0.8,0.5,0.5];
xgoal = params.xgoal;
pos = [xgoal(1)-params.goalEps, xgoal(2)-params.goalEps, params.goalEps*2, params.goalEps*2];
rectangle('Position',pos,'Curvature',1.0,'FaceColor',c,'LineStyle','none');
l(2) = scatter(xgoal(1),xgoal(2),[],[0.0,0.8,0.5],'filled');

% Plot the initial safe region
c = [0.0,0.0,1.0,0.3];
R = params.initialR;
pos = [x(1) - R, x(2) - R, 2*R, 2*R];
% s = rectangle('Position',pos,'Curvature',1.0,'FaceColor',c,'LineStyle','none');
s = rectangle('Position',pos,'Curvature',1.0,'LineStyle',':','LineWidth',1.5);
l(3) = plot(0, 0, 'Color', 'k', 'LineStyle',':','LineWidth',1.5);

% Plot the obstacle
obstacles = params.obstacles;
lowObs = obstacles{1}{1};
upObs = obstacles{1}{2};
width = upObs(1) - lowObs(1);
height = upObs(2) - lowObs(2);
obsCoord = [lowObs(1), lowObs(2), width, height];
e = rectangle('Position', obsCoord, ...
    'FaceColor', [0.5,0.5,0.5], 'EdgeColor', [0.2,0.2,0.2]);
l(4) = plot(0, 0, 'Color', [0.5,0.5,0.5], 'LineStyle','-');

% Setup size of figure and tick mark stuff.
ylim([params.lowEnv(2) params.upEnv(2)]);
xlim([params.lowEnv(1) params.upEnv(1)]);
font = 14; line = 0.2;
set(gca,'XTick',[0 5 10], 'fontsize', font-2);
set(gca,'YTick',[0 3.5 7], 'fontsize', font-2);
set(gca,'TickLabelInterpreter','latex')
xlabel('$p_x$', 'Interpreter','latex', 'fontsize', font+2);
ylabel('$p_y$', 'Interpreter','latex', 'fontsize', font+2);
widthMeters = abs(params.lowEnv(1) - params.upEnv(1));
heightMeters = abs(params.lowEnv(2) - params.upEnv(2));
set(gcf, 'Position',  0.5*[100, 100, widthMeters*100*0.6, heightMeters*100*0.6])
box on

% Legends
legendInfo = legend(l, {'Initial state', 'Goal position', '$\mathcal{X}_{init}$', 'Obstacle'},'Interpreter','Latex', 'fontsize', font-2);
% gridLegend(l, 2, ['Initial position and heading', 'Goal position', '$\mathcal{X}_{init}$', 'Obstacle'],'Interpreter','Latex');
legend boxoff 

% Save the figure
saveas(f, './figures/initial_setup.fig')
saveas(f, './figures/initial_setup.png')
saveas(f, './figures/initial_setup.pdf')
