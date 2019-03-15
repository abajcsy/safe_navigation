% Grab the parameters.
params = car3DHJICameraRRT();       

% create plotter.
plt = Plotter(params.lowEnv, params.upEnv, [], [], ...
    params.envType, params.obstacles, params.goalEps);

plt.updatePlot(params.xinit, params.xgoal, [], params.grid, [], ...
                [], [], false);