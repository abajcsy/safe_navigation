function plane_warm_start()
    %% Grid
    grid_min = [-5; -5; -3.1415]; % Lower corner of computation domain
    grid_max = [5; 5; 3.1415];    % Upper corner of computation domain
    N = [51; 51; 11];         % Number of grid points per dimension
    pdDims = 3;               % 3rd dimension is periodic
    g = createGrid(grid_min, grid_max, N, pdDims);

    %% time vector
    t0 = 0;
    tMax = 5.0;
    dt = 0.025;
    tau = t0:dt:tMax;

    %% problem parameters
    wMax =  1;
    xinit = [-2, -2, 0];
    vrange = [0.9, 1.1];
    dMax = [0.0, 0.0, 0];
    % Define dynamic system
    dCar = Plane(xinit, wMax, vrange, dMax); 

    %% Control Parameters
    uMode = 'max';
    dMode = 'min';
    % Put grid and dynamic systems into schemeData
    schemeData.grid = g;
    schemeData.dynSys = dCar;
    schemeData.accuracy = 'veryHigh'; %set accuracy
    schemeData.uMode = uMode;
    schemeData.dMode = dMode;

    %% target set
    R = [3, 2.99, 2.95, 2.85, 2.5, 1.5]; % obstacle should be shrinking
    data0 = shapeCylinder(g, 3, [0, 0, 0], R(1));
    lxOld = data0;

    %% Get base file path 
    filepath = mfilename('fullpath');
    tokens = split(filepath, "/");
    cellArray = join(tokens(1:length(tokens) - 1), "/");
    base_path = cellArray{1};
    
    %% Set HJIPDE hyper parameters
    HJIextraArgs.targets = data0;
    HJIextraArgs.visualize.valueSet = 1;
    HJIextraArgs.visualize.initialValueSet = 1;
    HJIextraArgs.visualize.figNum = 1; %set figure number
    HJIextraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
    HJIextraArgs.visualize.viewGrid = false;
    HJIextraArgs.visualize.plotData.plotDims = [1 1 0]; % Plot x, y
    HJIextraArgs.visualize.plotData.projpt = [0]; % Project at theta = 0
    HJIextraArgs.fig_num = 1; %set figure number
    HJIextraArgs.deleteLastPlot = true; %delete previous plot as you update
    HJIextraArgs.stopConverge = true;
    HJIextraArgs.convergeThreshold = 1e-3;
    
    %% Solve and plot HJIPDE warm start initialization
    f = figure(1); clf;
    [data, ~, ~] = HJIPDE_solve_warm(data0, [], data0, tau, schemeData, 'minVWithL', true, HJIextraArgs);
    save(sprintf("%s/../outputs/plane/warm_start/t1.mat", base_path), 'data');
    saveas(f, sprintf('%s/../outputs/plane/warm_start/t1_HJIPDE.png', base_path));
    % plot zero set
    f = figure(1); clf;
    [grid2D, data2D] = proj(g, data, [0 0 1], [0]);
    h = visSetIm(grid2D, data2D, 'r');
    saveas(f, sprintf('%s/../outputs/plane/warm_start/t1_zero_set.fig', base_path));
    
    %% Solve and plot HJIPDE local Q for each shrinking radius
    for t = 2:length(R)
        %% Set up HJIPDE solve local Q parameters
        radius = R(t);
        lx = shapeCylinder(g, 3, [0 0 -2], radius);
        data0 = data(:, :, :, end); 
        HJIextraArgs.targets = lx;
        
        %% Solve annd plot HJIPDE warm per timestamp
        f = figure(1); clf;
        [data, ~, ~] = HJIPDE_solve_warm(data0, lxOld, lx, tau, schemeData, 'minVWithL', true, HJIextraArgs);
        saveas(f, sprintf('%s/../outputs/plane/warm_start/t%d_HJIPDE.png', base_path, t));
        save(sprintf('%s/../outputs/plane/warm_start/t%d.mat', base_path, t), 'data');
        % plot zero set
        f = figure(1); clf;
        [grid2D, data2D] = proj(g, data, [0 0 1], [0]);    
        visSetIm(grid2D, data2D, 'r');
        lxOld = lx;
        saveas(f, sprintf('%s/../outputs/plane/warm_start/t%d_zero_set.png', base_path, t));
    end 
end