function slam_listener_node()
    clear all
    close all
    
    useTurtlebot = false; 
    if useTurtlebot
        % If receiving SLAM map from turtlebot, set the ROS_MASTER_URI to
        % use the turtlebot IP address.
        setenv('ROS_MASTER_URI','http://128.32.38.83:11311')
    else
        setenv('ROS_MASTER_URI','http://localhost:11311')
    end
    
    rosinit 
    
    % Subscriber that listens to occupancy grid.
    occuMapMsgType = 'nav_msgs/OccupancyGrid';
    occuMapSub = rossubscriber('/map', occuMapMsgType, @mapCallback);
    
    % Load up all the experiment parameters.
    params = car4DLocalQCameraNN();
    
    hold on;
    figure(1);
    % Spin and let everything go.
    rate = rosrate(100);
    reset(rate);
    while true
        waitfor(rate);
    end
end

function mapCallback(~, msg)
    res = double(msg.Info.Resolution);
    numY = msg.Info.Height;
    numX = msg.Info.Width;
    realHeight = double(numY)*double(res);
    realWidth = double(numX)*double(res);
    origin = [double(msg.Info.Origin.Position.X), double(msg.Info.Origin.Position.Y)];

    fprintf('Got SLAM occupancy map message with info:\n');
    fprintf('     realH: %f, # rows: %d\n', realHeight, numY);
    fprintf('     realW: %f, # cols: %d\n', realWidth, numX);
    fprintf('     origin: (%d, %d)\n', origin(1), origin(2));
    
    % Grab the SLAM-generated occupancy map.
    % Convention for SLAM map values:
    %   > 0     -- sensed obstacle
    %   = 0     -- sensed free-space
    %   < 0     -- unsensed map space.
    slamOccuMap = double(reshape(msg.Data, [numX,numY])); 
    
    % Convert to convention with (+1) to be free-space and (-1) to be
    % obstacle.
    slamOccuMap(find(slamOccuMap > 0)) = -1; 
    slamOccuMap(find(slamOccuMap == 0)) = 1;
    
    % Grid stuff.
    sideLength = 17;
    lowEnv = [0;-sideLength/2];
    upEnv = [sideLength;sideLength/2]; %[realWidth+origin(1);realHeight+origin(2)];
    gridLow = [lowEnv;-pi;-0.1];
    gridUp = [upEnv;pi;0.7];
    N = [71;71;21;11]; 
    pdDims = 3;
    grid = createGrid(gridLow, gridUp, N, pdDims);
    
    % Grab the 2D grid.
    [grid2D, ~] = proj(grid, grid.xs{1}, [0 0 1 1], [0 0]);
        mapBounds = [origin(1), origin(2), origin(1) + realWidth, origin(2) + realHeight];
    trueOccuMap = generate_computation_grid(grid2D, slamOccuMap, res, mapBounds);

    % Plot (negate so we see obstacle part as filled in).
    clf;
    contourf(grid2D.xs{1}, grid2D.xs{2}, -trueOccuMap, [0,0]);
    
    % ---------gets you the signed distance function -----------
    % We will use the FMM code to get the signed distance function. 
    % Since the FMM code works only on 2D, we will take a slice of 
    % the grid, compute FMM, and then project it back to a 3D array.
    unionL_2D_FMM = compute_fmm_map(grid2D, trueOccuMap);

    if grid.dim == 3
        signedDistSafety = repmat(unionL_2D_FMM, 1, 1, grid.N(3));
    else
        signedDistSafety = repmat(unionL_2D_FMM, 1, 1, grid.N(3), grid.N(4));
    end
    
    %---call the safety function with signed distance---
    data0 = signedDistSafety; % this is with no warm start though
    lxOld = [];
    lCurr = signedDistSafety;
    t0 = 0;
    dt = 0.05;
    tMax = 20;
    timeDisc = t0:dt:tMax;
    
    %% Dynamical System Params.
    xinit = [19.05; 31.4; 0.; 0.];
    wMax = 1.1;          % maxangular control
    aRange = [-0.4, 0.4];    % acceleration control range
    vRange = [0.0, 0.6];    % speed range

    % Define dynamic system. 
    dynSys = Plane4D(xinit, wMax, aRange, vRange);
    


    schemeData.grid = grid;
    schemeData.dynSys = dynSys;
    schemeData.accuracy = 'high'; % Set accuracy.
    schemeData.uMode = 'max';
    
    HJIextraArgs.ignoreBoundary = 0;
    HJIextraArgs.stopConverge = 1;
    HJIextraArgs.convergeThreshold = 0.01;
    HJIextraArgs.targets = lCurr;
    
    minWith = 'minVWithL';
    
    firstWarmStart = false;
    
    firstHJIextraArgs = HJIextraArgs;
    firstHJIextraArgs.stopConverge = 1;
    firstHJIextraArgs.convergeThreshold = 0.01;
    firstHJIextraArgs.visualize.plotData.plotDims = [1 1 0 0];
    firstHJIextraArgs.visualize.plotData.projpt = [0 0.5];
    firstHJIextraArgs.visualize.valueSet = 1;
    
    [dataOut, tau, extraOuts] = ...
    HJIPDE_solve_warm(data0, lxOld, lCurr, ...
      timeDisc, schemeData, minWith, ...
      firstWarmStart, firstHJIextraArgs);
    
  % --------- grab the final safe set and visualize----------
  valeuFun = dataOut(:,:,:,:,end);
  contourf(grid, grid, valueFun);
end