%% Run the safety & planning procedure. 

% Clear old figure plotting and variables.
clf 
clc 
clear 

%% Load the current experimental setup and all the parameters.
params = car3DWarmCameraRRT();

%% Setup Obstacle Map Generator.
extraArgs.obstacles = params.obstacles;
map = OccuMap(params.grid, params.envType, extraArgs);

% Compute the first occupancy map.
map.updateMapAndCost(params.initSenseData, params.senseShape);

%% Setup Safety Module.
% Setup safety module object and compute first set.
safety = SafetyModule(params.grid, params.dynSys, params.uMode, params.dMode, ...
    params.dt, params.updateEpsilon, params.warmStart, params.envType, params.updateMethod, params.tMax, params.initialR);

% Compute the first avoid set based on current sensing.
safety.computeAvoidSet(map.signed_dist_safety, 1);

%% Plot initial conditions, sensing, and safe set.

% If we want to visualize the simulation, plot current state.
if params.visualize
    hold on

    % Plot environment, car, and sensing.
    plt = Plotter(params.lowEnv, params.upEnv, params.lowEnv, params.upEnv, params.envType, params.obstacles, params.goalEps);
    plt.updatePlot(params.xinit, params.xgoal, safety.valueFun, params.grid, map, [], [], []);
    pause(params.dt);
end
    
%% Setup Planner.
if strcmp(params.plannerName, 'rrt')
    % Create RRT obj.
    planner = RRT(params.grid, map.occupancy_map_plan, params.maxIter, ...
        params.dx, params.goalEps);
    % build rrt and get optimal path
    [path, newpath] = planner.replan(params.xinit(1:2), params.xgoal(1:2));
    
    if params.visualize
        % (optional) plot optimal path
        plt.plotTraj(path);
    end
    
    % Create PID controller to track RRT trajectory.
    controller = PIDController(params.dynSys, params.dt);
    controller.updatePath(path, 1, newpath);
end

%% Save out actual states and planned trajectory.
if params.saveOutputData
    states = {};
    paths = {};
end

%% Simulation loop.
% Grab initial state.
x = params.dynSys.x;
prevSafeUpdate = 1;
prevPlanUpdate = 1;

for t=1:params.T
    % Do we want to force a safety update?
    forceUpdate = false;
    % Do we want to force a replan?
    forcedReplan = false;
    
    % If we are saving, record the current state and plan
    if params.saveOutputData
        states{end+1} = x;
        paths{end+1} = path;
    end
    
    % If we are close enough to the goal, stop simulation.
    if norm(x(1:2) - params.xgoal(1:2)) < params.goalEps
        break;
    end
    
    % Switch control and planning based on planner.
    if strcmp(params.plannerName, 'hand')
        u = getHandCodedControl(t);
    elseif strcmp(params.plannerName, 'rrt')
        u = controller.getControl(t, x);
    else
        error('Cannot run unsupported planner! %s\n', plannerName);
    end
    
    % Check if we are on boundary of safe set. If we are, apply safety 
    % controller instead. 
    [uOpt, onBoundary] = safety.checkAndGetSafetyControl(x, params.safetyTol);
    if onBoundary
       u = uOpt;
       forcedReplan = true;
    end

    % Apply control to dynamics.
    params.dynSys.updateState(u, params.dt, params.dynSys.x);
    x = params.dynSys.x;
    
    % Get the new sensing region.
    if strcmp(params.senseShape, 'circle')
      senseData = {[x(1);x(2);x(3)], [params.senseRad; params.senseRad]};
    elseif strcmp(params.senseShape, 'camera')
      senseData = {[x(1);x(2);x(3)], [params.senseFOV; params.initialR; params.farPlane]};
    elseif strcmp(params.senseShape, 'lidar')
      senseData = {[x(1);x(2);x(3)], [params.senseRad]};
    else
      error('unknown sensor type');
    end  
    
    % Update occupancy map, cost function, and the avoid set.
    map.updateMapAndCost(senseData, params.senseShape);
    
    % Time duration since last update of safe set and planner.
    dtSafe = t - prevSafeUpdate;
    dtPlan = t - prevPlanUpdate;
    
    % If need to update the safety set, do so.  
    if dtSafe >= params.safetyFreq || forceUpdate
        safety.computeAvoidSet(map.signed_dist_safety);
        prevSafeUpdate = t; 
    end
    
    % If need to update the planner, do so.
    if dtPlan >= params.planFreq || forcedReplan
        if strcmp(params.plannerName, 'rrt') 
            % Update internal variable.
            planner.updateOccuGrid(map.occupancy_map_plan);
            % Replan path.
            [path, newpath] = planner.replan(x(1:2), params.xgoal(1:2));
            % Update path that controller is trying to track.
            controller.updatePath(path, t, newpath);
            if params.visualize
                % (optional) plot optimal path
                plt.plotTraj(path);
            end
            prevPlanUpdate = t;
        end
    end
    
    if params.visualize
        % Update plotting.
        %plt.updatePlot(x, params.xgoal, safety.valueFun, params.grid, map, [], [], []);
        % Pause based on timestep.
        pause(params.dt);
    end
end

% Save out relevant data.
if params.saveOutputData
    valueFunCellArr = safety.valueFunCellArr; 
    lxCellArr = safety.lxCellArr; 
    QSizeCellArr = safety.QSizeCellArr;
    solnTimes = safety.solnTimes;
    fovCellArr = safety.fovCellArr;
    repo = what('safe_navigation');
    savePath = strcat(repo.path, '/data/', params.filename);
    save(savePath, 'valueFunCellArr', 'lxCellArr', 'QSizeCellArr', ...
        'solnTimes', 'fovCellArr', 'states', 'paths');
end