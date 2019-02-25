%% Run the safety & planning procedure. 

% Clear old figure plotting and variables.
clf 
clc 
clear 

%% Load the current experimental setup and all the parameters.
params = dubinsCameraExp1();
%params = dubinsLidarExp1(); this one is weird...

%% Setup Obstacle Map Generator.
map = OccuMap(params.grid, params.obstacles);

% Compute the first occupancy map.
map.updateMapAndCost(params.initSenseData, params.senseShape);

%% Setup Safety Module.
% Setup safety module object and compute first set.
safety = SafetyModule(params.grid, params.dynSys, params.uMode, ...
    params.dt, params.updateEpsilon, params.warmStart, params.updateMethod);

% Compute the first avoid set based on current sensing.
safety.computeAvoidSet(map.signed_dist_safety);

%% Plot initial conditions, sensing, and safe set.
hold on

% Plot environment, car, and sensing.
plt = Plotter(params.lowEnv, params.upEnv, params.obstacles);
plt.updatePlot(params.xinit, params.xgoal, safety.valueFun, map);
pause(params.dt);

%% Setup Planner.
if strcmp(params.plannerName, 'rrt')
    % Create RRT obj.
    planner = RRT(params.grid, map.occupancy_map_plan, params.maxIter, params.dx);
    % build rrt and get optimal path
    [path, newpath] = planner.replan(params.xinit(1:2), params.xgoal(1:2));
    % (optional) plot optimal path
    plt.plotTraj(path);
    
    % Create PID controller to track RRT trajectory.
    controller = PIDController(params.dynSys, params.dt);
    controller.updatePath(path, 1, newpath);
end

%% Simulation loop.
% Grab initial state.
x = params.dynSys.x;
prevSafeUpdate = 1;
prevPlanUpdate = 1;

for t=1:params.T
    forceUpdate = false;
    
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
        error("Can't run unsupported planner! %s\n", plannerName);
    end
    
    % Check if we are on boundary of safe set. If we are, apply safety 
    % controller instead. 
    [uOpt, onBoundary] = safety.checkAndGetSafetyControl(x);
    if onBoundary
       u = uOpt;
       forceUpdate = true;
    end

    % Apply control to dynamics.
    params.dynSys.updateState(u, params.dt, params.dynSys.x);
    x = params.dynSys.x;
    
    % Get the new sensing region.
    if strcmp(params.senseShape, 'circle')
      senseData = {[x(1);x(2);x(3)], [params.senseRad; params.senseRad]};
    elseif strcmp(params.senseShape, 'camera')
      senseData = {[x(1);x(2);x(3)], [params.senseFOV; params.initialR]};
    elseif strcmp(params.senseShape, 'lidar')
      senseData = {[x(1);x(2);x(3)], [params.senseRad]};
    else
      error('unknown sesnor type');
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
    if dtPlan >= params.planFreq
        if strcmp(params.plannerName, 'rrt') 
            % Update internal variable.
            planner.updateOccuGrid(map.occupancy_map_plan);
            % Replan path.
            [path, newpath] = planner.replan(x(1:2), params.xgoal(1:2));
            % Update path that controller is trying to track.
            controller.updatePath(path, t, newpath);
            % (optional) plot optimal path
            plt.plotTraj(path);
            prevPlanUpdate = t;
        end
    end
    
    % Update plotting.
	plt.updatePlot(x, params.xgoal, safety.valueFun, map);
    
    % Pause based on timestep.
    pause(params.dt);
end

if params.saveOutputData
    % Save out the sequence of value functions.
    valueFunCellArr = safety.valueFunCellArr; 
    lxCellArr = safety.lxCellArr; 
    QSizeCellArr = safety.QSizeCellArr;
    solnTimes = safety.solnTimes;
    fovCellArr = safety.fovCellArr;
    repo = what('safe_navigation');
    savePath = strcat(repo.path, '/data/', params.filename);
    save(savePath, 'valueFunCellArr', 'lxCellArr', 'QSizeCellArr', 'solnTimes', 'fovCellArr');
end