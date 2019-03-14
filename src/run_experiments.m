%% Run all the experiments.

function run_experiments()
    % Clear old figure plotting and variables.
    clf 
    clc 
    clear 
    close all
    
    % Compile FMM c-code.
    whatRepo = what('safe_navigation');
    repo = whatRepo.path;
    %repo = '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation';
    filename = 'mexEikonalFMM.cpp';
    cppPath = strcat(repo, '/src/fmm/cversion/', filename);
    mex(cppPath);
 
    % Setup all function handles to experimental setup.
    rrtExperiments = {@car3DLocalQCameraRRT, ...
                        @car3DWarmCameraRRT, ...
                        @car3DHJICameraRRT, ...
                        @car3DLocalQLidarRRT, ...
                        @car3DWarmLidarRRT, ...
                        @car3DHJILidarRRT};
   
    splineExperiments = {@car3DLocalQLidarSpline, ...
                        @car3DWarmLidarSpline, ...
                        @car3DHJILidarSpline, ...
                        @car3DLocalQCameraSpline, ...
                        @car3DWarmCameraSpline, ...
                        @car3DHJICameraSpline};
                   
    experiments = {@car3DLocalQCameraSpline};
    
    % Simulate each experiment.
    for i=1:length(experiments)
        experimentFun = experiments{i};
        runExperiment(experimentFun);
    end
end

%% Runs one experiment. 
% Inputs:
%   experimentFun   -- function handle to experimental parameters.
function runExperiment(experimentFun)
    %% Load the current experimental setup and all the parameters.
    params = experimentFun();

    %% Setup Obstacle Map Generator.
    extraArgs.obstacles = params.obstacles;
    map = OccuMap(params.grid, params.envType, extraArgs);

    % Compute the first occupancy map.
    map.updateMapAndCost(params.initSenseData, params.senseShape);

    %% Setup Safety Module.
    if params.useSafety
        % Setup safety module object and compute first set.
        safety = SafetyModule(params.grid, params.dynSys, params.uMode, params.dMode, ...
            params.dt, params.updateEpsilon, params.warmStart, params.envType, ...
            params.updateMethod, params.tMax);

        % Compute the first avoid set based on current sensing.
        safety.computeAvoidSet(map.signed_dist_safety, 1);
    else
        % No value function :(
        safety.valueFun = [];
    end

    %% Setup Planner.
    if strcmp(params.plannerName, 'rrt')
        % Create RRT obj.
        planner = RRT(params.grid, map.occupancy_map_plan, params.maxIter, ...
            params.dx, params.goalEps);
        % build rrt and get optimal path
        [path, newpath] = planner.replan(params.xinit(1:2), params.xgoal(1:2));
        
        % Create PID controller to track RRT trajectory.
        controller = PIDController(params.dynSys, params.dt);
        controller.updatePath(path, 1, newpath);
    elseif strcmp(params.plannerName, 'hand')
        % Hand-coded plan has no path.
        path = {};
    elseif strcmp(params.plannerName, 'spline')
        % Create spline planner subscriber node.        
        planner = SplinePlannerNode();
        
        % Replan. 
        ucurr = [1,0];
        [path, pathCtrls, newpath] = planner.replan(params.xinit, ...
            ucurr,map.occupancy_map_plan,params.xgoal);
        
        % Create PID controller to track RRT trajectory.
        controller = PIDController(params.dynSys, params.dt);
        controller.updatePath(path, 1, newpath);
    end

    %% Plot initial conditions, sensing, and safe set.

    % If we want to visualize the simulation, plot current state.
    if params.visualize
        hold on

        % Plot environment, car, and sensing.
        plt = Plotter(params.lowEnv, params.upEnv, ...
            map.boundLow, map.boundUp, params.envType, ...
            params.obstacles, params.goalEps);
        plt.updatePlot(params.xinit, params.xgoal, safety.valueFun, ...
            map.grid, map.gFMM, map.occupancy_map_safety, path, false);
        pause(params.dt);
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
        % Did we use the optimal control? (used by plotting)
        usedUOpt = false;

        % If we are saving, record the current state and plan
        if params.saveOutputData
            states{end+1} = x;
            paths{end+1} = path;
        end

        % If we are close enough to the goal, stop simulation.
        if norm(x(1:2) - params.xgoal(1:2)) < params.goalEps
            fprintf('Reached the goal!');
            break;
        end

        % Switch control and planning based on planner.
        if strcmp(params.plannerName, 'hand')
            u = getHandCodedControl(t);
        elseif strcmp(params.plannerName, 'rrt') || strcmp(params.plannerName, 'spline')
            u = controller.getControl(t, x);
        else
            error('Cannot run unsupported planner! %s\n', plannerName);
        end

        if params.useSafety
            % Check if we are on boundary of safe set. If we are, apply safety 
            % controller instead. 
            [uOpt, onBoundary] = safety.checkAndGetSafetyControl(x, params.safetyTol);
            if onBoundary
               u = uOpt;
               fprintf('optimal controller: [%f, %f]\n', u(1), u(2));
               forcedReplan = true;
               usedUOpt  = true;
            end
        end

        % Grab random disturbance. 
        d = params.dynSys.uniformDstb();
        
        % Apply control to dynamics.
        params.dynSys.updateState(u, params.dt, params.dynSys.x, d);
        x = params.dynSys.x;

        % Get the new sensing region.
        if strcmp(params.senseShape, 'circle')
          senseData = {[x(1);x(2);x(3)], [params.senseRad; params.senseRad]};
        elseif strcmp(params.senseShape, 'camera')
          senseData = {[x(1);x(2);x(3)], [params.senseFOV; params.initialR; params.farPlane]};
        elseif strcmp(params.senseShape, 'lidar')
          senseData = {[x(1);x(2);x(3)], [params.senseRad]};
        else
          error('Unknown sensor type: %s\n', params.senseShape);
        end  

        % Update occupancy map, cost function, and the avoid set.
        map.updateMapAndCost(senseData, params.senseShape);

        % Time duration since last update of safe set and planner.
        dtSafe = t - prevSafeUpdate;
        dtPlan = t - prevPlanUpdate;

        % If need to update the safety set, do so.  
        if params.useSafety
            if dtSafe >= params.safetyFreq || forceUpdate
                safety.computeAvoidSet(map.signed_dist_safety, t);
                prevSafeUpdate = t;
            end
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
                prevPlanUpdate = t;
            elseif strcmp(params.plannerName, 'spline')
                [path, ~, newpath] = planner.replan(x, u, ...
                    map.occupancy_map_plan, params.xgoal);
                % Update path that controller is trying to track.
                controller.updatePath(path, t, newpath);
                prevPlanUpdate = t;
            end
        end

        if params.visualize
%             plt.updateOccuMapSafe(map.gFMM, sign(safety.lCurr(:,:,1)));
%             plt.updateOccuMapPlan(map.gFMM, map.occupancy_map_plan);
            
            % Update plotting.
            plt.updatePlot(x, params.xgoal, safety.valueFun, ...
                map.grid, map.gFMM, map.occupancy_map_safety, path, usedUOpt);
            
            % Pause based on timestep.
            %pause(params.dt);
        end
    end

    % Save out relevant data.
    if params.saveOutputData
        safeOccuMaps = map.occuMapSafeCellArr;
        planOccuMaps = map.occuMapPlanCellArr;
        repo = what('safe_navigation');
        savePath = strcat(repo.path, '/data/', params.filename);
        
        % Save out safety analysis metrics if we were computing safe sets.
        if params.useSafety
            valueFunCellArr = safety.valueFunCellArr; 
            lxCellArr = safety.lxCellArr; 
            QSizeCellArr = safety.QSizeCellArr;
            solnTimes = safety.solnTimes;
            updateTimeArr = safety.updateTimeArr;
            save(savePath, 'valueFunCellArr', 'lxCellArr', 'QSizeCellArr', ...
                'solnTimes', 'safeOccuMaps', 'planOccuMaps', 'updateTimeArr', ...
                'states', 'paths', 'params');
        else
            save(savePath, 'safeOccuMaps', 'planOccuMap', 'params');
        end
    end
end
