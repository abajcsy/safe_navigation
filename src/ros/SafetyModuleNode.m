classdef SafetyModuleNode < handle
    %SAFETYMODULENODE Encapsulates all the safety computations 
    % and verification of the neural network plans with ROS communication.
    
    properties
        % Subscribers and publishers.
        occuMapSub
        planSub
        verifiedPub
        
        % All the info about our experimental setup.
        params
        
        % Safety data.
        safety
        
        % Occupancy map data.
        map             % (obj) OccuMap object that operates on occupancy grids.
        sbpdOccuMap     % (arr) Entire SBPD ground-truth occupancy map data.
        trueOccuMap     % (arr) (Localized) Ground-truth occupancy map data.
        res             % (float) Ground-truth dx resolution. 
        origin          % (arr) Ground-truth lower x,y coordinate.
        
        % Figure data.
        plotter
    end
    
    methods
        %% Constructor
        function obj = SafetyModuleNode()
            clf
            rmpath(genpath('/home/somilb/Documents/MATLAB/helperOC/'));
            addpath(genpath('/home/somilb/Documents/MATLAB/helperOC_dev/'));
            addpath(genpath('/home/somilb/Documents/Projects/visual_mpc/safe_navigation_ws/src/matlab_gen/'));
            addpath(genpath('/home/somilb/Documents/Projects/safe_navigation/'));
            
            % Create a new node.
            rosinit 
            
            % Load up all the experimental parameters.
            obj.params = car4DLocalQCameraNN();
            
            if obj.params.loadTrueOccuMaps 
                % Load the saved ground-truth occupancy maps.
                fprintf('Loading ground-truth SBPD occupancy map.\n');
                repo = what('safe_navigation');
                sbpdPath = strcat(repo.path, '/data/sbpdOccuMap.mat'); 
                truePath = strcat(repo.path, '/data/trueOccuMap.mat'); 
                load(sbpdPath);
                load(truePath);
                % note: need to transpose to get correct coordinates.
                obj.sbpdOccuMap = sbpdOccuMap;
                obj.trueOccuMap = trueOccuMap; 
            else
                % Subscriber that listens to occupancy grid.
                occuMapMsgType = 'nav_msgs/OccupancyGrid';
                obj.occuMapSub = rossubscriber(obj.params.occuMapTopicName, occuMapMsgType);
                
                % Grab the ground-truth occupancy map from Python.
                fprintf('Listening over ROS to ground-truth SBPD occupancy map.\n');
                msg = receive(obj.occuMapSub, 10);
                obj.saveGroundTruthOccupancyGrids(msg);
            end
            
            % Setup safety module object and compute first set.
            obj.safety = SafetyModule(obj.params.grid, obj.params.dynSys, ...
                            obj.params.uMode, obj.params.dt, ...
                            obj.params.updateEpsilon, obj.params.warmStart, ...
                            obj.params.envType, obj.params.updateMethod, obj.params.tMax);
            
            % Create the map and plotting objects.
            extraArgs.occuMap = obj.trueOccuMap;
            obj.map = OccuMap(obj.params.grid, obj.params.envType, extraArgs);
            obj.plotter = Plotter(obj.params.lowEnv, obj.params.upEnv, ...
                obj.map.boundLow, obj.map.boundUp, obj.params.envType, obj.trueOccuMap);
            
            % Compute the first avoid set based on initial safe set.
            obj.map.updateMapAndCost(obj.params.initSenseData, obj.params.senseShape);
            obj.safety.computeAvoidSet(obj.map.signed_dist_safety, 1);
            
            % Update the plotting.
            obj.plotter.updatePlot(obj.params.xinit, obj.params.xgoal, obj.safety.valueFun, ...
                obj.map.grid, obj.map.gFMM, obj.map.occupancy_map_safety, [], false);
            
            % Setup ROS pub/sub for trajectories to verify.
            obj.registerCallbacks();
            
            % Spin and let everything go.
            rate = rosrate(100);
            reset(rate);
            while true
                waitfor(rate);
            end
        end
        
        %% ------------ ROS Subscriber/Publisher Functions ------------- %%
                        
        % Create all the publishers and subscribers.
        function registerCallbacks(obj)
            % Publisher that publishes out verified trajectory.
            verifiedMsgType = 'safe_navigation_msgs/Trajectory';
            obj.verifiedPub = rospublisher(obj.params.verifiedTopicName,verifiedMsgType);

            % Subscriber that listens to planned trajectory
            planMsgType = 'safe_navigation_msgs/Trajectory';
            obj.planSub = rossubscriber(obj.params.planTopicName, planMsgType, @obj.verifyPlanCallback);
            pause(2); 
        end
        
        %% ------------------- ROS Msg Util Functions -------------------- %
        % Converts from array to trajectory message
        function msg = toTrajMsg(obj, x0, controls)
            msg = rosmessage('safe_navigation_msgs/Trajectory');
            
            % send out just initial condition
            initState = rosmessage('safe_navigation_msgs/State');
            initState.X = x0;
            
            msg.States = [initState];
            msg.Controls = [];
            for i=1:length(controls)
                control = rosmessage('safe_navigation_msgs/Control');
                control.U = controls{i};
                msg.Controls(end+1) = control;
            end
        end
        
        % Converts from 1D OccupancyGrid ROS message to 2D grid array.
        function saveGroundTruthOccupancyGrids(obj, msg)
            obj.res = double(msg.Info.Resolution);
            r = msg.Info.Height;
            c = msg.Info.Width;
            realHeight = double(r*obj.res);
            realWidth = double(c*obj.res);
            obj.origin = [double(msg.Info.Origin.Position.X), double(msg.Info.Origin.Position.Y)];
            
            % Save the entire SBPD ground-truth occupancy map
            obj.sbpdOccuMap = double(reshape(msg.Data, [r,c])); 
            
            fprintf('Received ground truth map with: \n');
            fprintf('    height: %f, # rows: %d\n', realHeight, r);
            fprintf('    width: %f, # cols: %d\n', realWidth, c);
            fprintf('    res: %f\n', obj.res);
            
            % Extract just the ground-truth map relative to our 
            % compute grid.
            if obj.params.grid.dim == 4
                [grid2D, ~] = proj(obj.params.grid, obj.params.grid.xs{1}, [0 0 1 1], [0 0]);
            else
                error('I am not programmed to use safety module with %D system.\n', obj.params.grid.dim);
            end
            mapBounds = [obj.origin(1), obj.origin(2), ...
                obj.origin(1) + realWidth, obj.origin(2) + realHeight];
            obj.trueOccuMap = ...
                generate_computation_grid(grid2D, obj.sbpdOccuMap', ...
                obj.res, mapBounds);
            %figure, imshow(flipud(obj.trueOccuMap'));
        end
        
        %% ------------------- ROS Callbacks  -------------------- %
        % Callback for when we get plans that need to be verified
        function verifyPlanCallback(obj, ~, msg)
            if length(msg.States) == 1
                x0 = msg.States.X; % we only get initial state
            elseif length(msg.States) > 1
                x0 = msg.States(1).X; % extract initial state
            else
                error('Did not get enough states! Got %d', length(msg.States));
            end
            controls = msg.Controls; % extract safe_navigation_msgs/Control[] 
            
            x0(3) = wrapToPi(x0(3)); % make sure we wrap to [-pi, pi]
            verifiedX0 = x0;    % stores initial state of planned traj
            verifiedU = {};     % stores sequence of verified controls
            xcurr = x0;         % state to start our verification from
            xplan = x0;
            appliedUOpt = false;
            
            % just for visualization, store planned states.
            planPath = {xplan};
            for i=1:length(controls)
                % (JUST FOR VISUALIZATION) simulate fwd with plan's
                % control.
                uplan = controls(i).U; 
                obj.params.dynSys.updateState(uplan, obj.params.dt, xplan);
                xplan = obj.params.dynSys.x;
                planPath{end+1} = xplan;
            end
            
            for i=1:length(controls) 
                if appliedUOpt
                    % if we applied safety controller, then the 
                    % plan's sequence of controls is now invalid, so 
                    % we just use the safety control from here on.
                    u = obj.safety.getSafetyControl(xcurr);
                else
                    % grab planner's current control.
                    u = controls(i).U; 
                    
                    % check the safety of the state we reach by applying control
                    [uOpt, onBoundary] = ...
                        obj.safety.checkAndGetSafetyControl(xcurr, obj.params.safetyTol);
                    % if state we reach is unsafe
                    if onBoundary
                        u = uOpt;
                        appliedUOpt = true;
                    end
                end
                
                % Get the new sensing region.
                if strcmp(obj.params.senseShape, 'camera')
                  senseData = {[xcurr(1);xcurr(2);xcurr(3)], ...
                      [obj.params.senseFOV; obj.params.initialR; obj.params.farPlane]};
                else
                  error('Right now, I cannot use a %s with a %s planner\n', ...
                      obj.params.senseShape, obj.params.plannerName);
                end  
                
                % update the occupancy map based on the current state
                obj.map.updateMapAndCost(senseData, obj.params.senseShape);
                
                % modify verified control sequence.
                verifiedU{end+1} = u;
                
            	% simulate the trajectory forward (applying safe or planned
                % control)
                obj.params.dynSys.updateState(u, obj.params.dt, xcurr);
                xcurr = obj.params.dynSys.x;
                
                % update plotting
                obj.plotter.updatePlot(xcurr, obj.params.xgoal, obj.safety.valueFun, ...
                    obj.map.grid, obj.map.gFMM, obj.map.occupancy_map_safety, planPath, appliedUOpt);
                pause(obj.params.dt);
            end
            
            % construct verified trajectory message
            verifiedMsg = obj.toTrajMsg(verifiedX0, verifiedU);
            
            % publish out the verified trajectory
            send(obj.verifiedPub, verifiedMsg);
            
            % update the safe set based on the occupancy map the 
            % robot will see assuming it will execute (exactly) the 
            % verified control sequence
            currTime = 1; % TODO: this is wrong, but doesn't really matter.
            obj.safety.computeAvoidSet(obj.map.signed_dist_safety, 1);
            
            % update plotting after value function is updated
            obj.plotter.updatePlot(xcurr, obj.params.xgoal, obj.safety.valueFun, ...
                obj.map.grid, obj.map.gFMM, obj.map.occupancy_map_safety, planPath, appliedUOpt);
        end
        
    end
end

