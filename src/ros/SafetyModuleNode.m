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
        rawOccuMap      % (arr) Entire SBPD or SLAM ground-truth occupancy map data.
        trueOccuMap     % (arr) (Localized) Ground-truth occupancy map data.
        res             % (float) Ground-truth dx resolution. 
        origin          % (arr) Ground-truth lower x,y coordinate.
        realWidth       % (float) real width of SBPD environment.
        realHeight      % (float) real height of SBPD environment.
        firstMap        % (bool) if this is the first map we have received.
        initSensingShape    % (array) for SLAM initial sensing region/shape
        initData2D
        firstCompute
        
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
            
            % Load up all the experimental parameters.
            %obj.params = car4DLocalQCameraNN();    % run inside of SBPD
            obj.params = car4DLocalQCameraSLAM();   % run SLAM environment
            
            % If we are running SLAM, we are using the turtlebot, 
            % so we need to set the ROS master uri to the turtlebot.
            if strcmp(obj.params.envType, 'slam')
                setenv('ROS_MASTER_URI','http://128.32.38.83:11311');
            end
            
            % Create a new node.
            rosinit 
            
            % We are receiving the first map.
            obj.firstMap = true;
            obj.rawOccuMap = [];
            obj.firstCompute = true; % TODO: is this correct?
            
            if strcmp(obj.params.envType, 'sbpd') && obj.params.loadTrueOccuMaps 
                % Load the saved ground-truth occupancy maps.
                fprintf('Loading ground-truth SBPD occupancy map.\n');
                repo = what('safe_navigation');
                sbpdPath = strcat(repo.path, '/data/sbpdOccuMapInfo.mat');  
                load(sbpdPath);
                obj.rawOccuMap = sbpdOccuMap;
                obj.origin = sbpdOrigin;
                obj.res = resolution;
                obj.realHeight = realHeight;
                obj.realWidth = realWidth;
                
                % Extract the ground-truth occupancy map based on our 
                % compute grid.
                obj.getTrueOccuMap();
            elseif strcmp(obj.params.envType, 'sbpd') && ~obj.params.loadTrueOccuMaps
                % Listen on the map topic for ground-truth occupancy maps.
                occuMapMsgType = 'nav_msgs/OccupancyGrid';
                obj.occuMapSub = rossubscriber(obj.params.occuMapTopicName, occuMapMsgType);
                
                % Grab the ground-truth occupancy map from Python.
                fprintf('Listening over ROS to ground-truth SBPD occupancy map.\n');
                msg = receive(obj.occuMapSub, 10);
                obj.saveSBPDGroundTruthOccupancyGrids(msg);
                
                % Extract the ground-truth occupancy map based on our 
                % compute grid.
                obj.getTrueOccuMap();
            elseif strcmp(obj.params.envType, 'slam')
                center = obj.params.initSenseData{1}(1:2);
                radius = obj.params.initSenseData{2}(2); 
                obj.initSensingShape = -shapeCylinder(obj.params.grid, [3,4], center, radius);
                [~, obj.initData2D] = proj(obj.params.grid, obj.initSensingShape, [0 0 1 1], [0 0]);
                
                % Subscriber that listens to SLAM occupancy maps.
                occuMapMsgType = 'nav_msgs/OccupancyGrid';
                obj.occuMapSub = rossubscriber(obj.params.occuMapTopicName, ...
                    occuMapMsgType, @obj.slamMapCallback);
                pause(2); % wait for subscriber to get called
            else
                error('SafetyModuleNode does not currently support environment: %s\n', ...
                    obj.params.envType);
            end
            
            % Setup safety module object and compute first set.
            obj.safety = SafetyModule(obj.params.grid, obj.params.dynSys, ...
                            obj.params.uMode, obj.params.dMode, obj.params.dt, ...
                            obj.params.updateEpsilon, obj.params.warmStart, ...
                            obj.params.envType, obj.params.updateMethod, obj.params.tMax);
            
            % Create the occupancy map handler.
            % TODO: THIS MAY NOT BE SET BEFORE....
            while isempty(obj.trueOccuMap)
                fprintf('Waiting to receive occupancy map...\n');
                pause(0.5);
            end
            extraArgs.occuMap = obj.trueOccuMap;
            obj.map = OccuMap(obj.params.grid, obj.params.envType, extraArgs);
            obj.plotter = Plotter(obj.params.lowEnv, obj.params.upEnv, ...
                obj.map.boundLow, obj.map.boundUp, obj.params.envType, ... 
                obj.trueOccuMap, obj.params.goalEps);
            
            % Map has been set up
            obj.firstCompute = false;
            
            % Update the occupancy map and the corresponding signed
            % distance function. 
            if strcmp(obj.params.envType, 'sbpd')
                obj.map.updateMapAndCost(obj.params.initSenseData, obj.params.senseShape);
            elseif strcmp(obj.params.envType, 'slam')
                % We need to pass in the combination of the initial radius
                % from which we computed the safe set + the slam map.
                obj.map.updateMapAndCost(obj.trueOccuMap, obj.params.senseShape);
            else
                error('SafetyModuleNode does not currently support environment: %s\n', ...
                    obj.params.envType);
            end
            
            % Compute the first avoid set based on initial safe set.
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
        function saveSBPDGroundTruthOccupancyGrids(obj, msg)
            obj.res = double(msg.Info.Resolution);
            r = msg.Info.Height;
            c = msg.Info.Width;
            obj.realHeight = double(r*obj.res);
            obj.realWidth = double(c*obj.res);
            obj.origin = [double(msg.Info.Origin.Position.X), double(msg.Info.Origin.Position.Y)];
            
            % Save the entire SBPD ground-truth occupancy map
            obj.rawOccuMap = double(reshape(msg.Data, [r,c])); 
            
            fprintf('Received ground truth map with: \n');
            fprintf('    height: %f, # rows: %d\n', obj.realHeight, r);
            fprintf('    width: %f, # cols: %d\n', obj.realWidth, c);
            fprintf('    res: %f\n', obj.res);
            
            % Get just the part of the grid where we will be doing computations.
            obj.getTrueOccuMap();
            %figure, imshow(flipud(obj.trueOccuMap'));
        end
        
        % Grabs the most recent SLAM occupancy map and converts it into 
        % a compatible representation for safety computations.
        function slamMapCallback(obj, ~, msg)
            obj.res = double(msg.Info.Resolution);
            numY = msg.Info.Height;
            numX = msg.Info.Width;
            obj.realHeight = double(numY)*double(obj.res);
            obj.realWidth = double(numX)*double(obj.res);
            obj.origin = [double(msg.Info.Origin.Position.X), double(msg.Info.Origin.Position.Y)];
            
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
            
            receivedNewMapVals = false;
            if isempty(obj.rawOccuMap)
                % If this is the first time we get the SLAM map, need
                % to store it and go through all the signed dist computations.
                receivedNewMapVals = true;
            else
                % If SLAM has updated the map, then we also need to store
                % it and go through all the signed dist computations.
                if ~isequal(obj.rawOccuMap, slamOccuMap)
                    receivedNewMapVals = slamOccuMap;
                end
            end
            
            if receivedNewMapVals 
                fprintf('Got new SLAM occupancy map message with info:\n');
                fprintf('     realH: %f, # rows: %d\n', obj.realHeight, numY);
                fprintf('     realW: %f, # cols: %d\n', obj.realWidth, numX);
                fprintf('     origin: (%d, %d)\n', obj.origin(1), obj.origin(2));

                % Store the correct representation of the full SLAM map.
                obj.rawOccuMap = slamOccuMap;

                % Crop and resize the SLAM map to match compute grid size.
                if obj.params.grid.dim == 4
                    [grid2D, ~] = proj(obj.params.grid, obj.params.grid.xs{1}, [0 0 1 1], [0 0]);
                else
                    error('I am not programmed to use safety module with %D system.\n', obj.params.grid.dim);
                end
                mapBounds = [obj.origin(1), obj.origin(2), obj.origin(1) + obj.realWidth, obj.origin(2) + obj.realHeight];
                %mapBounds = [0, 0, obj.realWidth, obj.realHeight];
                
                % Crop and interpolate the raw occupancy map from SBPD or SLAM
                % into the correct size for the computation grid.
                obj.trueOccuMap = ...
                    generate_computation_grid(grid2D, obj.rawOccuMap, ...
                    obj.res, mapBounds);

                % Need to add in the initial sensing circular radius of
                % free space to the SLAM occupancy map.
                initFreeIndicies = find(obj.initData2D >= 0);
                obj.trueOccuMap(initFreeIndicies) = 1; 
                
                if ~obj.firstCompute
                    % Update the signed distance function.
                    obj.map.updateMapAndCost(obj.trueOccuMap, obj.params.senseShape);
                    % update plotting
                    obj.plotter.updatePlot([], obj.params.xgoal, obj.safety.valueFun, ...
                        obj.map.grid, obj.map.gFMM, obj.map.occupancy_map_safety, [], []);
                end
            end
            
            
        end
        
        % Extracts ground-truth map relative to our compute grid.
        function getTrueOccuMap(obj)
            % Note: only works for 4D system right now.
            if obj.params.grid.dim == 4
                [grid2D, ~] = proj(obj.params.grid, obj.params.grid.xs{1}, [0 0 1 1], [0 0]);
            else
                error('I am not programmed to use safety module with %D system.\n', obj.params.grid.dim);
            end
            mapBounds = [obj.origin(1), obj.origin(2), obj.origin(1) + obj.realWidth, obj.origin(2) + obj.realHeight];
            if strcmp(obj.params.envType, 'sbpd')
                bigMap = obj.rawOccuMap'; % need to transpose for sbpd 
            else
                bigMap = obj.rawOccuMap;
            end
            
            % Crop and interpolate the raw occupancy map from SBPD or SLAM
            % into the correct size for the computation grid.
            obj.trueOccuMap = ...
                generate_computation_grid(grid2D, bigMap, ...
                obj.res, mapBounds);
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
            
            % How close to zero does the spatial derivative have to be
            % to be considered = 0?
            gradZeroTol = 0.01;
            
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
                    [uOpt, onBoundary, deriv] = ...
                        obj.safety.checkAndGetSafetyControl(xcurr, obj.params.safetyTol);
                    % extract just the theta and velocity derivative;
                    deriv = deriv(3:4);
                    % if state we reach is unsafe
                    if onBoundary
                        %u = uOpt.*(abs(deriv) > gradZeroTol) + u.*(abs(deriv) < gradZeroTol);
                        u = uOpt;
                        appliedUOpt = true;
                    end
                end
                
                % Get the new sensing region.
                if strcmp(obj.params.envType, 'sbpd') && strcmp(obj.params.senseShape, 'camera')
                    senseData = {[xcurr(1);xcurr(2);xcurr(3)], ...
                      [obj.params.senseFOV; obj.params.initialR; obj.params.farPlane]};
                  
                    % update the occupancy map based on the current state
                    % in simulation.
                    obj.map.updateMapAndCost(senseData, obj.params.senseShape);
                end 
                %elseif strcmp(obj.params.envType, 'slam') && strcmp(obj.params.senseShape, 'camera')
                %    senseData = obj.trueOccuMap;
                %else
                %    error('Right now, SafetyModuleNode cannot use a %s in a %s environment\n', ...
                %      obj.params.senseShape, obj.params.envType);
                %end  
                

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

