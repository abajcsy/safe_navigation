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
        %beliefOccuMap   % (arr) Occupancy map based on what we have sensed.
        %signedDist      % (arr) Signed distance function based on beliefOccuMap. 
        
        % Figure handles.
        plt
        carh
        maph
        safeh 
    end
    
    methods
        %% Constructor
        function obj = SafetyModuleNode()
            clf
            % Create a new node.
            rosinit 
            
            % Load up all the experimental parameters.
            obj.params = car4DLocalQCameraNN();
            
            % Setup ROS pub/sub.
            obj.registerCallbacks();
            
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
                        
            extraArgs.occuMap = obj.trueOccuMap;
            obj.map = OccuMap(obj.params.grid, obj.params.envType, extraArgs);
            obj.plt = Plotter(obj.params.lowEnv, obj.params.upEnv,[],[],[]);
            
            % ----- Plot initial state of car and ground-truth occupancy
            % map. -----
            hold on
            obj.maph = obj.plt.plotOccuMap(obj.params.grid, obj.trueOccuMap);
            obj.carh = obj.plt.plotCar(obj.params.xinit, false);
            
            % Compute the first avoid set based on initial safe set.
            obj.map.updateMapAndCost(obj.params.initSenseData, obj.params.senseShape);
            obj.safety.computeAvoidSet(obj.map.signed_dist_safety, 1);
            
            % ---- Plot the safe set. -----
            funToSee = obj.safety.valueFun(:,:,:,:,end);
            extraArgs.theta = obj.params.xinit(3);
            extraArgs.vel = obj.params.xinit(4);
            obj.safeh = obj.plt.plotFuncLevelSet(obj.params.grid, funToSee, true, extraArgs);
            
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

            if obj.params.loadTrueOccuMaps
                % Subscriber that listens to occupancy grid.
                occuMapMsgType = 'nav_msgs/OccupancyGrid';
                obj.occuMapSub = rossubscriber(obj.params.occuMapTopicName, occuMapMsgType);
            end
            
            % Subscriber that listens to planned trajectory
            planMsgType = 'safe_navigation_msgs/Trajectory';
            obj.planSub = rossubscriber(obj.params.planTopicName, planMsgType, @obj.verifyPlanCallback);
            pause(2); 
        end
        
        %% ------------------- ROS Msg Util Functions -------------------- %
        % Converts from array to trajectory message
        function msg = toTrajMsg(obj, x0, controls)
            msg = rosmessage('safe_navigation_msgs/Trajectory');
            
            % grab just first state
            initState = rosmessage('safe_navigation_msgs/State');
            initState.X = x0;
            
            msg.States = [initState];
            msg.Controls = [];
            for i=1:length(controls)
                control = rosmessage('safe_navigation_msgs/Control');
                control.U = controls(i);
                msg.Controls = [msg.Controls, control];
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
            
            verifiedX0 = x0;    % stores initial state of planned traj
            verifiedU = [];     % stores sequence of verified controls
            xcurr = x0;         % state to start our verification from
            appliedUOpt = false;
            
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
                      [obj.params.senseFOV; obj.params.initialR]};
                else
                  error('Right now, I cannot use a %s with a %s planner\n', ...
                      obj.params.senseShape, obj.params.plannerName);
                end  
                
                % update the occupancy map based on the current state
                obj.updateBeliefOccuMap(senseData, obj.params.senseShape);
                obj.updateSignedDist();
                
                % modify verified control sequence.
                verifiedU = [verifiedU, u];

            	% simulate the trajectory forward (applying safe or planned
                % control)
                obj.params.dynSys.updateState(u, obj.params.dt, xcurr);
                xcurr = obj.params.dynSys.x;
            end
            
            % construct verified trajectory message
            verifiedMsg = obj.toTrajMsg(verifiedX0, verifiedU);
            
            % publish out the verified trajectory
            send(obj.verifiedPub, verifiedMsg);
            
            % update the safe set based on the occupancy map the 
            % robot will see assuming it will execute (exactly) the 
            % verified control sequence
            currTime = 1; % TODO: this is wrong, but doesn't really matter.
            obj.safety.computeAvoidSet(obj.signedDist, currTime);
        end
        
        %% ---------------- Occupancy Grid Functions ------------------- %
        
        % Updates the safety occupancy map. 
        function updateBeliefOccuMap(obj, senseData, senseShape)
            % Project the slice of obstacle
            if obj.params.grid.dim == 4
                % Note: we just need to grab 2D computation grid here.
                [gFMM, ~] = proj(obj.params.grid, obj.params.grid.xs{1}, [0 0 1 1], [0, 0]);
            else
                error('I am not programmed to use safety module with %D system.\n', obj.params.grid.dim);
            end
            
            if strcmp(senseShape, 'circle')
                % Record which states we have sensed. 
                % (+1 sensed, -1 unsensed)
                sensingShape = -shapeCylinder(obj.params.grid, [3,4], senseData{1}(1:2), senseData{2}(2));

                % Union the sensed region with the actual obstacle.
                unionL = shapeUnion(sensingShape, obj.lReal);
                % Project the slice of unionL and create an occupancy map
                [obj.gFMM, dataFMM] = proj(obj.grid, unionL, [0 0 1 1], [0, 0]);

                % also: subtract small epsilon in case we get zero's
                epsilon = 1e-6;
                obj.beliefOccuMap = sign(dataFMM-epsilon);
            elseif strcmp(senseShape, 'camera')
                [obj.beliefOccuMap, ~] = ...
                    generate_camera_sensing_region(gFMM, obj.trueOccuMap, ...
                        senseData{2}(1), senseData{1}(1:2), senseData{1}(3));
            else
                error('Right now, I cannot use a %s with the safety module.\n', ...
                      obj.params.senseShape);
            end
        end
        
        % Updates signed distance function used by safety given the 
        % current belief occupancy grid.
        function updateSignedDist(obj)
            
            % TODO: obj.lReal needs to be generated from trueOccuGrid
            [gFMM, ~] = proj(obj.params.grid, obj.params.grid.xs{1}, [0 0 1 1], [0, 0]);
            
            % We will use the FMM code to get the signed distance function. 
            % Since the FMM code works only on 2D, we will take a slice of 
            % the grid, compute FMM, and then project it back to a 3D array.
            unionL_2D_FMM = compute_fmm_map(gFMM, obj.beliefOccuMap);
            
            if obj.params.grid.dim == 4
                obj.signedDist = repmat(unionL_2D_FMM, 1, 1, obj.params.grid.N(3), obj.params.grid.N(4));
            else
                error('I am not programmed to use safety module with %D system.\n', obj.params.grid.dim);
            end
        end
    end
end

