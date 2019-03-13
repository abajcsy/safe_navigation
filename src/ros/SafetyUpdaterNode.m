classdef SafetyUpdaterNode < handle
    %SAFETYUPDATENODE Encapsulates all the safety computations 
    % and verification of the neural network plans with ROS communication.
    
    properties
        % Subscribers and publishers.
        occuMapSub
        safetyPub
        
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
        realWidth       % (float) real width of SLAM environment.
        realHeight      % (float) real height of SLAM environment.
        firstMap        % (bool) if this is the first map we have received.
        initSensingShape    % (array) for SLAM initial sensing region/shape
        initData2D
        firstCompute
        
        % Figure data.
        plotter
    end
    
    methods
        %% Constructor
        function obj = SafetyUpdaterNode()
            clf
            rmpath(genpath('/home/somilb/Documents/MATLAB/helperOC/'));
            addpath(genpath('/home/somilb/Documents/MATLAB/helperOC_dev/'));
            addpath(genpath('/home/somilb/Documents/Projects/visual_mpc/safe_navigation_ws/src/matlab_gen/'));
            addpath(genpath('/home/somilb/Documents/Projects/safe_navigation/'));
            
            % Load up all the experimental parameters.
            obj.params = car4DLocalQCameraSLAM();   % run SLAM environment
            
            % If we are running SLAM, we are using the turtlebot, 
            % so we need to set the ROS master uri to the turtlebot.
            if strcmp(obj.params.envType, 'slam')
                setenv('ROS_MASTER_URI','http://128.32.38.83:11311');
            else
                 error('The safety update node is not set up for any environment other than SLAM!\n');
            end
            
            % Create a new node.
            rosinit 
            
            % We are receiving the first map.
            obj.firstMap = true;
            obj.rawOccuMap = [];
            obj.trueOccuMap = [];
            obj.firstCompute = true; % TODO: is this correct?
            
            % Setup the initial sensing shape.
            center = obj.params.initSenseData{1}(1:2);
            radius = obj.params.initSenseData{2}(2); 
            obj.initSensingShape = -shapeCylinder(obj.params.grid, [3,4], center, radius);
            [~, obj.initData2D] = proj(obj.params.grid, obj.initSensingShape, [0 0 1 1], [0 0]);

            % Subscriber that listens to SLAM occupancy maps.
            occuMapMsgType = 'nav_msgs/OccupancyGrid';
            obj.occuMapSub = rossubscriber(obj.params.occuMapTopicName, ...
                occuMapMsgType, @obj.slamMapCallback);
            pause(1); % wait for subscriber to get called

            % Setup safety module object and compute first set.
            obj.safety = SafetyModule(obj.params.grid, obj.params.dynSys, ...
                            obj.params.uMode, obj.params.dMode, obj.params.dt, ...
                            obj.params.updateEpsilon, obj.params.warmStart, ...
                            obj.params.envType, obj.params.updateMethod, obj.params.tMax);
            
            % Spin and make sure we get the first SLAM map. 
            while isempty(obj.trueOccuMap)
                fprintf('Waiting to receive occupancy map...\n');
                pause(0.5);
            end
            
            % Map has been set up.
            obj.firstCompute = false;
            
            % Setup occupancy map handler.
            extraArgs.occuMap = obj.trueOccuMap;
            obj.map = OccuMap(obj.params.grid, obj.params.envType, extraArgs);
            obj.plotter = Plotter(obj.params.lowEnv, obj.params.upEnv, ...
                obj.map.boundLow, obj.map.boundUp, obj.params.envType, ... 
                obj.trueOccuMap, obj.params.goalEps);
            
            % Update the occupancy map and the corresponding signed
            % distance function. 
            % We need to pass in the combination of the initial radius
            % from which we computed the safe set + the slam map.
            obj.map.updateMapAndCost(obj.trueOccuMap, obj.params.senseShape);
            
            % Compute the first avoid set based on initial safe set.
            obj.safety.computeAvoidSet(obj.map.signed_dist_safety, 1);
            if obj.params.grid.dim == 4
                currSafeSet = obj.safety.valueFun(:,:,:,:,end);
            else
                error('Safety Updater Node does not support %dD grids\n',  obj.params.grid.dim);
            end
            
            % Publish out the initial safe set.
            safeSetMsg = obj.toSafeSetMsg(currSafeSet);
            send(obj.safetyPub, safeSetMsg);
            
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
                       
        %% Create all the publishers and subscribers.
        function registerCallbacks(obj)
            % Publisher that publishes out current safe set.
            safeMsgType = 'safe_navigation_msgs/SafeSet';
            safeSetTopicName = '/safe_set';
            obj.safetyPub = rospublisher(safeSetTopicName, safeMsgType);
        end
        
        %% Converts ND value function into ROS message.
        function msg = toSafeSetMsg(obj, valueFun)
            % Note: we assume that the value function that was 3D was
            % flattened by doing reshape(arrVx, [1,prod(N)]) so that it can
            % be reconstructed by doing reshape(flatVx, N)
            
            msg = rosmessage('safe_navigation_msgs/SafeSet');
            msg.N = obj.params.N;
            msg.Dim = obj.params.grid.dim;
            msg.Values = reshape(valueFun, [1,prod(obj.params.N)]); 
        end
                
        %% Grabs the most recent SLAM occupancy map and converts it into 
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
                
                % Add in the 1-grid-cell-sized boundary obstacle into the
                % slam map for numerics.
                [~, boundaryData2D] = proj(obj.params.grid, obj.lBoundary, [0 0 1 1], [0 0]);
                boundaryIndicies = find(boundaryData2D < 0);
                obj.trueOccuMap(boundaryIndicies) = -1; 
                
                if ~obj.firstCompute
                    % Update the signed distance function.
                    obj.map.updateMapAndCost(obj.trueOccuMap, obj.params.senseShape);
                    
                    % Do safety update based on most recent SLAM map.
                    obj.safety.computeAvoidSet(obj.map.signed_dist_safety, 1);
                    currSafeSet = obj.safety.valueFun(:,:,:,:,end);

                    % Convert and send out message containing safe set.
                    safeSetMsg = obj.toSafetyMsg(currSafeSet);
                    send(obj.safetyPub, safeSetMsg);
                    
                    % update plotting
                    obj.plotter.updatePlot([], obj.params.xgoal, obj.safety.valueFun, ...
                        obj.map.grid, obj.map.gFMM, obj.map.occupancy_map_safety, [], []);
                end
            end
            
        end
               
    end
end

