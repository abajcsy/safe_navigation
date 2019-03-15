classdef SafetyUpdaterNodePremapped < handle
    %SAFETYUPDATENODE Encapsulates all the safety computations 
    % and verification of the neural network plans with ROS communication.
    
    properties
        % Subscribers and publishers.
        occuMapSub
        odomSub
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
        
        % State of robot
        currState
        currSafeSetMsg
        prevSafetyUpdate
        
        % Figure data.
        %plotter
    end
    
    methods
        %% Constructor
        function obj = SafetyUpdaterNodePremapped()
            clf
            rmpath(genpath('/home/somilb/Documents/MATLAB/helperOC/'));
            addpath(genpath('/home/somilb/Documents/MATLAB/helperOC_dev/'));
            addpath(genpath('/home/somilb/Documents/Projects/visual_mpc/safe_navigation_ws/src/matlab_gen/'));
            addpath(genpath('/home/somilb/Documents/Projects/safe_navigation/'));
            
            % Load up all the experimental parameters.
            obj.params = car4DLocalQCameraSLAM();   % run SLAM environment
            
            % If we are running SLAM, we are using the turtlebot, 
            % so we need to set the ROS master uri to the turtlebot.
            setenv('ROS_MASTER_URI','http://128.32.38.83:11311');
            
            % Create a new node.
            rosinit 
            
            % We are receiving the first map.
            obj.firstMap = true;
            obj.rawOccuMap = [];
            obj.trueOccuMap = [];
            obj.firstCompute = true; % TODO: is this correct?
            obj.currState = obj.params.xinit;
            
            % Grab the ground-truth occupancy map that was pre-mapped with SLAM.
            fprintf('Loading ground-truth SLAM occupancy map.\n');
            repo = what('safe_navigation');
            slamPath = strcat(repo.path, '/data/slamTrueOccuMap.mat');  
            load(slamPath);
            obj.rawOccuMap = rawOccuMap;
            obj.origin = origin - [2.,2.]; 
            obj.realWidth = realWidth;
            obj.realHeight = realHeight;
            obj.res = res;
            
            % Extract the ground-truth occupancy map based on our 
            % compute grid.
            obj.getTrueOccuMap();
            
            % HACK -- normalize by vehicle position
%             vehiclePos = [2.1;2.1];
%             obj.params.lowEnv = obj.params.lowEnv - vehiclePos(1:2);
%             obj.params.upEnv = obj.params.upEnv - vehiclePos(1:2);
%             obj.params.grid.xs{1} = obj.params.grid.xs{1} - vehiclePos(1);
%             obj.params.grid.xs{2} = obj.params.grid.xs{2} - vehiclePos(2);
            
            % Setup the initial sensing shape.
            center = obj.params.initSenseData{1}(1:2);
            radius = obj.params.initSenseData{2}(2); 
            obj.initSensingShape = -shapeCylinder(obj.params.grid, [3,4], center, radius);
            [~, obj.initData2D] = proj(obj.params.grid, obj.initSensingShape, [0 0 1 1], [0 0]);
           
            % Setup safety module object and compute first set.
            obj.safety = SafetyModule(obj.params.grid, obj.params.dynSys, ...
                            obj.params.uMode, obj.params.dMode, obj.params.dt, ...
                            obj.params.updateEpsilon, obj.params.warmStart, ...
                            obj.params.envType, obj.params.updateMethod, ...
                            obj.params.tMax, obj.params.initialR);
            
            % Map has been set up.
            obj.firstCompute = false;
            
            % Setup occupancy map handler.
            extraArgs.occuMap = obj.trueOccuMap;
            obj.map = OccuMap(obj.params.grid, obj.params.envType, extraArgs);
            
            % Update the occupancy map and the corresponding signed
            % distance function. 
            % We need to pass in the combination of the initial radius
            % from which we computed the safe set.
            obj.map.updateMapAndCost(obj.params.initSenseData, obj.params.senseShape);
            
            % Compute the first avoid set based on initial safe set.
            obj.safety.computeAvoidSet(obj.map.signed_dist_safety, 1);
            obj.prevSafetyUpdate = rostime('now');
            if obj.params.grid.dim == 4
                currSafeSet = obj.safety.valueFun(:,:,:,:,end);
            else
                error('SafetyUpdaterNodePremapped does not support %dD grids\n',  obj.params.grid.dim);
            end
            
            % Setup ROS pub/sub for trajectories to verify.
            obj.registerCallbacks();
            
            % Construct first safe set.
            safeSetMsg = obj.toSafeSetMsg(currSafeSet);
            obj.currSafeSetMsg = safeSetMsg;
            
            % Spin and let everything go.
            rate = rosrate(100);
            reset(rate);
            while true
                % Publish out the safe set.
                send(obj.safetyPub, obj.currSafeSetMsg);
                waitfor(rate);
            end
        end
                       
        %% Create all the publishers and subscribers.
        function registerCallbacks(obj)
            % Publisher that publishes out current safe set.
            safeMsgType = 'safe_navigation_msgs/SafeSet';
            safeSetTopicName = '/safe_set';
            obj.safetyPub = rospublisher(safeSetTopicName, safeMsgType);
            
            odomMsgType = 'nav_msgs/Odometry';
            odomTopicName = '/odom';
            obj.odomSub = rossubscriber(odomTopicName, ...
                odomMsgType, @obj.odomCallback);
            pause(1); % wait for subscriber to get called
        end
        
        %% Extracts ground-truth map relative to our compute grid.
        function getTrueOccuMap(obj)
            % Note: only works for 4D system right now.
            if obj.params.grid.dim == 4
                [grid2D, ~] = proj(obj.params.grid, obj.params.grid.xs{1}, [0 0 1 1], [0 0]);
            else
                error('I am not programmed to use safety module with %D system.\n', obj.params.grid.dim);
            end
            mapBounds = [obj.origin(1), obj.origin(2), obj.origin(1) + obj.realWidth, obj.origin(2) + obj.realHeight];
            
            % Crop and interpolate the raw occupancy map from SBPD or SLAM
            % into the correct size for the computation grid.
            obj.trueOccuMap = ...
                generate_computation_grid(grid2D, obj.rawOccuMap, ...
                obj.res, mapBounds);
            
            % Need to add in the initial sensing circular radius of
            % free space to the SLAM occupancy map.
            initFreeIndicies = find(obj.initData2D >= 0);
            obj.trueOccuMap(initFreeIndicies) = 1; 
        end
        
        %% Converts ND value function into ROS message.
        function msg = toSafeSetMsg(obj, valueFun)
            % Note: we assume that the value function that was 3D was
            % flattened by doing reshape(arrVx, [1,prod(N)]) so that it can
            % be reconstructed by doing reshape(flatVx, N)
            msg = rosmessage('safe_navigation_msgs/SafeSet');
            msg.N = obj.params.grid.shape;
            msg.Dim = obj.params.grid.dim;
            msg.Values = valueFun(:); %reshape(valueFun, [1,prod(obj.params.grid.shape)]); 
            
            fprintf('Created new Safe Set message to send!\n');
        end
        
        %% Records the current state of the robot from odometry topic.
        % Used for plotting.
        function odomCallback(obj, ~, msg)
            x = msg.Pose.Pose.Position.X;
            y = msg.Pose.Pose.Position.Y;
            % convert from quaternion to angle.
            quat = [msg.Pose.Pose.Orientation.X, msg.Pose.Pose.Orientation.Y, ...
                msg.Pose.Pose.Orientation.Z, msg.Pose.Pose.Orientation.W];
            eulerAngle = quat2eul(quat);
            theta = wrapToPi(eulerAngle(3));
            vel = msg.Twist.Twist.Linear.X;
            obj.currState = [x;y;theta;vel];
            
            % Update sensing info.
            senseData = {[x;y;theta], ...
                [obj.params.senseFOV; obj.params.initialR; obj.params.farPlane]};

            % update the occupancy map based on the current state
            % in simulation.
            obj.map.updateMapAndCost(senseData, obj.params.senseShape);
            
            % update safe set if it is time.
            currTime = rostime('now');
            deltaTime = seconds(currTime) - seconds(obj.prevSafetyUpdate);
            
            if deltaTime > obj.params.safetyFreq
                % recompute safety.
                obj.safety.computeAvoidSet(obj.map.signed_dist_safety, 1);
                
                if obj.params.grid.dim == 4
                    currSafeSet = obj.safety.valueFun(:,:,:,:,end);
                else
                    error('SafetyUpdaterNodePremapped does not support %dD grids\n',  obj.params.grid.dim);
                end
                % update current safe set message and send it.
                obj.currSafeSetMsg = toSafeSetMsg(obj, currSafeSet);
                send(obj.safetyPub, obj.currSafeSetMsg);
                
                obj.prevSafetyUpdate = rostime('now');
            end
        end

    end
end

