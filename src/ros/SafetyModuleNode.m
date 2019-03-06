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
        sbpdOccuMap     % (arr) Entire SBPD ground-truth occupancy map data.
        trueOccuMap     % (arr) (Localized) Ground-truth occupancy map data.
        res             % (float) Ground-truth dx resolution. 
        origin          % (arr) Ground-truth lower x,y coordinate.
        beliefOccuMap   % (arr) Occupancy map based on what we have sensed.
        signedDist      % (arr) Signed distance function based on beliefOccuMap. 
    end
    
    methods
        %% Constructor
        function obj = SafetyModuleNode()
            % Create a new node.
            rosinit 
            
            % Load all the experimental parameters and setup ROS pub/sub.
            obj.loadParameters();
            obj.registerCallbacks();
            
            % Grab the ground-truth occupancy map from Python.
            fprintf('Getting ground-truth occupancy map.\n');
            msg = receive(obj.occuMapSub, 10);
            obj.saveGroundTruthOccupancyGrids(msg);
            
            % Compute the first avoid set based on current sensing.
            obj.updateBeliefOccuMap(obj.params.initSenseData, obj.params.senseShape);
            obj.updateSignedDist();
            obj.safety.computeAvoidSet(obj.signedDist, 1);
            
            % Spin and let everything go.
            rate = rosrate(100);
            reset(rate);
            while true
                waitfor(rate);
            end
        end
        
        %% ------------ ROS Subscriber/Publisher Functions ------------- %%
                
        % Load all the parameters and construct safety object.
        function loadParameters(obj)
            % load up all the parameters
            obj.params = car4DLocalQCameraNN();

            % Setup safety module object and compute first set.
            obj.safety = SafetyModule(obj.params.grid, obj.params.dynSys, obj.params.uMode, ...
                           obj.params.dt, obj.params.updateEpsilon, obj.params.warmStart, ...
                           obj.params.updateMethod, obj.params.tMax);
            
            obj.beliefOccuMap = [];
            obj.signedDist = [];
        end
        
        % Create all the publishers and subscribers.
        function registerCallbacks(obj)
            % Publisher that publishes out verified trajectory.
            verifiedMsgType = 'safe_navigation_msgs/Trajectory';
            obj.verifiedPub = rospublisher(obj.params.verifiedTopicName,verifiedMsgType);

            % Subscriber that listens to occupancy grid.
            occuMapMsgType = 'nav_msgs/OccupancyGrid';
            obj.occuMapSub = rossubscriber(obj.params.occuMapTopicName, occuMapMsgType);
            
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
                generate_computation_grid(grid2D, obj.sbpdOccuMap, ...
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
            
            if strcmp(senseShape, 'camera')
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

