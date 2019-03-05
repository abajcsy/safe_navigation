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
        trueOccuMap     % (arr) Ground-truth occupancy map data.
        resolution      % (float) Ground-truth dx resolution. 
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
            [obj.trueOccuMap, obj.resolution, obj.origin] = obj.fromOccupancyGrid(msg);
            
            % Compute the first avoid set based on current sensing.
            % obj.updateBeliefOccuMap(obj.params.xinit, obj.params.initSenseData, obj.params.senseShape);
            % obj.updateSignedDist()
            % obj.safety.computeAvoidSet(signed_dist_safety, 1);
            
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
            %obj.safety = SafetyModule(obj.params.grid, obj.params.dynSys, obj.params.uMode, ...
            %                obj.params.dt, obj.params.updateEpsilon, obj.params.warmStart, ...
            %                obj.params.updateMethod, obj.params.tMax);
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
        function [occuGrid, res, origin] = fromOccupancyGrid(obj, msg)
            % TODO.
            res = double(msg.Info.Resolution);
            h = msg.Info.Height;
            w = msg.Info.Width;
            origin = [msg.Info.Origin.Position.X, msg.Info.Origin.Position.Y];
            %TODO: sanity check that the reshape does it right.
            occuGrid = reshape(msg.Data, [h,w]); 
            fprintf('Received ground truth map with: \n');
            fprintf('    height: %f, # rows: %d\n', h, rows);
            fprintf('    width: %f, # cols: %d\n', w, cols);
            fprintf('    res: %f\n', res);
        end
        
        %% ------------------- ROS Callbacks  -------------------- %
        % Callback for when we get plans that need to be verified
        function verifyPlanCallback(obj, ~, msg)
            if length(msg.States) == 1
                currState = msg.States.X; % we only get initial state
            elseif length(msg.States) > 1
                currState = msg.States(1).X; % extract initial state
            else
                error('Did not get enough states! Got %d', length(msg.States));
            end
            controls = msg.Controls; % these are Control[] msgs
            
            x = currState;
            verifiedX0 = currState;
            verifiedU = [];
            
            for i=1:length(controls) % TODO: this is wrong, as soon as we apply safety control, 
                                     % all the controls after are messed
                                     % up...
                % grab candidate 
                u = controls(i).U; 
                
%                 % update the occupancy map based on the current state
%                 % check the safety of the state we reach by applying control
%             
%                 [uOpt, onBoundary] = ...
%                     obj.safety.checkAndGetSafetyControl(x, obj.params.safetyTol);
%                 % if state we reach is unsafe
%                 if onBoundary
%                     u = uOpt;
%                 end

                % modify next control to apply to be safe control
                verifiedU = [verifiedU, u];
% 
%             	% simulate the trajectory forward (applying safe or planned
%                 % control)
%                 obj.params.dynSys.updateState(u, obj.params.dt, x);
%                 x = obj.params.dynSys.x;
            end
            
            % construct verified trajectory message
            verifiedMsg = obj.toTrajMsg(verifiedX0, verifiedU);
            
            % publish out the verified trajectory
            send(obj.verifiedPub, verifiedMsg);
            
            % update the safe set based on the occupancy map the 
            % robot will see assuming it will execute the 
            % verified control sequence
            % 
            % currTime = 1; % TODO: this is wrong, but doesnt really matter.
            % obj.safety.computeAvoidSet(signed_dist_safety, currTime);
        end
        
        %% ---------------- Occupancy Grid Functions ------------------- %
        function updateBeliefOccuMap(obj, xcurr, senseData, senseShape)
            % TODO.
            obj.beliefOccuMap = [];
        end
        
        function updateSignedDist(obj)
            % TODO.
            obj.signedDist = [];
        end
    end
end

