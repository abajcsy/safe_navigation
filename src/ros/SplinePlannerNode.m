classdef SplinePlannerNode < handle
    %SPLINEPLANNERNODE Interface between MATLAB and Python Spline Planner.
    
    properties
        replanPub   % Publisher that requests for a new plan from Python Spline Planner.
        trajSub     % Subscriber that listens to new spline plans
        states      % (cell arr) of most recent planned states
        controls    % (cell arr) of most recent controls
    end
    
    methods
        %% Constructor.
        function obj = SplinePlannerNode()
            setenv('ROS_MASTER_URI', 'http://localhost:11311');
            
            % Create a new node.
            rosinit 
            
            % Publisher that initiates replan requests.
            pubTopicName = '/replan_info';
            pubMsgType = 'safe_navigation_msgs/ReplanInfo';
            obj.replanPub = rospublisher(pubTopicName,pubMsgType);

            % Subscriber that listens to trajectory messages.
            subTopicName = '/traj';
            subMsgType = 'safe_navigation_msgs/Trajectory';
            obj.trajSub = rossubscriber(subTopicName, subMsgType);
            pause(2); 
            
            obj.states = [];
            obj.controls = [];
        end
        
        %% Creates a replan request and replans.
        function [path, pathCtrls, newpath] = replan(obj, xcurr, ucurr, map, goal)
            % Construct replan message.
            replanMsg = obj.toReplanInfo(xcurr,ucurr,map,goal);
            
            % Publish replan request.
            %fprintf('waiting for replan...\n');
            send(obj.replanPub, replanMsg);
            
            % Get the trajectory.s
            msg = receive(obj.trajSub);
            fprintf('Got new trajectory of length %d!\n', length(msg.States));
            
            % Update the trajectory variables.
            stateCellArr = {};
            ctrlCellArr = {};
            for i=1:length(msg.States)
                stateCellArr{end+1} = msg.States(i).X;
            end
            for i=1:length(msg.Controls)
                ctrlCellArr{end+1} = msg.Controls(i).U;
            end
            obj.states = stateCellArr;
            obj.controls = ctrlCellArr;
            path = obj.states;
            pathCtrls = obj.controls;
            newpath = true;
        end
        
        %% Creates a ReplanRequest ROS message.
        function msg = toReplanInfo(obj,x,u,map,g)
            % NOTE: we need to capitalize all message fields for MATLAB
            %   ex. if the ROS message has a field called
            %           big_boi
            %       then in MATLAB it would be called
            %           BigBoi

            msg = rosmessage('safe_navigation_msgs/ReplanInfo');
            % Construct state msg.
            state = rosmessage('safe_navigation_msgs/State');
            state.X = x;
            % Construct goal msg.
            goal = rosmessage('safe_navigation_msgs/State');
            goal.X = g;
            % Construct control msg.
            control = rosmessage('safe_navigation_msgs/Control');
            control.U = u;
            % Construct nav_msgs/OccupancyGrid msg.
            grid = rosmessage('nav_msgs/OccupancyGrid');
            grid.Data = map(:); % flatten to 1D array
            [r,c] = size(map);
            grid.Info.Resolution = 1; % TODO: this needs to be actual res.
            grid.Info.Width = c;
            grid.Info.Height = r;

            % Populate ReplanInfo msg.
            msg.CurrX = state;
            msg.CurrU = control;
            msg.OccuMap = grid;
            msg.Goal = goal;
        end

    end
end

