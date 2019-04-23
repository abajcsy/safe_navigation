classdef SetAndOdomGrabberNode < handle
    %SETODOMGRABBERNODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        safeSetSub
        odomSub
        
        params
        
        stateCellArr
        setCellArr
        
        odomFreq
        prevOdomTime
        
        setFreq
        prevSetTime
    end
    
    methods
        %% Constructor.
        function obj = SetAndOdomGrabberNode()
            clf
            rmpath(genpath('/home/somilb/Documents/MATLAB/helperOC/'));
            addpath(genpath('/home/somilb/Documents/MATLAB/helperOC_dev/'));
            addpath(genpath('/home/somilb/Documents/Projects/visual_mpc/safe_navigation_ws/src/matlab_gen/'));
            addpath(genpath('/home/somilb/Documents/Projects/safe_navigation/'));
            
            % Load up all the experimental parameters.
            obj.params = car4DLocalQCameraSLAM();   
            
            % Create a new node.
            rosinit 
                    
            obj.stateCellArr = {};
            obj.setCellArr = {};
            
            obj.odomFreq = 0;
            obj.prevOdomTime = rostime('now');

            obj.setFreq = 0;
            obj.prevSetTime = rostime('now');

            % Create subscribers.
            obj.registerCallbacks()
            
            startTime = rostime('now');
            totalTime = 15;
            elapsedTime = 0;
            
            % Spin and let everything go.
            rate = rosrate(100);
            reset(rate);
            while elapsedTime < totalTime
                waitfor(rate);
                currTime = rostime('now');
                elapsedTime = seconds(currTime) - seconds(startTime);
            end
                    
            stateCellArr = obj.stateCellArr;
            setCellArr = obj.setCellArr;
            save('setAndOdom.mat', 'stateCellArr', 'setCellArr');
        end
        
        %% Create all the publishers and subscribers.
        function registerCallbacks(obj)
            % Subscriber for current state.
            odomMsgType = 'nav_msgs/Odometry';
            odomTopicName = '/odom';
            obj.odomSub = rossubscriber(odomTopicName, ...
                odomMsgType, @obj.odomCallback);
            pause(1); % wait for subscriber to get called
            
            % Subscriber for current safe set.
            safeMsgType = 'safe_navigation_msgs/SafeSet';
            safeSetTopicName = '/safe_set';
            obj.safeSetSub = rossubscriber(safeSetTopicName, safeMsgType, @obj.safeSetCallback);
            pause(1); % wait for subscriber to get called
        end
        
        %% Odometry callback
        function odomCallback(obj, ~, msg)
            x = msg.Pose.Pose.Position.X;
            y = msg.Pose.Pose.Position.Y;
            
            % Convert from quaternion to angle.
            quat = [msg.Pose.Pose.Orientation.X, msg.Pose.Pose.Orientation.Y, ...
                msg.Pose.Pose.Orientation.Z, msg.Pose.Pose.Orientation.W];
            eulerAngle = quat2eul(quat);
            theta = wrapToPi(eulerAngle(3));
            vel = msg.Twist.Twist.Linear.X;
            
            % Save the current state.
            currState = [x;y;theta;vel];
            
            currTime = rostime('now');
            deltaTime = seconds(currTime) - seconds(obj.prevOdomTime);
            
            if deltaTime > obj.odomFreq
                % put it in array.
                obj.stateCellArr{end+1} = currState;
            end
        end
        
        %% Safe set callback
        function safeSetCallback(obj, ~, msg)
            dim = msg.Dim;                  % dimension of system for which safe set was computed
            N = msg.N;                      % discretization for each dim
            values = double(msg.Values);    % flattened 1D value function
            if dim == 4
                currSafeSet = reshape(values, [N(1), N(2), N(3), N(4)]);
            else
                error('Verifier node cannot accept a %dD safe set!\n', dim);
            end
            
            currTime = rostime('now');
            deltaTime = seconds(currTime) - seconds(obj.prevSetTime);
            
            if deltaTime > obj.setFreq
                % put it in array.
                obj.setCellArr{end+1} = currSafeSet;
            end
        end
    end
end

