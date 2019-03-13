classdef VerifierNode < handle
    %VERIFIERNODE Communicates with NN planner and verifies plans
    % using the safe set computed and maintained by SafetyUpdateNode.
    
    properties
        % Publishers and subscribers.
        planSub
        safeSetSub
        verifiedPub
        
        params      % (struct) maintains all the information about experiment we are running.
        valueFun    % (ND array) most recent value function encoding the safe set.
    end
    
    methods
        %% Constructor
        function obj = VerifierNode()
            clf
            rmpath(genpath('/home/somilb/Documents/MATLAB/helperOC/'));
            addpath(genpath('/home/somilb/Documents/MATLAB/helperOC_dev/'));
            addpath(genpath('/home/somilb/Documents/Projects/visual_mpc/safe_navigation_ws/src/matlab_gen/'));
            addpath(genpath('/home/somilb/Documents/Projects/safe_navigation/'));

            % Load up all the experimental parameters for SLAM environment.
            obj.params = car4DLocalQCameraSLAM();   
            
            % Most recent value function from safety updater.
            obj.valueFun = [];

            % If we are running SLAM, we are using the turtlebot, 
            % so we need to set the ROS master uri to the turtlebot.
            if strcmp(obj.params.envType, 'slam')
                setenv('ROS_MASTER_URI','http://128.32.38.83:11311');
            else
                error('The verifier is not set up for any environment other than SLAM!\n');
            end

            % Create a new node.
            rosinit   
            
             % Setup ROS pub/sub for trajectories to verify.
            obj.registerCallbacks();
            
            % Spin and let everything go.
            rate = rosrate(100);
            reset(rate);
            while true
                waitfor(rate);
            end
        end
        
        %% Sets up all the publishers and subscribers.
        function registerCallbacks(obj)
            % Publisher that publishes out verified trajectory.
            verifiedMsgType = 'safe_navigation_msgs/Trajectory';
            obj.verifiedPub = rospublisher(obj.params.verifiedTopicName,verifiedMsgType);
            
            % Subscriber that listens to planned trajectory
            safeMsgType = 'safe_navigation_msgs/SafeSet';
            safeSetTopicName = '/safe_set';
            obj.planSub = rossubscriber(safeSetTopicName, safeMsgType, @obj.safeSetCallback);
            pause(1); 

            % Subscriber that listens to planned trajectory
            planMsgType = 'safe_navigation_msgs/Trajectory';
            obj.planSub = rossubscriber(obj.params.planTopicName, planMsgType, @obj.verifyPlanCallback);
            pause(1); 
        end
        
        %% Callback for when we get plans that need to be verified
        function verifyPlanCallback(obj, ~, msg)
            if isempty(obj.valueFun)
                % we haven't gotten first safe set from the safety updater
                % so we can't verify plans yet.
                fprintf('Have not received first safe set...');
                return;
            end
            
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
                    u = obj.getSafetyControl(xcurr);
                else
                    % grab planner's current control.
                    u = controls(i).U; 
                    
                    % check the safety of the state we reach by applying control
                    [uOpt, onBoundary, deriv] = ...
                        obj.checkAndGetSafetyControl(xcurr, obj.params.safetyTol);
                    % extract just the theta and velocity derivative;
                    deriv = deriv(3:4);
                    % if state we reach is unsafe
                    if onBoundary
                        %u = uOpt.*(abs(deriv) > gradZeroTol) + u.*(abs(deriv) < gradZeroTol);
                        u = uOpt;
                        appliedUOpt = true;
                    end
                end

                % modify verified control sequence.
                verifiedU{end+1} = u;
                
            	% simulate the trajectory forward (applying safe or planned
                % control)
                obj.params.dynSys.updateState(u, obj.params.dt, xcurr);
                xcurr = obj.params.dynSys.x;
                
                % update plotting
                
                % TODO -- need to add the proper things here.
                gfmm = []; %obj.map.gFMM;
                occu_map_safe = []; %obj.map.occupancy_map_safety;
                obj.plotter.updatePlot(xcurr, obj.params.xgoal, obj.valueFun, ...
                    obj.params.grid, gfmm, occu_map_safe, planPath, appliedUOpt);
                
                % TODO -- consider taking this pause out for speed.
                pause(obj.params.dt);
            end
            
            % construct verified trajectory message
            verifiedMsg = obj.toTrajMsg(verifiedX0, verifiedU);
            
            % publish out the verified trajectory
            send(obj.verifiedPub, verifiedMsg);
        end
        
        %% Unpacks most recent safe set and stores it.
        function safeSetCallback(obj, ~, msg)
            % Note: we assume that the value function that was 3D was
            % flattened by doing reshape(arrVx, [1,prod(N)]) so that it can
            % be reconstructed by doing reshape(flatVx, N)
            
            dim = msg.Dim;          %  dimension of system for which safe set was computed
            N = msg.N;              % discretization for each dim: e.g. N = [31, 31, 21] for 3D system. 
            values = double(msg.Values); % flattened 1D value function
            obj.valueFun = reshape(values, N);
        end
        
        %% Converts from array to trajectory message
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
        
        
        %% Checks if state x is at the safety boundary. If it is, returns
        %  the optimal safety control to take. 
        function [uOpt, onBoundary, current_deriv] = checkAndGetSafetyControl(obj, x, tol)
            % Grab the value at state x from the most recent converged 
            % value function.
            vx = obj.valueFun;
            value = eval_u(obj.params.grid, vx, x);
            
            % If the value is close to zero, we are close to the safety
            % boundary.
            if value < tol 
                deriv = computeGradients(obj.params.grid, vx);
                % value of the derivative at that particular state
                current_deriv = eval_u(obj.params.grid, deriv, x);
                % NOTE: need all 5 arguments (including NaN's) to get 
                % correct optimal control!
                uOpt = obj.params.dynSys.optCtrl(NaN, x, current_deriv, obj.params.uMode, NaN); 
                onBoundary = true;
                if iscell(uOpt)
                    uOpt = cell2mat(uOpt);
                end
            else
                current_deriv = [0.0;0.0;0.0;0.0];
                uOpt = zeros(length(x), 1);
                onBoundary = false;
            end
        end
        
        %% Gets the optimal control to apply at state x.
        function uOpt = getSafetyControl(obj, x)
            vx = obj.valueFun;
            deriv = computeGradients(obj.params.grid, vx);
            % value of the derivative at that particular state
            current_deriv = eval_u(obj.params.grid, deriv, x);
            % NOTE: need all 5 arguments (including NaN's) to get 
            % correct optimal control!
            uOpt = obj.params.dynSys.optCtrl(NaN, x, current_deriv, obj.params.uMode, NaN);
            uOpt = cell2mat(uOpt);
        end
    end
end

