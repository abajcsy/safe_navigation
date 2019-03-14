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
        deriv       % (ND array) represents gradients for most recent valueFun
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
            obj.deriv = [];

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
                if obj.params.useSafety
                    msg = receive(obj.safeSetSub);
                    obj.safeSetCallback(msg);
                end
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
            if obj.params.useSafety
                obj.safeSetSub = rossubscriber(safeSetTopicName, safeMsgType); %, @obj.safeSetCallback);
                msg = receive(obj.safeSetSub);
                obj.safeSetCallback(msg);
                pause(1); 
            end
            
            % Subscriber that listens to planned trajectory
            planMsgType = 'safe_navigation_msgs/Trajectory';
            obj.planSub = rossubscriber(obj.params.planTopicName, planMsgType, @obj.verifyPlanCallback);
            pause(1); 
        end
        
        %% Callback for when we get plans that need to be verified
        function verifyPlanCallback(obj, ~, msg)
            if ~obj.params.useSafety
                % just send out original message.
                fprintf('No safety: sending out original plan.\n');
                send(obj.verifiedPub, msg);
            else 
                if isempty(obj.valueFun)
                    % we haven't gotten first safe set from the safety updater
                    % so we can't verify plans yet.
                    fprintf('Have not received first safe set...\n');
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

    %             % just for visualization, store planned states.
    %             planPath = {xplan};
    %             for i=1:length(controls)
    %                 % (JUST FOR VISUALIZATION) simulate fwd with plan's
    %                 % control.
    %                 uplan = controls(i).U; 
    %                 obj.params.dynSys.updateState(uplan, obj.params.dt, xplan);
    %                 xplan = obj.params.dynSys.x;
    %                 planPath{end+1} = xplan;
    %             end

                % How close to zero does the spatial derivative have to be
                % to be considered = 0?
                gradZeroTol = 0.01;
                appliedZero = false;
                %fprintf('Verifying plan...\n');

                for i=1:length(controls) 
                    if appliedUOpt
                        % if we applied safety controller, then the 
                        % plan's sequence of controls is now invalid, so 
                        % we just use the safety control from here on.
                        u = obj.getSafetyControl(xcurr);
                        %if appliedZero
                        %    u = 0.0*u;
                        %end
                        %fprintf('usafe: [%f, %f]\n', u(1), u(2));
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
                            u = ucOpt.*(abs(deriv) > gradZeroTol) + u.*(abs(deriv) < gradZeroTol);
                            %if (abs(deriv) < gradZeroTol)
                            %    u = 0.0*u;
                            %    appliedZero = true;
                            %end
                            %u = uOpt;
                            appliedUOpt = true;
                            %fprintf('usafe: [%f, %f]\n', u(1), u(2));
                        end
                    end

                    % modify verified control sequence.
                    verifiedU{end+1} = u;

                    % simulate the trajectory forward (applying safe or planned
                    % control)
                    obj.params.dynSys.updateState(u, obj.params.dt, xcurr);
                    xcurr = obj.params.dynSys.x;
                end

                % construct verified trajectory message
                verifiedMsg = obj.toTrajMsg(verifiedX0, verifiedU);

                % publish out the verified trajectory
                send(obj.verifiedPub, verifiedMsg);

                fprintf('Sent verified plan!\n');
            end
            %rate = rosrate(100);
            %waitfor(rate);
        end
        
        %% Unpacks most recent safe set and stores it.
        function safeSetCallback(obj, msg)
            % Note: we assume that the value function that was 3D was
            % flattened by doing reshape(arrVx, [1,prod(N)]) so that it can
            % be reconstructed by doing reshape(flatVx, N)
            
            dim = msg.Dim;          %  dimension of system for which safe set was computed
            N = msg.N;              % discretization for each dim: e.g. N = [31, 31, 21] for 3D system. 
            values = double(msg.Values); % flattened 1D value function
            if dim == 4
                newValueFun = reshape(values, [N(1), N(2), N(3), N(4)]);
            else
                error('Verifier node cannot accept a %dD safe set!\n', dim);
            end
            
            fprintf('Got safe set message!\n');
            oldValueFun = obj.valueFun;
            isSame = isequal(oldValueFun, newValueFun);
            if ~isSame
                fprintf('----> safe set is NEW!\n');
            end
            obj.valueFun = newValueFun;
            % compute the corresponding gradients and store them.
            obj.deriv = computeGradients(obj.params.grid, obj.valueFun);
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
            %fprintf('------------------\n');
            %tic;
            vx = obj.valueFun;
            value = eval_u(obj.params.grid, vx, x);
            %t1 = toc;
            
            % If the value is close to zero, we are close to the safety
            % boundary.
            if value < tol 
                % value of the derivative at that particular state
                current_deriv = eval_u(obj.params.grid, obj.deriv, x);
                %t2 = toc;
                % NOTE: need all 5 arguments (including NaN's) to get 
                % correct optimal control!
                uOpt = obj.params.dynSys.optCtrl(NaN, x, current_deriv, obj.params.uMode, NaN); 
                %t3 = toc;
                onBoundary = true;
                if iscell(uOpt)
                    uOpt = cell2mat(uOpt);
                end

                %fprintf('deriv computation: %f\n', t2-t1);
                %fprintf('optCtrl computation: %f\n', t3-t2);
            else
                current_deriv = [0.0;0.0;0.0;0.0];
                uOpt = zeros(length(x), 1);
                onBoundary = false;
            end
            %t4 = toc;
            
            %fprintf('value computation: %f\n', t1);
            %fprintf('overall computation: %f\n', t4);
        end
        
        %% Gets the optimal control to apply at state x.
        function uOpt = getSafetyControl(obj, x)
            vx = obj.valueFun;
            % value of the derivative at that particular state
            current_deriv = eval_u(obj.params.grid, obj.deriv, x);
            % NOTE: need all 5 arguments (including NaN's) to get 
            % correct optimal control!
            uOpt = obj.params.dynSys.optCtrl(NaN, x, current_deriv, obj.params.uMode, NaN);
            uOpt = cell2mat(uOpt);
        end
    end
end

