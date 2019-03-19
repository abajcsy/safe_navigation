classdef VisualizationNode < handle
    %VISUALIZATIONNODE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Subscribers.
        odomSub
        mapSub
        planSub
        safeSetSub
        
        % Experimental parameters
        params
        
        % Internal variables.
        currState
        currMap
        currPlan
        currSafeSet
        currDeriv
        
        % Figure handles
        figh
        envh
        senseh
        carh
        vxh
        trajh
    end
    
    methods
        
        %% Constructor
        function obj = VisualizationNode()
            clf
            rmpath(genpath('/home/somilb/Documents/MATLAB/helperOC/'));
            addpath(genpath('/home/somilb/Documents/MATLAB/helperOC_dev/'));
            addpath(genpath('/home/somilb/Documents/Projects/visual_mpc/safe_navigation_ws/src/matlab_gen/'));
            addpath(genpath('/home/somilb/Documents/Projects/safe_navigation/'));
            
            % Load up all the experimental parameters.
            obj.params = car4DLocalQCameraSLAM();   % run SLAM environment
            
            % HACK -- normalize by vehicle position
            %vehiclePos = [2.1;2.1];
            %obj.params.lowEnv = obj.params.lowEnv - vehiclePos(1:2);
            %obj.params.upEnv = obj.params.upEnv - vehiclePos(1:2);
            %obj.params.grid.xs{1} = obj.params.grid.xs{1} - vehiclePos(1);
            %obj.params.grid.xs{2} = obj.params.grid.xs{2} - vehiclePos(2);
            
            % Initialize internal vars.
            obj.currState = [];
            obj.currMap = [];
            obj.currPlan = [];
            obj.currSafeSet = [];
            
            % Figure handles.
            obj.envh = [];
            obj.senseh = [];
            obj.carh = [];
            obj.vxh = [];
            obj.trajh = [];
            obj.figh = figure(1);
            
            % Visualize goal region.
            hold on
            c = [0.1,0.8,0.5,0.5];
            pos = [obj.params.xgoal(1)-obj.params.goalEps, obj.params.xgoal(2)-obj.params.goalEps, ...
                obj.params.goalEps*2, obj.params.goalEps*2];
            rectangle('Position',pos,'Curvature',1.0,'FaceColor',c,'LineStyle','none');
            scatter(obj.params.xgoal(1),obj.params.xgoal(2),[],[0.0,0.8,0.5],'filled');

            % Setup size of figure and tick mark stuff.
            set(gca,'XTick',[obj.params.lowEnv(1) obj.params.upEnv(1)]);
            set(gca,'YTick',[obj.params.lowEnv(2) obj.params.upEnv(2)]);
            widthMeters = abs(obj.params.lowEnv(1) - obj.params.upEnv(1));
            heightMeters = abs(obj.params.lowEnv(2) - obj.params.upEnv(2));
            set(gcf, 'Position',  [100, 100, widthMeters*100*0.6, heightMeters*100*0.6])
            axis tight;
            
            % If we are running SLAM, we are using the turtlebot, 
            % so we need to set the ROS master uri to the turtlebot.
            if strcmp(obj.params.envType, 'slam')
                setenv('ROS_MASTER_URI','http://128.32.38.83:11311');
            else
                 error('The safety update node is not set up for any environment other than SLAM!\n');
            end
            
            % Create a new node.
            rosinit 
            
            % Create subscribers.
            obj.registerCallbacks()
            
            % Spin and let everything go.
            rate = rosrate(100);
            reset(rate);
            while true
                waitfor(rate);
            end
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
           
            % Subscriber that listens to planned trajectory
            planMsgType = 'safe_navigation_msgs/Trajectory';
            obj.planSub = rossubscriber(obj.params.planTopicName, planMsgType, @obj.planCallback);
            pause(1); % wait for subscriber to get called
        end
       
        function savePlot(obj, prefix)
          currTime = datestr(now, 'dd-mm-yyyy-HH:MM:SS:FFF');
          filename = strcat('img/', currTime, '_', prefix, '.png');
          saveas(obj.figh, filename);
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
            obj.currState = [x;y;theta;vel];
            
            % Update the plotting for where the car is right now.
            if ~isempty(obj.carh)
                delete(obj.carh{2});
            end
            obj.carh = obj.plotCar(obj.currState, false);

            % If there is a value function to visualize, do so.
            if ~isempty(obj.currSafeSet) && ~isempty(obj.currState)
                visSet = true;
                if length(obj.currState) == 3
                    extraArgs.theta = obj.currState(3);
                elseif length(obj.currState) == 4
                    extraArgs.theta = obj.currState(3);
                    extraArgs.vel = obj.currState(4);
                else
                    error('I cannot visualize a %dD system!', length(obj.currState));
                end

                if ~isempty(obj.vxh)
                    delete(obj.vxh);
                end
                obj.vxh = obj.plotFuncLevelSet(obj.params.grid, obj.currSafeSet, visSet, extraArgs);
                
                % If the current state is on the boundary, then 
                % change the plotting to reflect that.
                [~, onBoundary, ~] = ...
                    obj.checkAndGetSafetyControl(obj.currState, obj.params.safetyTol);
                
                if onBoundary
                    % Update the plotting for where the car is right now.
                    if ~isempty(obj.carh)
                        delete(obj.carh{2});
                    end
                    obj.carh = obj.plotCar(obj.currState, true);
                end
            end
        end
        
        %% Safe set callback
        function safeSetCallback(obj, ~, msg)
            dim = msg.Dim;                  % dimension of system for which safe set was computed
            N = msg.N;                      % discretization for each dim
            values = double(msg.Values);    % flattened 1D value function
            if dim == 4
                obj.currSafeSet = reshape(values, [N(1), N(2), N(3), N(4)]);
            else
                error('Verifier node cannot accept a %dD safe set!\n', dim);
            end
            
            % compute the corresponding gradients and store them.
            obj.currDeriv = computeGradients(obj.params.grid, obj.currSafeSet);
            %obj.savePlot('safeset');
        end
        
        %% Plan callback
        function planCallback(obj, ~, msg)
            if length(msg.States) == 1
                x0 = msg.States.X; % we only get initial state
            elseif length(msg.States) > 1
                x0 = msg.States(1).X; % extract initial state
            else
                error('Did not get enough states! Got %d', length(msg.States));
            end
            controls = msg.Controls; % extract safe_navigation_msgs/Control[] 
            
            x0(3) = wrapToPi(x0(3)); % make sure we wrap to [-pi, pi]
            xplan = x0;
            
            % TODO -- i might need to update the dynSys initial state.
            
            % Just for visualization, store planned states.
            planPath = {xplan};
            for i=1:length(controls)
                % Simulate fwd with plan's control.
                uplan = controls(i).U; 
                obj.params.dynSys.updateState(uplan, obj.params.dt, xplan);
                xplan = obj.params.dynSys.x;
                planPath{end+1} = xplan;
            end
            
            % Plot the most recent trajectory.
            obj.plotTraj(planPath);
          end
        
        %% Checks if state x is at the safety boundary. If it is, returns
        %  the optimal safety control to take. 
        function [uOpt, onBoundary, current_deriv] = checkAndGetSafetyControl(obj, x, tol)
            % Grab the value at state x from the most recent converged 
            % value function.
            vx = obj.currSafeSet;
            value = eval_u(obj.params.grid, vx, x);
            fprintf('tol: %f, value: %f\n', tol, value);
            
            % If the value is close to zero, we are close to the safety
            % boundary.
            if value < tol 
                % value of the derivative at that particular state
                current_deriv = eval_u(obj.params.grid, obj.currDeriv, x);
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
        
        
        %% Plots optimal path
        function plotTraj(obj, path)
            if ~isempty(obj.trajh)
                for i=1:length(obj.trajh)
                    delete(obj.trajh(i));
                end
            end
            obj.trajh = [];
            i = 1;
            xvals = [];
            yvals = [];
            while i < length(path)
                x = path{i};
                xvals = [xvals, x(1)];
                yvals = [yvals, x(2)];
                xnext = path{i+1};
                % plot the line segments.
                h = line([x(1), xnext(1)], [x(2), xnext(2)], 'Color', 'blue', 'LineWidth', 1.5);
                obj.trajh = [obj.trajh, h];
                i = i+1;
            end
            
            if length(path) ~= 0
              % get final waypt.
              x = path{i};
              xvals = [xvals, x(1)];
              yvals = [yvals, x(2)];
              % plot the waypts.
              s = scatter(xvals, yvals, 15, 'b', 'filled');
              obj.trajh = [obj.trajh, s];
            end
        end
        
        %% Plot level set of arbitrary function
        % Inputs:
        %   g [array]     - grid corresponding to data
        %   func [array]  - data for function to visualize
        %   visSet [bool] - if true, plots 2D slice of func.
        %                   Otherwise plots 3D.
        %   extraArgs
        %           .theta [float] - (if 3D system) angle for which to 
        %                           plot the level set
        %           .vel [float]   - (if 4D system) velocity for which
        %                           to plot the level set
        %           .edgeColor [vector or string] - color of level set boundary
        %           .cmap [string] - name of colormap to use
        % Outputs: 
        %   plots level set in (x,y) for fixed theta.
        function h = plotFuncLevelSet(obj, g, func, visSet, extraArgs)
            
            if isfield(extraArgs, 'theta')
                if isfield(extraArgs, 'vel')
                    % (4D system) Grab slice at theta & vel.
                    [gPlot, dataPlot] = proj(g, func, [0 0 1 1], [extraArgs.theta, extraArgs.vel]);
                else
                    % (3D system) Grab slice at theta.
                    [gPlot, dataPlot] = proj(g, func, [0 0 1], extraArgs.theta);
                end
            else
                % (2D system) Plot complete value function.
                gPlot = g;
                dataPlot = func;
            end
            
            % grab the edge color
            if isfield(extraArgs, 'edgeColor')
                edgeColor = extraArgs.edgeColor;
            else
                edgeColor = [1,0,0];
            end
            
            % grab the color map
            if isfield(extraArgs, 'cmap')
                cmap = extraArgs.cmap;
            else
                cmap = 'gray';
            end
            
            % Visualize final set.
            % NOTE: plot -data if using contourf plots because it uses all values
            % that are ABOVE zero, but inside our obstacle we have values
            % BELOW zero.
            if visSet
                visExtraArgs.LineWidth = 2.0;
                h = visSetIm(gPlot, dataPlot, edgeColor, 0:0.1:obj.params.safetyTol, visExtraArgs);
                %[~, h] = contourf(gPlot.xs{1}, gPlot.xs{2}, dataPlot, 0:0.1:5);
            else
                alpha = 0.5;
                h = visFuncIm(gPlot, dataPlot, edgeColor, alpha); %, edgeColor, 0.5);
                xlabel('V(x)');
            end

            colormap(flipud(cmap));
            xlabel('$p_x$', 'Interpreter','latex');
            ylabel('$p_y$', 'Interpreter','latex');
            grid on
            set(gca,'TickLength',[0 0]);
        end
        
        %% Plots dubins car point and heading.
        % Inputs:
        %   x [vector]  - 3D/4D state of dubins car
        % Ouput:
        %   c   - handle for figure
        function c = plotCar(obj, x, usedUOpt)
            % color car red if we applied safe control
            if usedUOpt
                carColor = 'r';
            else
                carColor = 'k';
            end
            c = {};
            if ~isempty(obj.carh)
                if isequal(obj.carh{1}.MarkerFaceColor, [0, 0, 0]) 
                    set(obj.carh{1}, 'Color', [0.7,0.7,0.7]);
                    set(obj.carh{1}, 'MarkerFaceColor', [0.7,0.7,0.7]);
                    set(obj.carh{1}, 'MarkerEdgeColor', [0.7,0.7,0.7]);
                else
                    set(obj.carh{1}, 'Color', [1.0,0.5,0.5]);
                    set(obj.carh{1}, 'MarkerFaceColor', [1.0,0.5,0.5]);
                    set(obj.carh{1}, 'MarkerEdgeColor', [1.0,0.5,0.5]);
                end
            end
            c{1} = plot(x(1), x(2), 'ko','MarkerSize', 5, ...
                'MarkerEdgeColor', carColor, 'MarkerFaceColor', carColor);
            
            % Plot heading.
            center = x(1:2);
            
            if length(x) >= 3
                % Rotation matrix.
                R = [cos(x(3)) -sin(x(3)); 
                     sin(x(3)) cos(x(3))];
                % Heading pt.
                hpt = [0.5; 0];
                hptRot = R*hpt + center;
                p2 = plot([center(1) hptRot(1)], [center(2) hptRot(2)], carColor, 'LineWidth', 1.5);
                p2.Color(4) = 1.0;
                c{2} = p2;
            end
            
            % Setup the figure axes to represent the entire environment
            xlim([obj.params.lowEnv(1) obj.params.upEnv(1)]);
            ylim([obj.params.lowEnv(2) obj.params.upEnv(2)]);
        end
    end
end

