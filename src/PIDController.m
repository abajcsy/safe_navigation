classdef PIDController < handle
    %PIDCONTROLLER
    
    properties
        path            % (cell arr) waypoints
        dynSys          % (obj) dynamical system (dubins car)
        dt              % (float) time step
        K               % (matrix) 2x2 gain matrix
        ff              % (vector) 2x1 feed-forward term
        wayptTimes      % (arr) timestamps for each waypoint
    end
    
    methods
        %% Constructor.
        function obj = PIDController(dynSys, dt)
            obj.dynSys = dynSys;
            obj.dt = dt;
            
            % Proportional gain matrix and feed-forward term
            if dynSys.nx == 3
                obj.K = [1 1 0; 
                         0 0 1];
            elseif dynSys.nx == 4
                % TODO: tune these gains.
                obj.K = [0 0 1 0; 
                         -1 -1 0 1];
            else
                error('Cannot PID control for %dD system!', dynSys.nx);
            end
            obj.ff = [0;0];
        end
        
        %% Updates internal path variable and sets up the next waypt to ctrl to
        function updatePath(obj, path, startT, newPath)
            if newPath
                % store new path.
                obj.path = path;
                
                % compute lengths of each segment in path
                dTotal = 0;
                dists = [];
                for i=1:length(obj.path)-1
                    prevPt = obj.path{i};
                    nextPt = obj.path{i+1};
                    d = norm(nextPt - prevPt);
                    dTotal = dTotal + d;
                    dists = [dists, d];
                end
                % total time to execute path
                totalT = dTotal/obj.dt;

                % re-time the trajectory
                obj.wayptTimes = [startT];
                for i=2:length(obj.path)
                    newTime = obj.wayptTimes(i-1) + totalT*(dists(i-1)/dTotal);
                    obj.wayptTimes = [obj.wayptTimes, newTime];
                end
            end
        end
        
        %% Gets a PID control to track the path
        % Inputs:
        %   t       -- (int) current simulation time
        %   x       -- (array) current state of robot
        function u = getControl(obj, t, x)
            %xystar = obj.getNextWaypt(x);
            xystar = obj.getTimedWaypt(t);
            %h = scatter(xystar(1), xystar(2), 'c', 'filled');
            %h.MarkerFaceAlpha = 0.2;

            % Rotation matrix.
            R = [cos(x(3)) sin(x(3)); 
                 -sin(x(3)) cos(x(3))];
            errXY = R*(xystar - x(1:2));
            errTheta = atan2(errXY(2), errXY(1));
            
            % Compute error.
            error = [errXY; errTheta];
            
            % If we have 4D system, need to add error in acceleration
            if obj.dynSys.nx == 4
                error = [error; x(4)];
            end

            % compute u = [v, omega]
            u = obj.K*error + obj.ff;

            % clip control based on bounds
            if obj.dynSys.nx == 3
                if u(1) > obj.dynSys.vrange(2)
                    u(1) = obj.dynSys.vrange(2);
                elseif u(1) < obj.dynSys.vrange(1)
                    u(1) = obj.dynSys.vrange(1);
                end

                if u(2) > obj.dynSys.wMax
                    u(2) = obj.dynSys.wMax;
                elseif u(2) < -obj.dynSys.wMax
                    u(2) = -obj.dynSys.wMax;
                end
            else % for 4D system
                if u(1) > obj.dynSys.wMax
                    u(1) = obj.dynSys.wMax;
                elseif u(1) < -obj.dynSys.wMax
                    u(1) = -obj.dynSys.wMax;
                end

                if u(2) > obj.dynSys.aRange(2)
                    u(2) = obj.dynSys.aRange(2);
                elseif u(2) < obj.dynSys.aRange(1)
                    u(2) = obj.dynSys.aRange(1);
                end
            end
            u = transpose(u);
        end

        %% Get reference point from trajectory in time-parametrized way.
        function xref = getTimedWaypt(obj, t)
            if t > obj.wayptTimes(end)
                xref = obj.path{end};
                return;
            end
            % parametrize the trajectory in time.
            for i=1:length(obj.wayptTimes)-1
                prevt = obj.wayptTimes(i);
                nextt = obj.wayptTimes(i+1);
                if t >= prevt && t < nextt
                    prevpt = obj.path{i};
                    nextpt = obj.path{i+1};
                    xref = (nextpt - prevpt)*((t-prevt)/(nextt - prevt)) + prevpt;
                    return;
                end
            end
            % hack.
            xref = obj.path{end};
        end
        
    end
end

