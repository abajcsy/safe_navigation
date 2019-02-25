classdef PIDController < handle
    %PIDCONTROLLER
    
    properties
        path            % (cell arr) waypoints
        dynSys          % (obj) dynamical system (dubins car)
        dt              % (float) time step
        nextWayptIdx    % (int) idx into next candidate waypt
        K               % (matrix) 2x2 gain matrix
        ff              % (vector) 2x1 feed-forward term
        totalT          % (int) total time to execute path
        wayptTimes      % (arr) timestamps for each waypoint
    end
    
    methods
        %% Constructor.
        function obj = PIDController(dynSys, dt)
            obj.dynSys = dynSys;
            obj.dt = dt;
            
            % Proportional gain matrix and feed-forward term
            obj.K = [1 1 0; 0 0 1];
            obj.ff = [0;0];
        end
        
        %% Updates internal path variable and sets up the next waypt to ctrl to
        function updatePath(obj, path, startT, newPath)
            % reset the next candidate waypt
            obj.path = path;
            if length(obj.path) > 1
                obj.nextWayptIdx = 2;
            else
                obj.nextWayptIdx = 1;
            end
            
            if newPath
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
                obj.totalT = dTotal/obj.dt;

                % re-time the trajectory
                obj.wayptTimes = [startT];
                for i=2:length(obj.path)
                    newTime = obj.wayptTimes(i-1) + obj.totalT*(dists(i-1)/dTotal);
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
            h = scatter(xystar(1), xystar(2), 'c', 'filled');
            h.MarkerFaceAlpha = 0.2;

            % Rotation matrix.
            R = [cos(x(3)) sin(x(3)); 
                 -sin(x(3)) cos(x(3))];
            errXY = R*(xystar - x(1:2));
            errTheta = atan2(errXY(2), errXY(1));

            % Compute error.
            error = [errXY; errTheta];

            % compute u = [v, omega]
            u = obj.K*error + obj.ff;

            % clip control based on bounds
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
            u = transpose(u);
        end

        %% Get reference point from trajectory in time-parametrized way.
        function xref = getTimedWaypt(obj, t)
            if t > obj.totalT
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

        %% Get closest waypoint along path to query point x.
        function xClosest = getNextWaypt(obj, x)
            xClosest = obj.path{obj.nextWayptIdx};
            
            distEps = 0.1;
            % if we got close enough to the next waypoint, choose new one
            if norm(xClosest - x(1:2)) < distEps
                obj.nextWayptIdx = obj.nextWayptIdx + 1;
                % if we are at the goal
                if obj.nextWayptIdx > length(obj.path)
                    obj.nextWayptIdx = length(obj.path);
                end
            end
        end
        
    end
end

