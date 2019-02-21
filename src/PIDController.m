classdef PIDController < handle
    %PIDCONTROLLER
    
    properties
        path
        dynSys
        nextWayptIdx
        K
        ff
    end
    
    methods
        function obj = PIDController(dynSys)
            obj.dynSys = dynSys;
            
            % Proportional gain matrix and feed-forward term
            obj.K = [1 1 0; 0 0 1];
            obj.ff = [0;0];
        end
        
        %% Updates internal path variable and sets up the next waypt to ctrl to
        function updatePath(obj, path)
            obj.path = path;
            if length(path) > 1
                obj.nextWayptIdx = 2;
            else
                obj.nextWayptIdx = 1;
            end
        end
        
        %% Gets a PID control to track the path
        % Inputs:
        %   x       -- (array) current state of robot
        %   path    -- (cell array) series of waypoints to goal
        function u = getControl(obj, t, x)
            % find which waypoint is closest to our current position.
            %xystar = obj.getNextWaypt(x);
            xystar = obj.getTimedWaypt(t);
            h = scatter(xystar(1), xystar(2), 'r', 'filled');
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

%             fprintf('error theta: %f\n', errTheta);
%             fprintf('u1: %f\n', u(1));
%             fprintf('u2: %f\n', u(2));
%             fprintf('\n');
        end

        %% Gets reference point from trajectory in time-parametrized way.
        % todo: have an error where xref isnt assigned ...
        function xref = getTimedWaypt(obj, t)
            T = 200;
            tLen = T/length(obj.path);
            if t > T
                xref = obj.path{end};
                return;
            end
            times = [1];
            for i=1:length(obj.path)-1
                times = [times, tLen*i];
            end
            % parametrize the trajectory in time.
            for j=2:length(times)
                prevt = times(j-1);
                nextt = times(j);
                if t >= prevt && t < nextt
                    prevpt = obj.path{j-1};
                    nextpt = obj.path{j};
                    xref = (nextpt - prevpt)*((t-prevt)/(nextt - prevt)) + prevpt;
                    return;
                end
            end
            % hack.
            xref = obj.path{end};
        end

        %% Helper function get closest waypoint along path to query point x.
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

