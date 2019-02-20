%% Gets a PID control to track the path
% Inputs:
%   x       -- (array) current state of robot
%   path    -- (cell array) series of waypoints to goal
function u = getPIDControl(t, x, path, dynSys)
    % find which waypoint is closest to our current position.
    xystar = getClosestWaypt(x,path);
    %xystar = getRefWaypt(t,path);
    h = scatter(xystar(1), xystar(2), 'r', 'filled');
    h.MarkerFaceAlpha = 0.2;
    
    % Proportional gain matrix
    Kp = [0.1 0.1 0; 0 0 0.5];
    ff = [0;0];
    
	% Rotation matrix.
    R = [cos(x(3)) -sin(x(3)); 
         sin(x(3)) cos(x(3))];
    xyopt = R*(xystar - x(1:2));
    thetaopt = atan2(xyopt(2), xyopt(1));
    optpt = [xyopt; thetaopt];
    
    % Compute error
    error = optpt - x;
    
    % compute u = [v, omega]
    u = Kp*error + ff;
    
    % clip control based on bounds
    if u(1) > dynSys.vrange(2)
        u(1) = dynSys.vrange(1);
    elseif u(1) < dynSys.vrange(1)
        u(1) = dynSys.vrange(2);
    end
    
    if u(2) > dynSys.wMax
        u(2) = dynSys.wMax;
    elseif u(2) < -dynSys.wMax
        u(2) = -dynSys.wMax;
    end
    u = transpose(u);
    
    fprintf('relative angle: %f\n', thetaopt);
    fprintf('u1: %f\n', u(1));
    fprintf('u2: %f\n', u(2));
    fprintf('\n');
end

% Gets reference point from trajectory in time-parametrized way.
% todo: have an error where xref isnt assigned ...
function xref = getRefWaypt(t,path)
    T = 200;
    tLen = T/length(path);
    if t > T
        xref = path{end};
        return;
    end
    times = [1];
    for i=1:length(path)-1
        times = [times, tLen*i];
    end
    % parametrize the trajectory in time.
    for j=2:length(times)
        prevt = times(j-1);
        nextt = times(j);
        if t >= prevt && t < nextt
            prevpt = path{j-1};
            nextpt = path{j};
            xref = (nextpt - prevpt)*((t-prevt)/(nextt - prevt)) + prevpt;
            return;
        end
    end
    % hack.
    xref = path{end};
end

% Helper function get closest waypoint along path to query point x.
function xClosest = getClosestWaypt(x,path)
    xClosest = [];
    minDist = Inf;
    for i=1:length(path)
        d = norm(path{i} - x(1:2));
        % avoid corner case where query point is closest point
        if d < minDist && d > 0
            minDist = d;
            xClosest = path{i};
        end
    end
end