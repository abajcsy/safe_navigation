function slam_listener_node()
    rosinit 
    clear all
    close all
    
    % Subscriber that listens to occupancy grid.
    occuMapMsgType = 'nav_msgs/OccupancyGrid';
    occuMapSub = rossubscriber('/map', occuMapMsgType, @mapCallback);
    
    hold on;
    figure(1);
    % Spin and let everything go.
    rate = rosrate(100);
    reset(rate);
    while true
        waitfor(rate);
    end
end

function mapCallback(~, msg)
    res = double(msg.Info.Resolution);
    r = msg.Info.Height;
    c = msg.Info.Width;
    realHeight = double(r)*double(res);
    realWidth = double(c)*double(res);
    origin = [double(msg.Info.Origin.Position.X), double(msg.Info.Origin.Position.Y)];

    fprintf('Got SLAM occupancy map message with info:\n');
    fprintf('     realH: %f, # rows: %d\n', realHeight, r);
    fprintf('     realW: %f, # cols: %d\n', realWidth, c);
    fprintf('     origin: (%d, %d)\n', origin(1), origin(2));
    
    % Grab the SLAM-generated occupancy map.
    % Convention for SLAM map values:
    %   > 0     -- sensed obstacle
    %   = 0     -- sensed free-space
    %   < 0     -- unsensed map space.
    slamOccuMap = double(reshape(msg.Data, [c,r])); 
    
    % Convert to convention with (+1) to be free-space and (-1) to be
    % obstacle.
    slamOccuMap(find(slamOccuMap > 0)) = -1; 
    slamOccuMap(find(slamOccuMap == 0)) = 1;
    
    % Grid stuff.
    lowEnv = [origin(1);origin(2)];
    upEnv = [0.6;0.6]; %[realWidth+origin(1);realHeight+origin(2)];
    gridLow = [lowEnv;-pi;-0.1];
    gridUp = [upEnv;pi;0.7];
    N = [31;31;21;11]; 
    pdDims = 3;
    grid = createGrid(gridLow, gridUp, N, pdDims);
    
    % Grab the 2D grid.
    [grid2D, ~] = proj(grid, grid.xs{1}, [0 0 1 1], [0 0]);
        mapBounds = [origin(1), origin(2), origin(1) + realWidth, origin(2) + realHeight];
     trueOccuMap = generate_computation_grid(grid2D, slamOccuMap, res, mapBounds);

    % Plot (negate so we see obstacle part as filled in).
    contourf(grid2D.xs{1}, grid2D.xs{2}, -trueOccuMap, [0,0]);

    %unsensedIndicies = find(slamOccuMap(:) < 0);
    %freeIndicies = find(slamOccuMap(:) == 0);
    %occuIndicies = find(slamOccuMap(:) > 0);
    %[occuR, occuC] = ind2sub(size(slamOccuMap), occuIndicies);
    %[freeR, freeC] = ind2sub(size(slamOccuMap), freeIndicies);
    
    % plot in real-world coordinates.
    %scatter(occuR*res-origin(1), occuC*res-origin(1), [], 'k', 'filled');
    %scatter(freeR*res, freeC*res, [], [0.8,0.8,0.8], 'filled');
    %xlim([0,realWidth]);
    %ylim([0,realHeight]);
end