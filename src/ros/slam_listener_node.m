function slam_listener_node()
    rosinit 
    
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
    realHeight = double(r*res);
    realWidth = double(c*res);
    origin = [double(msg.Info.Origin.Position.X), double(msg.Info.Origin.Position.Y)];

    fprintf('Got occupancy map message with info:\n');
    fprintf('     realH: %f, # rows: %d\n', realHeight, r);
    fprintf('     realW: %f, # cols: %d\n', realWidth, c);
    fprintf('     origin: (%d, %d)\n', origin(1), origin(2));
    % Save the entire SBPD ground-truth occupancy map
    slamOccuMap = double(reshape(msg.Data, [r,c])); 

     imagesc(slamOccuMap);
    xlim([0,c]);
    ylim([0,r]);
end