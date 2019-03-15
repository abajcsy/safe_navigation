function grab_slam_map()
    setenv('ROS_MASTER_URI','http://128.32.38.83:11311');
    rosinit 

    % Subscriber that listens to SLAM occupancy maps.
    occuMapMsgType = 'nav_msgs/OccupancyGrid';
    occuMapTopicName = '/map';
    occuMapSub = rossubscriber(occuMapTopicName, ...
        occuMapMsgType, @slamMapCallback);
    % Spin and let everything go.
    rate = rosrate(100);
    reset(rate);
    while true
        waitfor(rate);
    end
end

%% Grabs the most recent SLAM occupancy map and converts it into 
% a compatible representation for safety computations.
function slamMapCallback(~, msg)
    res = double(msg.Info.Resolution);
    numY = msg.Info.Height;
    numX = msg.Info.Width;
    realHeight = double(numY)*double(res);
    realWidth = double(numX)*double(res);
    origin = [double(msg.Info.Origin.Position.X), double(msg.Info.Origin.Position.Y)];

    % Grab the SLAM-generated occupancy map.
    % Convention for SLAM map values:
    %   > 0     -- sensed obstacle
    %   = 0     -- sensed free-space
    %   < 0     -- unsensed map space.
    slamOccuMap = double(reshape(msg.Data, [numX,numY]));

    % Convert to convention with (+1) to be free-space and (-1) to be
    % obstacle.
    slamOccuMap(find(slamOccuMap > 0)) = -1; 
    slamOccuMap(find(slamOccuMap == 0)) = 1;

    fprintf('Got new SLAM occupancy map message with info:\n');
    fprintf('     realH: %f, # rows: %d\n', realHeight, numY);
    fprintf('     realW: %f, # cols: %d\n', realWidth, numX);
    fprintf('     origin: (%d, %d)\n', origin(1), origin(2));

    % Store the correct representation of the full SLAM map.
    rawOccuMap = slamOccuMap;
    
    % save it out.
    save('rawOccuMap_testSLAM_2.mat', 'rawOccuMap', 'realHeight', ...
        'realWidth', 'origin', 'res', 'origin');
end