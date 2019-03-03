%% Test sending pose data through ROS
function matlab_test_node()
%% Initialize ROS. 
% Creates a ROS master in MATLAB and starts a "global node" that is 
% connected to the master.
rosinit

%% Generate Custom Messages: DO THIS ONCE
%folderpath = '/home/abajcsy/visual_mpc_mount/safe_navigation_ws/src/safe_navigation/';

% TODO:
% - then generate the custom messages for matlab: rosgenmsg('path')
% - Add location of files to javaclasspath.txt: 
%   Add the specified paths as new lines of text in the javaclasspath.txt file. 
%   If this file does not exist, a message in the command window prompts you to create it. 
%   This text file defines the static class path for Java classes. 
% - Add location of class files to MATLAB path: 
%   Use addpath to add new locations of files with the .m extension to 
%   the MATLAB path and use savepath to save these changes.
% - Restart MATLAB and verify messages are available: After restarting 
%   MATLAB, call rosmsg list or rosmessage to check that you 
%   can use the messages as expected.
%examplePackages = fullfile(fileparts(which('rosgenmsg')), 'examples', 'packages');
%userFolder = '/home/abajcsy/visual_mpc_mount/test_msgs';
%copyfile(examplePackages, userFolder)
%rosgenmsg(folderpath)

%% Publisher that initiates replan requests.
pubTopicName = '/replan_info';
pubMsgType = 'safe_navigation_msgs/ReplanInfo';
pub = rospublisher(pubTopicName,pubMsgType);

%% Subscriber that listens to trajectory messages.
subTopicName = '/traj';
subMsgType = 'safe_navigation_msgs/Trajectory';
rossubscriber(subTopicName, subMsgType, @trajCallback);

% Test: construction and publishing.
x = [0,0,0];
u = [0,0];
map = ones(31,31); % everything is free!
map(10,10) = -1;
g = [4,4,0];

replanMsg = toReplanInfo(x,u,map,g);
while true
	send(pub, replanMsg);
    pause(0.01);
end

% stop the pose subscriber
%clear robotpose

% kill ros 
rosshutdown
end

% Callback function to grab pose message, do some operations, and print it out.
function trajCallback(~,msg)
    states = msg.States;
    controls = msg.Controls;
    fprintf('states: \n');
    for i=1:numel(states)
        s = states(i);
        fprintf('    (%f, %f, %f)\n', s.X(1), s.X(2), s.X(3));
    end
    fprintf('controls: \n');
    for i=1:numel(controls)
        c = controls(i);
        fprintf('    (%f, %f)\n', c.U(1), c.U(2));
    end
end

% Creates a ReplanRequest ROS message
function msg = toReplanInfo(x,u,map,g)
    % NOTE: we need to capitalize all message fields for MATLAB
    %   ex. if the ROS message has a field called
    %           big_boi
    %       then in MATLAB it would be called
    %           BigBoi
    
	msg = rosmessage('safe_navigation_msgs/ReplanInfo');
	% Construct state msg.
	state = rosmessage('safe_navigation_msgs/State');
	state.X = x;
	% Construct goal msg.
	goal = rosmessage('safe_navigation_msgs/State');
	goal.X = g;
	% Construct control msg.
	control = rosmessage('safe_navigation_msgs/Control');
	control.U = u;
    % Construct nav_msgs/OccupancyGrid msg.
    grid = rosmessage('nav_msgs/OccupancyGrid');
    grid.Data = map(:); % flatten to 1D array
    [r,c] = size(map);
    grid.Info.Resolution = 1; % TODO: this needs to be actual res.
    grid.Info.Width = c;
    grid.Info.Height = r;
    
	% Populate ReplanInfo msg.
	msg.CurrX = state;
	msg.CurrU = control;
	msg.OccuMap = grid;
	msg.Goal = goal;
end
