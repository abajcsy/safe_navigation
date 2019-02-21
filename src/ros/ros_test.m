%% Test sending pose data through ROS

%% Initialize ROS. 
% Creates a ROS master in MATLAB and starts a "global node" that is 
% connected to the master.
rosinit

%exampleHelperROSCreateSampleNetwork

%% Subscriber
% need to subscribe to state 
topicName = '/pose';
msgType = 'geometry_msgs/Pose';
posemsg = rossubscriber(topicName, msgType, @poseCallback);

% Wait to ensure subscriber can receive messages
%pause(2) 
maxTime=1000;
i = 1;
while true
    pause(0.01);
end

% stop the pose subscriber
clear robotpose

% kill ros 
rosshutdown

%% Callback function to grab pose message, do some operations, and print it out.
function pose = poseCallback(~,msg)
    x = msg.Position.X;
    y = msg.Position.Y;
    quat = [msg.Orientation.X msg.Orientation.Y msg.Orientation.Z msg.Orientation.W];
    eul = quat2eul(quat);
    fprintf("(x,y,theta): %f, %f, %f\n",x,y,eul(1));
    pose = msg;
end