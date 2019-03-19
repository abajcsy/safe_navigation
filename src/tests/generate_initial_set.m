%% Little script to grab the initial safe set for SLAM 4D model.

% parameters for SLAM
params = car4DLocalQCameraSLAM();

params.envType = 'hand'; % spoof.

% create safety module 
safety = SafetyModule(params.grid, params.dynSys, params.uMode, params.dMode, ...
    params.dt, params.updateEpsilon, params.warmStart, params.envType, ...
    params.updateMethod, params.tMax, params.initialR);

% generate cost function corresponding to initial safe set
center = params.initSenseData{1}(1:2);
radius = params.initSenseData{2}(2); 
sensingShape = -shapeCylinder(params.grid, [3,4], center, radius);

% compute the safe set.
safety.computeAvoidSet(sensingShape, 1);