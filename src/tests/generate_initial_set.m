%% Little script to grab the initial safe set for SLAM 4D model.

% parameters for SLAM
params = car4DLocalQCameraSLAM();

% create safety module 
safety = SafetyModule(params.grid, params.dynSys, params.uMode, params.dMode, ...
    params.dt, params.updateEpsilon, params.warmStart, params.envType, ...
    params.updateMethod, params.tMax);

% generate cost function corresponding to initial safe set
center = params.initSenseData{1}(1:2);
radius = params.initSenseData{2}(2); %TODO -- is this wrong everywhere in safety module???
sensingShape = -shapeCylinder(params.grid, [3,4], center, radius);

% compute the safe set.
safety.computeAvoidSet(sensingShape, 1);