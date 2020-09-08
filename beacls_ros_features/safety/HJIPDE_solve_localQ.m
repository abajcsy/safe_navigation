function [data, tau, extraOuts] = ...
    HJIPDE_solve_localQ(data0, lxOld, lx, updateEpsilon, tau, schemeData, compMethod, extraArgs)
% ----- How to use this function -----
%
% Inputs:
%   data0           - initial value function
%   lxOld           - prior cost function from last HJIPDE computation 
%   lx              - current cost function 
%   updateEpsilon   - don't consider states when change in Vx is < epsilon
%   tau             - list of computation times
%   schemeData      - problem parameters passed into the Hamiltonian func
%                       .grid: grid (required!)
%                       .accuracy: accuracy of derivatives
%   compMethod      - Informs which optimization we're doing
%                       - 'minVWithV0' to do min with original data 
%                         (default)
%                       - 'maxVWithV0' to do max with original data
%                       - 'set' or 'none' to compute reachable set 
%                         (not tube)
%                       - 'zero' or 'minWithZero' to min Hamiltonian with 
%                          zero
%                       - 'minVOverTime' to do min with previous data
%                       - 'maxVOverTime' to do max with previous data
%                       - 'minVWithL' to do min with targets
%                       - 'maxVWithL' to do max with targets
%   extraArgs       - this structure can be used to leverage other 
%                       additional functionalities within this function. 
%     .targets:             (matrix) a single target or a list of targets 
%                           with time stamps tau (targets must have same 
%                           time stamp as the solution). This functionality 
%                           is mainly useful when the targets are 
%                           time-varying, in case of variational inequality 
%                           for example; data0 can be used to specify the  
%                           target otherwise. This is also useful when
%                           warm-starting with a value function (data0)
%                           that is not equal to the target/cost function
%                           (l(x))
%     .stopConverge:        (bool) set to true to stop the computation when
%                           it converges
%     .convergeThreshold:   Max change in each iteration allowed when 
%                           checking convergence
%     .discountFactor:      (double) amount by which you'd like to discount
%                           the value function. Used for ensuring
%                           convergence. Remember to move your targets
%                           function (l(x)) and initial value function
%                           (data0) down so they are below 0 everywhere.
%                           you can raise above 0 by the same amount at the
%                           end.
%     .discountAnneal:      (string) if you want to anneal your discount
%                           factor over time so it converges to the right
%                           solution, use this. 
%                               - 'soft' moves it slowly towards 1 each 
%                                 time convergence happens
%                               - 'hard' sets it to 1 once convergence
%                                  happens
%                               - 1 sets it to 'hard'
%     .visualize:           either fill in struct or set to 1 to visualize
%                           .plotData:          (struct) information   
%                                               required to plot the data:
%                                               .plotDims: dims to plot
%                                               .projpt: projection points.
%                                                        Can be vector or
%                                                        cell. e.g.
%                                                        {pi,'min'} means
%                                                        project at pi for
%                                                        first dimension,
%                                                        take minimum
%                                                        (union) for second
%                                                        dimension
%                           .plotColorVS0:      color of initial value set
%                           .plotColorVF0:      color of initial value
%                                               function
%                           .plotAlphaVF0:      transparency of initial
%                                               value function
%                           .plotColorVS:       color of value set
%                           .plotColorVF:       color of value function
%                           .plotAlphaVF:       transparency of initial
%                                               value function
%                           .plotColorOS:       color of obstacle set
%                           .plotColorOF:       color of obstacle function
%                           .plotAlphaOF:       transparency of obstacle
%                                               function
%                           .plotColorTS:       color of target set
%                           .plotColorTF:       color of target function
%                           .plotAlphaTF:       transparency of target
%                                               function
%                           .plotColorlxOld:    color of old cost function
%       .inheritVals        which function to inherit values from (lx or
%                           Vold)
% Outputs:
%   data - solution corresponding to grid g and time vector tau
%   tau  - list of computation times (redundant)
%   extraOuts - This structure can be used to pass on extra outputs, for
%               example:
%       .QSizes: (array) size of Q at each step backwards in computation time 

%% Default parameters
if numel(tau) < 2
    error('Time vector must have at least two elements!')
end

if nargin < 4
    compMethod = 'minVWdithV0';
end

if nargin < 5
    extraArgs = [];
end

extraOuts = [];
quiet = false;
lowMemory = false;
keepLast = false;
flipOutput = false;

small = 1e-4;
g = schemeData.grid;
gDim = g.dim;
clns = repmat({':'}, 1, gDim);

%% Extract the information from extraargs
% Quiet mode
if isfield(extraArgs, 'quiet') && extraArgs.quiet
    fprintf('HJIPDE_solve_localQ running in quiet mode...\n')
    quiet = true;
end

% Low memory mode
if isfield(extraArgs, 'lowMemory') && extraArgs.lowMemory
    fprintf('HJIPDE_solve running in low memory mode...\n')
    lowMemory = true;
    
    % Save the output in reverse order
    if isfield(extraArgs, 'flipOutput') && extraArgs.flipOutput
        flipOutput = true;
    end
    
end

% Only keep latest data (saves memory)
if isfield(extraArgs, 'keepLast') && extraArgs.keepLast
    keepLast = true;
end

%---Extract the information about obstacles--------------------------------
obsMode = 'none';

% if you accidentally write obstacle instead of obstacles, we got you
% covered
if isfield(extraArgs, 'obstacle')
    extraArgs.obstacles = extraArgs.obstacle;
    warning('you wrote extraArgs.obstacle instead of extraArgs.obstacles')
end

if isfield(extraArgs, 'obstacles')
    obstacles = extraArgs.obstacles;
    
    % are obstacles moving or not?
    if numDims(obstacles) == gDim
        obsMode = 'static';
        obstacle_i = obstacles;
    elseif numDims(obstacles) == gDim + 1
        obsMode = 'time-varying';
        obstacle_i = obstacles(clns{:}, 1);
    else
        error('Inconsistent obstacle dimensions!')
    end
    
    % We always take the max between the data and the obstacles
    % note that obstacles are negated.  That's because if you define the
    % obstacles using something like ShapeSphere, it sets it up as a
    % target. To make it an obstacle we just negate that.
    data0 = max(data0, -obstacle_i);
end

%---Extract the information about targets----------------------------------
% TO DO: allow targets to be time-varying

% if you accidentally write target instead of targets, we got you
% covered
if isfield(extraArgs, 'target')
    warning('you wrote extraArgs.target instead of extraArgs.targets')
    extraArgs.targets = extraArgs.target;
end

if isfield(extraArgs, 'targets')
    targets = extraArgs.targets;
end

%---Stopping Conditions----------------------------------------------------

% Check validity of stopInit if needed
if isfield(extraArgs, 'stopInit')
    if ~isvector(extraArgs.stopInit) || gDim ~= length(extraArgs.stopInit)
        error('stopInit must be a vector of length g.dim!')
    end
end

% Check validity of stopSet if needed
if isfield(extraArgs, 'stopSet') % For backwards compatibility
    extraArgs.stopSetInclude = extraArgs.stopSet;
end

if isfield(extraArgs,'stopSetInclude') || isfield(extraArgs,'stopSetIntersect')
    if isfield(extraArgs,'stopSetInclude')
        stopSet = extraArgs.stopSetInclude;
    else
        stopSet = extraArgs.stopSetIntersect;
    end
    
    if numDims(stopSet) ~= gDim || any(size(stopSet) ~= g.N')
        error('Inconsistent stopSet dimensions!')
    end
    
    % Extract set of indices at which stopSet is negative
    setInds = find(stopSet(:) < 0);
    
    % Check validity of stopLevel if needed
    if isfield(extraArgs, 'stopLevel')
        stopLevel = extraArgs.stopLevel;
    else
        stopLevel = 0;
    end
end

%% Visualization
if (isfield(extraArgs, 'visualize') && isstruct(extraArgs.visualize))...
   || (isfield(extraArgs, 'visualize') && ~isstruct(extraArgs.visualize)...
        && extraArgs.visualize)...
   || (isfield(extraArgs, 'makeVideo') && extraArgs.makeVideo)
    % Mark initial iteration, state that for the first plot we need
    % lighting
    timeCount = 0;
    needLight = true;
    
    %---Projection Parameters----------------------------------------------

    % Extract the information about plotData
    plotDims = ones(gDim, 1);
    projpt = [];
    if isfield(extraArgs.visualize, 'plotData')
        % Dimensions to visualize
        % It will be an array of 1s and 0s with 1s means that dimension should
        % be plotted.
        plotDims = extraArgs.visualize.plotData.plotDims;
        
        % Points to project other dimensions at. There should be an entry point
        % corresponding to each 0 in plotDims.
        projpt = extraArgs.visualize.plotData.projpt;
    end
    
    % Number of dimensions to be plotted and to be projected
    pDims = nnz(plotDims);
    projDims = length(projpt);
    
    % Basic Checks
    if (pDims > 4)
        error('Currently plotting up to 3D is supported!');
    end
    
    %---Defaults-----------------------------------------------------------
    
    % If person just specifies that they want to visualize, set default
    % things to visualize
    if isfield(extraArgs, 'visualize') && ...
            ~isstruct(extraArgs.visualize) && extraArgs.visualize==1
        extraArgs.visualize.initialValueSet = 1;
        extraArgs.visualize.valueSet = 1;
    end
    
    if isfield(extraArgs, 'obstacles')
        if ~isfield(extraArgs.visualize, 'obstacleSet')
            extraArgs.visualize.obstacleSet = 1;
        end
    end
    
    if isfield(extraArgs, 'targets')
        if ~isfield(extraArgs.visualize, 'targetSet')
            extraArgs.visualize.targetSet = 1;
        end
    end
    
    % Set level set slice
    if isfield(extraArgs.visualize, 'sliceLevel')
        sliceLevel = extraArgs.visualize.sliceLevel;
    else
        sliceLevel = 0;
    end
    
    % Do we want to see every single plot at every single time step, or
    % only the most recent one?
    if isfield(extraArgs.visualize, 'deleteLastPlot')
        deleteLastPlot = extraArgs.visualize.deleteLastPlot;
    else
        deleteLastPlot = false;
    end
   
    
    view3D = 0;
    
    %---Perform Projections------------------------------------------------
    
    % Project
    if projDims == 0
        gPlot = g;
        dataPlot = data0;
        
        if isfield(extraArgs, 'obstacles')
            % if strcmp(obsMode, 'time-varying')
            obsPlot = obstacle_i;
        end
        if isfield(extraArgs, 'targets')
            % if strcmp(obsMode, 'time-varying')
            targetsPlot = targets;
        end
    else
        % if projpt is a cell, project each dimensions separately. This
        % allows us to take the union/intersection through some dimensions
        % and to project at a particular slice through other dimensions.
        if iscell(projpt)
            idx = find(plotDims==0);
            plotDimsTemp = ones(size(plotDims));
            gPlot = g;
            dataPlot = data0;
            if isfield(extraArgs, 'obstacles')
                obsPlot = obstacle_i;
            end
            if isfield(extraArgs, 'targets')
                targetsPlot = targets;
            end
            
            for ii = length(idx):-1:1
                plotDimsTemp(idx(ii)) = 0;
                if isfield(extraArgs, 'obstacles')
                    %if strcmp(obsMode, 'time-varying')
                    [~, obsPlot] = proj(gPlot, obsPlot, ~plotDimsTemp,...
                        projpt{ii});
                end
                if isfield(extraArgs, 'targets')
                    %if strcmp(obsMode, 'time-varying')
                    [~, targetsPlot] = proj(gPlot, targetsPlot, ...
                        ~plotDimsTemp, projpt{ii});
                end
                
                [gPlot, dataPlot] = proj(gPlot, dataPlot, ~plotDimsTemp,...
                    projpt{ii});
                plotDimsTemp = ones(1,gPlot.dim);
            end
            
        else
            [gPlot, dataPlot] = proj(g, data0, ~plotDims, projpt);
            
            if isfield(extraArgs, 'obstacles')
                %if strcmp(obsMode, 'time-varying')
                [~, obsPlot] = proj(g, obstacle_i, ~plotDims, projpt);
            end
            if isfield(extraArgs, 'targets')
                %if strcmp(obsMode, 'time-varying')
                [~, targetsPlot] = proj(g, targets, ~plotDims, projpt);
            end
        end
        
        
        
    end
    
    
    %---Video Parameters---------------------------------------------------
    
    % If we're making a video, set up the parameters
    if isfield(extraArgs, 'makeVideo') && extraArgs.makeVideo
        if ~isfield(extraArgs, 'videoFilename')
            extraArgs.videoFilename = ...
                [datestr(now,'YYYYMMDD_hhmmss') '.mp4'];
        end
        
        vout = VideoWriter(extraArgs.videoFilename,'MPEG-4');
        vout.Quality = 100;
        if isfield(extraArgs, 'frameRate')
            vout.FrameRate = extraArgs.frameRate;
        else
            vout.FrameRate = 30;
        end
        
        try
            vout.open;
        catch
            error('cannot open file for writing')
        end
    end
   
    
    %---Initialize Figure--------------------------------------------------
    
    
    % Initialize the figure for visualization
    if isfield(extraArgs.visualize,'figNum')
        f = figure(extraArgs.visualize.figNum);
    else
        f = figure;
    end
    
    % Clear figure unless otherwise specified
    if ~isfield(extraArgs.visualize,'holdOn')|| ~extraArgs.visualize.holdOn
        clf
    end
    hold on
    grid on
    
    % Set defaults
    eAT_visSetIm.sliceDim = gPlot.dim;
    eAT_visSetIm.applyLight = false;
    if isfield(extraArgs.visualize, 'lineWidth')
        eAT_visSetIm.LineWidth = extraArgs.visualize.lineWidth;
    else
        eAT_visSetIm.LineWidth = 2;
    end
    
    % If we're stopping once we hit an initial condition requirement, plot
    % said requirement
    if isfield(extraArgs, 'stopInit')
        projectedInit = extraArgs.stopInit(logical(plotDims));
        if nnz(plotDims) == 2
            plot(projectedInit(1), projectedInit(2), 'b*')
        elseif nnz(plotDims) == 3
            plot3(projectedInit(1), projectedInit(2), projectedInit(3), 'b*')
        end
    end
    
    %% Visualize Inital Value Function/Set
    
    %---Visualize Initial Value Set----------------------------------------
    if isfield(extraArgs.visualize, 'initialValueSet') &&...
            extraArgs.visualize.initialValueSet
        
        if ~isfield(extraArgs.visualize,'plotColorVS0')
            extraArgs.visualize.plotColorVS0 = 'b';
        end
        
        extraOuts.hVS0 = visSetIm(...
            gPlot, dataPlot, extraArgs.visualize.plotColorVS0,...
            sliceLevel, eAT_visSetIm);
    end
    
    %---Visualize Initial Value Function-----------------------------------
    if isfield(extraArgs.visualize, 'initialValueFunction') &&...
            extraArgs.visualize.initialValueFunction
        
        % If we're making a 3D plot, mark so we know to view this at an
        % angle appropriate for 3D
        if gPlot.dim >= 2
            view3D = 1;
        end
        
        % Set up default parameters
        if ~isfield(extraArgs.visualize,'plotColorVF0')
            extraArgs.visualize.plotColorVF0 = 'b';
        end
        
        if ~isfield(extraArgs.visualize,'plotAlphaVF0')
            extraArgs.visualize.plotAlphaVF0 = .5;
        end
        
        % Visualize Initial Value function (hVF0)
        [extraOuts.hVF0]= visFuncIm(gPlot,dataPlot,...
            extraArgs.visualize.plotColorVF0,...
            extraArgs.visualize.plotAlphaVF0);
    end
    
    %% -------------- Visualize old cost funtion. -----------------
    
    if isfield(extraArgs.visualize,'plotColorlxOld')
        [gPlot, lxOldPlot] = proj(g, lxOld, ~plotDims, projpt);
        extraOuts.hlxOld = visSetIm(gPlot, lxOldPlot, ...
            extraArgs.visualize.plotColorlxOld, sliceLevel, eAT_visSetIm);
    end
    
    % -------------------------------------------------------------
    
    %% Visualize Target Function/Set
    
    %---Visualize Target Set-----------------------------------------------
    if isfield(extraArgs.visualize, 'targetSet') ...
            && extraArgs.visualize.targetSet
        
        
        if ~isfield(extraArgs.visualize,'plotColorTS')
            extraArgs.visualize.plotColorTS = 'g';
        end
        
        extraOuts.hTS = visSetIm(gPlot, targetsPlot, ...
            extraArgs.visualize.plotColorTS, sliceLevel, eAT_visSetIm);
    end
    
    %---Visualize Target Function------------------------------------------
    if isfield(extraArgs.visualize, 'targetFunction') &&...
            extraArgs.visualize.targetFunction
        % If we're making a 3D plot, mark so we know to view this at an
        % angle appropriate for 3D
        if gPlot.dim >= 2
            view3D = 1;
        end
        
        % Set up default parameters
        if ~isfield(extraArgs.visualize,'plotColorTF')
            extraArgs.visualize.plotColorTF = 'g';
        end
        
        if ~isfield(extraArgs.visualize,'plotAlphaTF')
            extraArgs.visualize.plotAlphaTF = .5;
        end
        
        % Visualize Target function (hTF)
        [extraOuts.hTF]= visFuncIm(gPlot,targetsPlot,...
            extraArgs.visualize.plotColorTF,...
            extraArgs.visualize.plotAlphaTF);
    end
    
    %% Visualize Obstacle Function/Set
    
    %---Visualize Obstacle Set---------------------------------------------
    if isfield(extraArgs.visualize, 'obstacleSet') ...
            && extraArgs.visualize.obstacleSet
        
        if ~isfield(extraArgs.visualize,'plotColorOS')
            extraArgs.visualize.plotColorOS = 'r';
        end
        
        % Visualize obstacle set (hOS)
        extraOuts.hOS = visSetIm(gPlot, obsPlot, ...
            extraArgs.visualize.plotColorOS, 0);%, eAO_visSetIm);
    end
    
    %---Visualize Obstacle Function----------------------------------------
    if  isfield(extraArgs.visualize, 'obstacleFunction') ...
            && extraArgs.visualize.obstacleFunction
        % If we're making a 3D plot, mark so we know to view this at an
        % angle appropriate for 3D
        if gPlot.dim >= 2
            view3D = 1;
        end
        
        % Set up default parameters
        if ~isfield(extraArgs.visualize,'plotColorOF')
            extraArgs.visualize.plotColorOF = 'r';
        end
        
        if ~isfield(extraArgs.visualize,'plotAlphaOF')
            extraArgs.visualize.plotAlphaOF = .5;
        end
        
        % Visualize function
        [extraOuts.hOF]= visFuncIm(gPlot,-obsPlot,...
            extraArgs.visualize.plotColorOF,...
            extraArgs.visualize.plotAlphaOF);
    end
    %% Visualize Value Function/Set
    
    %---Visualize Value Set------------------------------------------------
    if isfield(extraArgs.visualize, 'valueSet') &&...
            extraArgs.visualize.valueSet
        
        if ~isfield(extraArgs.visualize,'plotColorVS')
            extraArgs.visualize.plotColorVS = 'b';
        end
        
        extraOuts.hVS = visSetIm(gPlot, dataPlot, ...
            extraArgs.visualize.plotColorVS, sliceLevel, eAT_visSetIm);
    end
    
    %---Visualize Value Function-------------------------------------------
    if isfield(extraArgs.visualize, 'valueFunction') && ...
            extraArgs.visualize.valueFunction
        % If we're making a 3D plot, mark so we know to view this at an
        % angle appropriate for 3D
        if gPlot.dim >= 2
            view3D = 1;
        end
        
        % Set up default parameters
        if ~isfield(extraArgs.visualize,'plotColorVF')
            extraArgs.visualize.plotColorVF = 'b';
        end
        
        if ~isfield(extraArgs.visualize,'plotAlphaVF')
            extraArgs.visualize.plotAlphaVF = .5;
        end
        
        % Visualize Value function (hVF)
        [extraOuts.hVF]= visFuncIm(gPlot,dataPlot,...
            extraArgs.visualize.plotColorVF,...
            extraArgs.visualize.plotAlphaVF);
        
    end
    
    %% General Visualization Stuff
    
    %---Set Angle, Lighting, axis, Labels, Title---------------------------
    
    % Set Angle
    if pDims >2 || view3D || isfield(extraArgs.visualize, 'viewAngle')
        if isfield(extraArgs.visualize, 'viewAngle')
            view(extraArgs.visualize.viewAngle)
        else
            view(30,10)
        end
        
        % Set Lighting
        if needLight% && (gPlot.dim == 3)
            lighting phong
            c = camlight;
            %need_light = false;
        end
        if isfield(extraArgs.visualize, 'camlightPosition')
            c.Position = extraArgs.visualize.camlightPosition;
        else
            c.Position = [-30 -30 -30];
        end
    end
 
    % Grid and axis
    if isfield(extraArgs.visualize, 'viewGrid') && ...
            ~extraArgs.visualize.viewGrid
        grid off
    end
    
    if isfield(extraArgs.visualize, 'viewAxis')
        axis(extraArgs.visualize.viewAxis)
    end
    axis square
    
    % Labels
    if isfield(extraArgs.visualize, 'xTitle')
        xlabel(extraArgs.visualize.xTitle, 'interpreter','latex')
    end
    
    if isfield(extraArgs.visualize, 'yTitle')
        ylabel(extraArgs.visualize.yTitle,'interpreter','latex')
    end 
    
    if isfield(extraArgs.visualize, 'zTitle')
        zlabel(extraArgs.visualize.zTitle,'interpreter','latex')
    end
    
    title(['t = ' num2str(0) ' s'])
    set(gcf,'Color','white')
    
    if isfield(extraArgs.visualize, 'fontSize')
        set(gca,'FontSize',extraArgs.visualize.fontSize)
    end
    
    if isfield(extraArgs.visualize, 'lineWidth')
        set(gca,'LineWidth',extraArgs.visualize.lineWidth)
    end
    
    drawnow;
    
    % If we're making a video, grab the frame
    if isfield(extraArgs, 'makeVideo') && extraArgs.makeVideo
        current_frame = getframe(gcf); %gca does just the plot
        writeVideo(vout,current_frame);
    end
    
    % If we're saving each figure, do so now
    if isfield(extraArgs.visualize, 'figFilename')
        export_fig(sprintf('%s%d', extraArgs.visualize.figFilename, 0), '-png')
    end
end


%% Extract cdynamical system if needed
% if isfield(schemeData, 'dynSys')
%     schemeData.hamFunc = @genericHam;
%     schemeData.partialFunc = @genericPartial;
% end

if ~isfield(schemeData, 'hamFunc')
    schemeData.hamFunc = @genericHam;
end
if ~isfield(schemeData, 'partialFunc')
    schemeData.partialFunc = @genericPartial;
end

stopConverge = false;
if isfield(extraArgs, 'stopConverge')
    stopConverge = extraArgs.stopConverge;
    if isfield(extraArgs, 'convergeThreshold')
        convergeThreshold = extraArgs.convergeThreshold;
    else
        convergeThreshold = 1e-5;
    end
end

%% SchemeFunc and SchemeData
schemeFunc = @termLaxFriedrichs_localQ;
% Extract accuracy parameter o/w set default accuracy
accuracy = 'veryHigh';
if isfield(schemeData, 'accuracy')
    accuracy = schemeData.accuracy;
end

%% Numerical approximation functions
dissType = 'global';
[schemeData.dissFunc, integratorFunc, schemeData.derivFunc] = ...
    getNumericalFuncs(dissType, accuracy);

if strcmp(compMethod, 'minWithZero') || strcmp(compMethod, 'zero')
    schemeFunc = @termRestrictUpdate;
    schemeData.innerFunc = @termLaxFriedrichs;
    schemeData.innerData = schemeData;
    schemeData.innerData = schemeData;
    schemeData.positive = 0;
end

%% Time integration
integratorOptions = odeCFLset('factorCFL', 0.8, 'singleStep', 'on');

startTime = cputime;

%% ------- Extract states where Q = {x | lxOld < lx}. --------- %
% Store the (linear) indicies of all the states where the new cost
% function has changed (i.e. we sensed free-space). 
Q = find((lxOld <= 0).*(lx > 0));
% Set states in warm-started value function that are free-space
% to have the l(x) value -- need this for V(x) to change at all (in
% theory!)
data0(Q) = lx(Q);

% Q1 = find(lxOld <= 0);
% data0(Q1) = lx(Q1);

% Vector that helps us convert from 3D to linear indicies.
szDims = size(data0);
if schemeData.dynSys.nx == 3
    % For 3D system: [1, nX, nX*nY]
    stride = [1, szDims(1), szDims(1)*szDims(2)];
elseif schemeData.dynSys.nx == 4
    % For 4D system: [1, nX, nX*nY, nX*nY*nTheta]
    stride = [1, szDims(1), szDims(1)*szDims(2), szDims(1)*szDims(2)*szDims(3)];
else
    error('Cannot do localQ update for %dD system!', length(szDims));
end

% Ensures we won't look outside of grid bounds.
maxLinIdx = prod(szDims);
% Number of neighbors to look at (related to order of gradient approx)
numNeighs = 3;

% Get neighbors for all states in Q in each dimension 
%   3D: (x,y,theta)
%   4D: (x,y,theta,vel)
neighbors = [];
% Periodic dimenion is the 3rd dimension (theta).
pDim = 3;
for dim=1:schemeData.dynSys.nx
	neighList = getNeigh(Q, dim, numNeighs, stride, maxLinIdx, schemeData.grid, pDim);
    neighbors = [neighbors; neighList];
end
Q = vertcat(Q, neighbors(:));
Q = unique(Q);

% Store initial value function for computing how much a one-step 
% update affected the values.
Vold = data0(:); 

% Stores the sequence of Q sizes over computation time.
extraOuts.QSizes = [];
Qold = [];
% ------------------------------------------------------------ %

%% Initialize PDE solution
data0size = size(data0);

if numDims(data0) == gDim
    % New computation
    if keepLast
        data = data0;
    elseif lowMemory
        data = single(data0);
    else
        data = zeros([data0size(1:gDim) length(tau)]);
        data(clns{:}, 1) = data0;
    end
    
    istart = 2;
elseif numDims(data0) == gDim + 1
    % Continue an old computation
    if keepLast
        data = data0(clns{:}, data0size(end));
    elseif lowMemory
        data = single(data0(clns{:}, data0size(end)));
    else
        data = zeros([data0size(1:gDim) length(tau)]);
        data(clns{:}, 1:data0size(end)) = data0;
    end
    
    % Start at custom starting index if specified
    if isfield(extraArgs, 'istart')
        istart = extraArgs.istart;
    else
        istart = data0size(end)+1;
    end
else
    error('Inconsistent initial condition dimension!')
end


if isfield(extraArgs,'ignoreBoundary') &&...
        extraArgs.ignoreBoundary
    [~, dataTrimmed] = truncateGrid(...
        g, data0, g.min+4*g.dx, g.max-4*g.dx);
end


%% Beginning of backwards reachable set/tube computation. 
for i = istart:length(tau)
    if ~quiet
        fprintf('tau(i) = %f\n', tau(i))
    end
    %% Variable schemeData
    if isfield(extraArgs, 'SDModFunc')
        if isfield(extraArgs, 'SDModParams')
            paramsIn = extraArgs.SDModParams;
        else
            paramsIn = [];
        end
        
        schemeData = extraArgs.SDModFunc(schemeData, i, tau, data, obstacles, ...
            paramsIn);
    end
    
    if keepLast
        y0 = data;
    elseif lowMemory
        if flipOutput
            y0 = data(clns{:}, 1);
        else
            y0 = data(clns{:}, size(data, g.dim+1));
        end
        
    else
        y0 = data(clns{:}, i-1);
    end
    y = y0(:);

    tNow = tau(i-1);
    
    %% Main integration loop to get to the next tau(i)  
    % Stop updates if either converged AND have no more states to update. 
    while tNow < tau(i) - small && ~isempty(Q) && (~isempty(setdiff(Q, Qold)) || ~isempty(setdiff(Qold, Q)))
        %fprintf('\n');
        [sz,~] = size(Q);
        %fprintf('Q size: %f\n', sz);
        %fprintf('\n');
        
        % Record the current Q size.
        extraOuts.QSizes = [extraOuts.QSizes, sz];
                
        % Save previous data if needed
        if strcmp(compMethod, 'minVOverTime') || ...
                strcmp(compMethod, 'maxVOverTime')
            yLast = y;
        end
        
        if ~quiet
            fprintf('  Computing [%f %f]...\n', tNow, tau(i))
        end
        
        % Solve hamiltonian and apply to value function (y) to get updated
        % value function
        [tNow, y] = feval(integratorFunc, schemeFunc, [tNow tau(i)], y, ...
            integratorOptions, schemeData, Q);
        
        if any(isnan(y))
            keyboard
        end
        
        %Tube Computations
        %   compMethod - set to 'set' or 'none' to compute reachable set (not tube)
        %              - set to 'zero' to do min Hamiltonian with zero
        %              - set to 'minVOverTime' to do min with previous data
        %              - set to 'minVWithV0' to do min with original data
        %              - set to 'maxVOverTime' to do max with previous data
        %              - set to 'maxVWithV0' to do max with original data
        %              - set to 'minVWithL' to do min with targets
        %              - set to 'maxVWithL' to do max with targets
        if strcmp(compMethod, 'minVOverTime') %Min over Time
            y = min(y, yLast);
        elseif strcmp(compMethod, 'maxVOverTime')
            y = max(y, yLast);
        elseif strcmp(compMethod, 'minVWithV0')%Min with data0
            y = min(y,data0(:));
        elseif strcmp(compMethod, 'maxVWithV0')
            y = max(y,data0(:));
        elseif strcmp(compMethod, 'minVWithL') ...
                || isfield(extraArgs, 'targets')
            if numDims(targets) == gDim
                y = min(y, targets(:));
            else
                target_i = targets(clns{:}, i);
                y = min(y, target_i(:));
            end
        elseif strcmp(compMethod, 'maxVWithL')
            if ~isfield(extraArgs, 'targets')
                error('Need to define target function l(x)!')
            end
            if numDims(targets) == gDim
                y = max(y, targets(:));
            else
                target_i = targets(clns{:}, i);
                y = max(y, target_i(:));
            end
        elseif strcmp(compMethod, 'zero') || strcmp(compMethod, 'set')
        else
            error('Check which compMethod you are using')
        end
        
        % ------- Only keep Vx changes for the states in Q. --------- %
        % Store the change between the old value and the new value at the 
        % candidate states stored in Q.
        VxError = Vold(Q) - y(Q);
        
        % Change the old values at the states in Q. 
        Vold(Q) = y(Q);
        
        % Set new value function to have only the changes for states in Q.
        y = Vold;
        
        % --- Remove states from Q where Vx has not changed enough. --- %
        % Grab the states whose value wasn't affected enough by the update.
        unchangedIndicies = find(abs(VxError) < updateEpsilon);
    
        % Remove the unchanged states from the list of states to update.
        Qold = Q;
        Q(unchangedIndicies) = [];
        % ----------------------------------------------------------- %
        
        % --- Add neighbors of the states that still remain in Q. --- %
        %   NOTE:  Since we are using a 3rd order approx scheme for our
        %   spatial derivative, we should add three neighboring points in each
        %   direction for each state dimension. 6*num_states is the total
        %   number of neighbors we add for each state in Q. 
        %.  e.g. add 3+ and 3- from theta state, and same for x and y.
                
        % Get neighbors for all states in Q in each dimension (x,y,theta)
        neighbors = [];
        for dim=1:schemeData.dynSys.nx
            neighList = getNeigh(Q, dim, numNeighs, stride, maxLinIdx, schemeData.grid, pDim);
            neighbors = [neighbors; neighList];
        end
        Q = vertcat(Q, neighbors(:));
        Q = unique(Q);
        % ---------------------------------------------------------- %
        fprintf('  Q size: %d  Max Change: %.3f\n', length(Q), max(VxError));
        
        % "Mask" using obstacles
        if isfield(extraArgs, 'obstacles')
            if strcmp(obsMode, 'time-varying')
                obstacle_i = obstacles(clns{:}, i);
            end
            y = max(y, -obstacle_i(:));
        end
        
        if isfield(extraArgs, 'discountFactor') && ...
                extraArgs.discountFactor
            if isfield(extraArgs, 'discountMode') && ...
                    strcmp(extraArgs.discountMode,'Jaime') ||...
                    strcmp(extraArgs.discountMode,'jaime')
                %%%% Jaime's method
                % If we're discounting, do that now
                
                y = extraArgs.discountFactor*y;
                if isfield(extraArgs, 'targets')
                    y = y + ...
                        (1-extraArgs.discountFactor).*extraArgs.targets(:);
                else
                    y = y + ...
                        (1-extraArgs.discountFactor).*data0(:);
                end
                
                
            else
                %%%%% Kene's Code %%%%%%
                % make sure to set min mode to 'none'
                % If we're discounting, do that now
                
                
                % move everything below 0
                maxVal = max(abs(extraArgs.targets(:)));
                ytemp = y - maxVal;
                targettemp = extraArgs.targets - maxVal;
                
                % Discount
                ytemp = extraArgs.discountFactor*ytemp;
                
                % Take min
                ytemp = min(ytemp, targettemp(:));
                
                % restore height
                y = ytemp + maxVal;
                
            end
            extraOuts.discountFactor = extraArgs.discountFactor;
        end
    end
    
    % Reshape value function
    data_i = reshape(y, g.shape);
    if keepLast
        data = data_i;
    elseif lowMemory
        if flipOutput
            data = cat(g.dim+1, reshape(y, g.shape), data);
        else
            data = cat(g.dim+1, data, reshape(y, g.shape));
        end
        
    else
        data(clns{:}, i) = data_i;
    end
    
    
        % If we're stopping once converged, print how much change there was in
        % the last iteration
        if stopConverge
            if isfield(extraArgs,'ignoreBoundary') &&...
                    extraArgs.ignoreBoundary
                [gTrunc, dataNew] = truncateGrid(...
                    g, data_i, g.min+4*g.dx, g.max-4*g.dx);
                
                [change, indicies] = max(abs(dataNew(:)-dataTrimmed(:)));
                dataTrimmed = dataNew;
                if ~quiet
                    fprintf('Max change since last iteration: %f\n', change)
                end
            else
                [change, indicies] = max(abs(y - y0(:)));
                if ~quiet
                    fprintf('Max change since last iteration: %f\n', change)
                end
            end
        end
    
    % ---- If Q is empty, we need to exit the outer for-loop as well! --- %
    if isempty(Q) || ~(~isempty(setdiff(Q, Qold)) || ~isempty(setdiff(Qold, Q)))
        extraOuts.stoptau = tau(i);
        tau(i+1:end) = [];

        if ~lowMemory && ~keepLast
            data(clns{:}, i+1:size(data, gDim+1)) = [];
        end
        break
    end
    % ------------------------------------------------------------------- %
        
        
    %% If commanded, stop the reachable set computation once it contains
    % the initial state.
    if isfield(extraArgs, 'stopInit')
        initValue = eval_u(g, data_i, extraArgs.stopInit);
        if ~isnan(initValue) && initValue <= 0
            extraOuts.stoptau = tau(i);
            tau(i+1:end) = [];
            
            if ~lowMemory && ~keepLast
                data(clns{:}, i+1:size(data, gDim+1)) = [];
            end
            break
        end
    end
    
    %% Stop computation if reachable set contains a "stopSet"
    if exist('stopSet', 'var')
        dataInds = find(data_i(:) <= stopLevel);
        
        if isfield(extraArgs, 'stopSetInclude')
            stopSetFun = @all;
        else
            stopSetFun = @any;
        end
        
        if stopSetFun(ismember(setInds, dataInds))
            extraOuts.stoptau = tau(i);
            tau(i+1:end) = [];
            
            if ~lowMemory && ~keepLast
                data(clns{:}, i+1:size(data, gDim+1)) = [];
            end
            break
        end
    end
    
    %% Stop computation if we've converged
    if stopConverge && change < convergeThreshold
        
        if isfield(extraArgs, 'discountFactor') && ...
                extraArgs.discountFactor && ...
                isfield(extraArgs, 'discountAnneal') && ...
                extraArgs.discountFactor ~= 1
            
            if strcmp(extraArgs.discountAnneal, 'soft')
                extraArgs.discountFactor = 1-((1-extraArgs.discountFactor)/2);
                
                if abs(1-extraArgs.discountFactor) < .00005
                    extraArgs.discountFactor = 1;
                end
            elseif strcmp(extraArgs.discountAnneal, 'hard') ...
                    || extraArgs.discountAnneal==1
                extraArgs.discountFactor = 1;
            end
            
            fprintf('\nDiscount factor: %f\n\n', ...
                extraArgs.discountFactor)
        else
            extraOuts.stoptau = tau(i);
            tau(i+1:end) = [];
            
            if ~lowMemory && ~keepLast
                data(clns{:}, i+1:size(data, gDim+1)) = [];
            end
            break
        end
    end
    
    
    %% If commanded, visualize the level set.
    
    if (isfield(extraArgs, 'visualize') && ...
            (isstruct(extraArgs.visualize) || extraArgs.visualize == 1))...
            || (isfield(extraArgs, 'makeVideo') && extraArgs.makeVideo)
        timeCount = timeCount + 1;
        
        %---Delete Previous Plot-------------------------------------------

        if deleteLastPlot
            if isfield(extraOuts, 'hOS') && strcmp(obsMode, 'time-varying')
                if iscell(extraOuts.hOS)
                    for hi = 1:length(extraOuts.hOS)
                        delete(extraOuts.hOS{hi})
                    end
                else
                    delete(extraOuts.hOS);
                end
            end
            
            if isfield(extraOuts, 'hOF') && strcmp(obsMode, 'time-varying')
                if iscell(extraOuts.hOF)
                    for hi = 1:length(extraOuts.hOF)
                        delete(extraOuts.hOF{hi})
                    end
                else
                    delete(extraOuts.hOF);
                end
            end
            if isfield(extraOuts, 'hVS')
                if iscell(extraOuts.hVS)
                    for hi = 1:length(extraOuts.hVS)
                        delete(extraOuts.hVS{hi})
                    end
                else
                    delete(extraOuts.hVS);
                end
            end
            if isfield(extraOuts, 'hVF')
                if iscell(extraOuts.hVF)
                    for hi = 1:length(extraOuts.hVF)
                        delete(extraOuts.hVF{hi})
                    end
                else
                    delete(extraOuts.hVF);
                end
            end
        end
        
        %---Perform Projections--------------------------------------------
        
        % Project
        if projDims == 0
            gPlot = g;
            dataPlot = data_i;
            
            if strcmp(obsMode, 'time-varying')
                obsPlot = obstacle_i;
            end
        else
            % if projpt is a cell, project each dimensions separately. This
            % allows us to take the union/intersection through some dimensions
            % and to project at a particular slice through other dimensions.
            if iscell(projpt)
                idx = find(plotDims==0);
                plotDimsTemp = ones(size(plotDims));
                gPlot = g;
                dataPlot = data_i;
                if strcmp(obsMode, 'time-varying')
                    obsPlot = obstacle_i;
                end
                
                for ii = length(idx):-1:1
                    plotDimsTemp(idx(ii)) = 0;
                    if strcmp(obsMode, 'time-varying')
                        [~, obsPlot] = proj(gPlot, obsPlot, ~plotDimsTemp,...
                            projpt{ii});
                    end
                    
                    [gPlot, dataPlot] = proj(gPlot, dataPlot, ~plotDimsTemp,...
                        projpt{ii});
                    plotDimsTemp = ones(1,gPlot.dim);
                end
                
            else
                [gPlot, dataPlot] = proj(g, data_i, ~plotDims, projpt);
                
                if strcmp(obsMode, 'time-varying')
                    %if strcmp(obsMode, 'time-varying')
                    [~, obsPlot] = proj(g, obstacle_i, ~plotDims, projpt);
                end
            end
            
            
        end
        
        
        

        %% Visualize Obstacle Function/Set
        
        %---Visualize Obstacle Set-----------------------------------------
        if strcmp(obsMode, 'time-varying') ...
                && isfield(extraArgs.visualize, 'obstacleSet') ...
                && extraArgs.visualize.obstacleSet
            
            % Visualize obstacle set (hOS)
            extraOuts.hOS = visSetIm(gPlot, obsPlot, ...
                extraArgs.visualize.plotColorOS, 0);%, eAO_visSetIm);
        end
        
        %---Visualize Obstacle Function------------------------------------
        if  strcmp(obsMode, 'time-varying') ...
                && isfield(extraArgs.visualize, 'obstacleFunction') ...
                && extraArgs.visualize.obstacleFunction
            
            % Visualize function
            [extraOuts.hOF]= visFuncIm(gPlot,-obsPlot,...
                extraArgs.visualize.plotColorOF,...
                extraArgs.visualize.plotAlphaOF);
        end
        %% Visualize Value Function/Set
        
        %---Visualize Value Set--------------------------------------------
        if isfield(extraArgs.visualize, 'valueSet') &&...
                extraArgs.visualize.valueSet
            
            extraOuts.hVS = visSetIm(gPlot, dataPlot, ...
                extraArgs.visualize.plotColorVS, sliceLevel, eAT_visSetIm);
        end
        
        
        %---Visualize Value Function---------------------------------------
        if isfield(extraArgs.visualize, 'valueFunction') && ...
                extraArgs.visualize.valueFunction
            % Visualize Target function (hTF)
            [extraOuts.hVF]= visFuncIm(gPlot,dataPlot,...
                extraArgs.visualize.plotColorVF,...
                extraArgs.visualize.plotAlphaVF);
            
        end

        %---Update Title---------------------------------------------------
        if ~isfield(extraArgs.visualize, 'dtTime')
            title(['t = ' num2str(tNow) ' s'])
        elseif isfield(extraArgs.visualize, 'dtTime') && floor(...
                extraArgs.visualize.dtTime/((tau(end)-tau(1))/length(tau))) ...
            == timeCount
            
            title(['t = ' num2str(tNow) ' s'])
            timeCount = 0;
        end
        drawnow;
        
        
        %---Save Video, Figure---------------------------------------------
        if isfield(extraArgs, 'makeVideo') && extraArgs.makeVideo
            current_frame = getframe(gcf); %gca does just the plot
            writeVideo(vout,current_frame);
        end
        
        if isfield(extraArgs.visualize, 'figFilename')
            export_fig(sprintf('%s%d', ...
                extraArgs.visualize.figFilename, i), '-png')
        end
    end
    
    %% Save the results if needed
    if isfield(extraArgs, 'saveFilename')
        if mod(i, extraArgs.saveFrequency) == 0
            ilast = i;
            save(extraArgs.saveFilename, 'data', 'tau', 'ilast', '-v7.3')
        end
    end
end

endTime = cputime;
if ~quiet
    fprintf('Total execution time %g seconds\n', endTime - startTime);
end
fprintf('Total execution time %g seconds\n', endTime - startTime);

if isfield(extraArgs, 'makeVideo') && extraArgs.makeVideo
    vout.close
end

end

function [dissFunc, integratorFunc, derivFunc] = ...
    getNumericalFuncs(dissType, accuracy)
% Dissipation
switch(dissType)
    case 'global'
        dissFunc = @artificialDissipationGLF_localQ;
    case 'local'
        dissFunc = @artificialDissipationLLF;
    case 'locallocal'
        dissFunc = @artificialDissipationLLLF;
    otherwise
        error('Unknown dissipation function %s', dissFunc);
end

% Accuracy
switch(accuracy)
    case 'low'
        derivFunc = @upwindFirstFirst;
        integratorFunc = @odeCFL1;
    case 'medium'
        derivFunc = @upwindFirstENO2;
        integratorFunc = @odeCFL2;
    case 'high'
        derivFunc = @upwindFirstENO3_localQ;
        integratorFunc = @odeCFL3_localQ;
    case 'veryHigh'
        derivFunc = @upwindFirstWENO5;
        integratorFunc = @odeCFL3;
    otherwise
        error('Unknown accuracy level %s', accuracy);
end
end

%% Function to get neighbors of a list of linearly indexed states.
%   linIndicies  -- list of linear indicies whose neighbors we are trying to get
%   dim          -- dimension in which we are looking for neighbors (e.g. x, y)
%                   dim needs to be between 1 and dimension of state
%   numNeighs    -- number of neighbors to find in the left (and right)
%                   direction.
%   stride       -- list containing size of each state
%   maxLinIdx    -- maximum linear idx value
function neighList = getNeigh(linIndicies, dim, numNeighs, stride, maxLinIdx, grid, periodicDim)
	[~,sz] = size(stride);
    
    % Check that the dimension is valid.
    if dim > sz || dim < 1
        error('Looking for neighbors in invalid dimension!', dim);
    end

    neighList = [];

    for n=1:numNeighs
      % Get right and left neighbors that are n gridcells away.
      if grid.dim == 3
        [subInd1, subInd2, subInd3] = ind2sub(grid.shape, linIndicies);
        subIndicies = [subInd1, subInd2, subInd3];
      else % 4D
        [subInd1, subInd2, subInd3, subInd4] = ind2sub(grid.shape, linIndicies);
        subIndicies = [subInd1, subInd2, subInd3, subInd4];
      end
      rightNeigh = subIndicies;
      leftNeigh = subIndicies;
      rightNeigh(:, dim) = rightNeigh(:, dim) + n;
      leftNeigh(:, dim) = leftNeigh(:, dim) - n;

      % Take into account periodic boundaries
      if dim == periodicDim
        rightNeigh(:, dim) = mod(rightNeigh(:, dim)-1, grid.shape(dim)) + 1;
        leftNeigh(:, dim) = mod(leftNeigh(:, dim)-1, grid.shape(dim)) + 1;
      end

      % Make sure we don't return neighbors that are outside of our
      % computation grid.
      valid_indicies = find((rightNeigh(:, dim) <= grid.shape(dim)) .* (rightNeigh(:, dim) > 0));
      if grid.dim == 3
        rightNeigh = sub2ind(grid.shape, rightNeigh(valid_indicies, 1), rightNeigh(valid_indicies, 2), rightNeigh(valid_indicies, 3));
      else
        rightNeigh = sub2ind(grid.shape, rightNeigh(valid_indicies, 1), rightNeigh(valid_indicies, 2), rightNeigh(valid_indicies, 3), rightNeigh(valid_indicies, 4));
      end
      neighList = [neighList; rightNeigh(:)];

      valid_indicies = find((leftNeigh(:, dim) <= grid.shape(dim)) .* (leftNeigh(:, dim) > 0));
      if grid.dim == 3
        leftNeigh = sub2ind(grid.shape, leftNeigh(valid_indicies, 1), leftNeigh(valid_indicies, 2), leftNeigh(valid_indicies, 3));
      else
        leftNeigh = sub2ind(grid.shape, leftNeigh(valid_indicies, 1), leftNeigh(valid_indicies, 2), leftNeigh(valid_indicies, 3), leftNeigh(valid_indicies, 4));
      end
      neighList = [neighList; leftNeigh(:)];
    end
end