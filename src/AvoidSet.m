classdef AvoidSet < handle
    %AVOIDSET Computes and stores an avoid set.
    % Find the set of all states that can avoid a given set of	
    % states despite the disturbance for a time	duration of	T
    % 
    % min_u max_d min_{t \in [0,T]} l(x(t))
    %   s. t.   \dot{x} = f(x,u,d,t)
    %           L = {x : l(x) <= 0}
    
    properties
        % Computation grid & dynamics
        gridLow         % (float arr) Lower corner of computation domain
        gridUp          % (float arr) Upper corner of computation domain
        N               % (int arr) Number of grid points per dimension
        grid            % (obj) Computation grid struct
        dt              % (float) Timestep in discretization
        computeTimes    % (float arr) Stores the computation times
        dynSys          % (obj) Dynamical system (dubins car)
        pdDims          % (int) Which dimension is periodic?
        uMode           % (string) is control max or min-ing l(x)? 
        dMode           % (string) is disturbance max or min-ing l(x)? 
        boundLow        % (arr) lower limit of boundary padding
        boundUp         % (arr) upper limit of boundary padding 
        
        % Environment information
        lowRealObs      % (float arr) (x,y) lower left corner of obstacle
        upRealObs       % (float arr) (x,y) upper right corner of obstacle
        lReal           % (float arr) Cost function representing TRUE environment
        lCurr           % (float arr) Current cost function representation
                        %             (constructed from measurements)
                        
        % Value function computation information
        valueFun        % (float arr) Stores most recent converged value function 
        timeDisc        % (float arr) Discretized time vector          
        schemeData      % (struct) Used to specify dyn, grid, etc. for HJIPDE_solve()
        HJIextraArgs    % (struct) Specifies extra args to HJIPDE_solve()
        updateEpsilon   % (float) Value change threshold used in local update rule
        warmStart       % (bool) if we want to warm start with prior V(x)
        firstCompute    % (bool) flag to see if this is the first time we have done computation
        updateMethod    % (string) what kind of solution method to use
        inheritVals     % (string) if using localQ update method, pick where to inherit values from
        
        % Utility variables and flags
        saveValueFuns   % (bool) flag to save out sequence of V(x) and l(x)
        runComparison   % (bool) flag to run computational comparisons
        valueFunCellArr % (cell arr) stores sequence of V(x) 
        lxCellArr       % (cell arr) stores sequence of l(x)
        maxQSize        % (arr) Maximum queue size achieved for each timestep
        occup
        gFMM            % Computation grid struct for FMM
        unionL_2D_FMM   % Signed distance for the sensed occupancy map
    end
    
    methods
        %% Constructor. 
        % NOTE: Assumes DubinsCar dynamics!
        function obj = AvoidSet(gridLow, gridUp, lowRealObs, upRealObs, ...
                obsShape, xinit, N, dt, updateEpsilon, ...
                warmStart, saveValueFuns, runComparison, ...
                inheritVals, updateMethod)
            obj.gridLow = gridLow;  
            obj.gridUp = gridUp;    
            obj.N = N;      
            obj.pdDims = 3;      % 3rd dimension is periodic for dubins
            obj.dt = dt;
            obj.updateEpsilon = updateEpsilon;
            obj.grid = createGrid(obj.gridLow, obj.gridUp, obj.N, obj.pdDims);
            obj.computeTimes = [];
            
            % Store obstacle coordinates.
            obj.lowRealObs = lowRealObs;
            obj.upRealObs = upRealObs;
            
            % Create the 'ground-truth' cost function from obstacle.
            if strcmp(obsShape, 'rectangle')
                lowObs = [lowRealObs;gridLow(3)];
                upObs = [upRealObs;gridUp(3)];
                obj.lReal = shapeRectangleByCorners(obj.grid,lowObs,upObs);
            else % if circular obstacle
                center = lowRealObs;
                radius = upRealObs(1,1);
                obj.lReal = shapeCylinder(obj.grid, 3, center, radius);
            end
            
            % For numerics -- add a 1-grid-cell-sized obstacle along the 
            % edge of the compute grid. 
            offsetX = (gridUp(1) - gridLow(1))/N(1);
            offsetY = (gridUp(2) - gridLow(2))/N(2);
            obj.boundLow = [gridLow(1)+offsetX, gridLow(2)+offsetY, gridLow(3)];
            obj.boundUp = [gridUp(1)-offsetX, gridUp(2)-offsetY, gridUp(3)];
            % NOTE: need to negate the default shape function to make sure
            %       compute region is assigned (+) and boundary obstacle is (-)
            lBoundary = -shapeRectangleByCorners(obj.grid,obj.boundLow,obj.boundUp);
            
            % Incorporate boundary obstacle into the ground-truth obstacle
            obj.lReal = shapeUnion(obj.lReal, lBoundary);
            
            % Store the current estimate of the cost function (from
            % sensing).
            obj.lCurr = NaN;
            
            % Before the problem starts, dont initialize the value
            % function.
            obj.valueFun = NaN;
            obj.warmStart = warmStart;
            
            % flag to tell us if this is the first time we have computed
            % lCurr or valueFun
            obj.firstCompute = true;
            
            % Input bounds for dubins car.
            wMax = 1;
            vrange = [0.5,1];
            
            % --- DISTURBANCE --- %
            obj.dMode = 'min';
            dMax = [0,0,0]; %[.2, .2, .2];
            % ------------------- %

            % Define dynamic system.            
            % Create dubins car where u = [v, w]
            obj.dynSys = Plane(xinit, wMax, vrange, dMax);
            
            % Time vector.
            t0 = 0;
            tMax = 100; % tMax = 100 means compute infinite-horizon solution
            obj.timeDisc = t0:obj.dt:tMax; 
            
            % Control is trying to maximize value function.
            obj.uMode = 'max';
            
            % Put grid and dynamic systems into schemeData.
            obj.schemeData.grid = obj.grid;
            obj.schemeData.dynSys = obj.dynSys;
            obj.schemeData.accuracy = 'high'; % Set accuracy.
            obj.schemeData.uMode = obj.uMode;
            obj.schemeData.dMode = obj.dMode;
            
            % Save out sequence of value functions as system moves through
            % space as well as the cost functions and the max Q size (if using Q method). 
            obj.saveValueFuns = saveValueFuns;
            obj.valueFunCellArr = [];
            obj.lxCellArr = [];
            obj.maxQSize = [];
            
            % For local update, specify where we update values from.
            obj.inheritVals = inheritVals;
            obj.updateMethod = updateMethod;
            
            % since we have a finite compute grid, we may not want to 
            % trust values near the boundary of grid
            obj.HJIextraArgs.ignoreBoundary = 0; 
            
            % Convergence information
            if strcmp(obj.updateMethod, 'HJI')
                obj.HJIextraArgs.stopConverge = 1;
                obj.HJIextraArgs.convergeThreshold = obj.updateEpsilon;
            elseif strcmp(updateMethod, 'globalQ') || strcmp(obj.updateMethod, 'localQ')
                obj.HJIextraArgs.stopConverge = 0;
            else
                msg = strcat('Your update method: ', obj.updateMethod, ... 
                    'is not a valid option.');
                error(msg);
            end
            
            fprintf('------ Avoid Set Problem Setup -------\n');
            fprintf('   update method: %s\n', obj.updateMethod);
            fprintf('   warm start: %d\n', obj.warmStart);
            fprintf('   stopConverge: %d\n', obj.HJIextraArgs.stopConverge);
            fprintf('   updateEpsilon: %.3f\n', obj.updateEpsilon);
            fprintf('--------------------------------------\n');
            
            % Grab the ground truth value functions over time
            obj.runComparison = runComparison;
            if obj.runComparison
                repo = what('safe_navigation');
                valFunPath = strcat(repo.path, '/data/groundTruthValueFuns.mat');
                load(valFunPath, 'valueFunCellArr');
                obj.valueFunCellArr = valueFunCellArr;
            end
            
        end
        
        %% Computes avoid set. 
        % Inputs:
        %   senseData [vector]      - if rectangle sensing region, 
        %                           (x,y) coords of lower left sensing box and
        %                           (x,y) coords of upper right sensing box.
        %                           if circle sensing region, 
        %                           (x,y) coords of center and radius
        %   senseShape [string]     - either 'rectangle' or 'circle'
        %   currTime [int]          - current timestep (in simulation) 
        % Outputs:
        %   dataOut             - infinite-horizon (converged) value function 
        function dataOut = computeAvoidSet(obj, senseData, senseShape, ...
                currTime)
            
            % ---------- START CONSTRUCT l(x) ---------- %
            
            % Construct cost function for region outside sensing radius
            % or 'known' environment.
            % NOTE: need to negate the default shape function to make sure
            %       free space is assigned (+) and unknown space is (-)
            if strcmp(senseShape, 'rectangle')
                lowSenseXY = senseData{1}(1:2);
                upSenseXY = senseData{2};
                lowObs = [lowSenseXY;obj.gridLow(3)];
                upObs = [upSenseXY;obj.gridUp(3)];
                sensingShape = -shapeRectangleByCorners(obj.grid,lowObs,upObs);
                % Union the sensed region with the actual obstacle.
                unionL = shapeUnion(sensingShape, obj.lReal);
                % Project the slice of unionL and create an occupancy map
                [obj.gFMM, dataFMM] = proj(obj.grid, unionL, [0 0 1], 0);
                occupancy_map = sign(dataFMM);
            elseif strcmp(senseShape, 'circle') % if circular sensing region
                center = senseData{1}(1:2);
                radius = senseData{2}(1);
                sensingShape = -shapeCylinder(obj.grid, 3, center, radius);
                % Union the sensed region with the actual obstacle.
                unionL = shapeUnion(sensingShape, obj.lReal);
                % Project the slice of unionL and create an occupancy map
                [obj.gFMM, dataFMM] = proj(obj.grid, unionL, [0 0 1], 0);
                occupancy_map = sign(dataFMM);
            elseif strcmp(senseShape, 'camera') % if camera sensing region
                % It is assumed that the obstacle is only position
                % dependent in this computation.
                
                % If this is the first computation, use a circle; 
                % otherwise use a camera
                if obj.firstCompute
                  sensingShape = -shapeCylinder(obj.grid, 3, senseData{1}(1:2), senseData{2}(2));
                  % Union the sensed region with the actual obstacle.
                  unionL = shapeUnion(sensingShape, obj.lReal);
                  % Project the slice of unionL and create an occupancy map
                  [obj.gFMM, dataFMM] = proj(obj.grid, unionL, [0 0 1], 0);
                  occupancy_map = sign(dataFMM);
                else
                  % Project the slice of obstacle
                  [obj.gFMM, obsSlice] = proj(obj.grid, obj.lReal, [0 0 1], 0);
                  occupancy_map = generate_camera_sensing_region(obj.gFMM, ...
                    obsSlice, senseData{2}(1), senseData{1}(1:2), senseData{1}(3));
                end
            else
               error('Unrecognized sensor type');
            end
            
            % --- LOCAL UPDATE --- % 
            % store old cost function
            lxOld = obj.lCurr;
            % -------------------- %
            
            % We will use the FMM code to get the signed distance function. 
            % Since the FMM code works only on 2D, we will take a slice of 
            % the grid, compute FMM, and then project it back to a 3D
            % array.
            obj.unionL_2D_FMM = compute_fmm_map(obj.gFMM, occupancy_map);
            unionL = repmat(obj.unionL_2D_FMM, 1, 1, obj.grid.N(3));
            
            if isnan(obj.lCurr)
                obj.lCurr = unionL;
            else
                obj.lCurr = shapeIntersection(unionL, obj.lCurr);
            end
            
            % ------------- CONSTRUCT V(x) ----------- %
            if obj.firstCompute
                % First time we are doing computation, set data0 to lcurr
                % no matter our update method. 
                data0 = obj.lCurr;
            else
                if obj.warmStart
                    % If we are warm starting, use the old value function
                    % as initial V(x) and then the true/correct l(x) in targets
                    data0 = obj.valueFun(:,:,:,end);
                else
                    data0 = obj.lCurr;
                end
            end

            % Make sure that we store the true cost function 
            % so we can do min with L.
            obj.HJIextraArgs.targets = obj.lCurr;
            
            % We can use min with l regardless of if we are warm
            % starting (but we need to set targets = l!) 
            minWith = 'minVWithL';
            
            
            % ------------ Compute value function ---------- % 
            if obj.firstCompute 
                % (option 1) load offline-computed infinite-horizon safe set
                repo = what('safe_navigation');
                pathToInitialVx = strcat(repo.path, '/data/initialVx.mat');
                %pathToInitialVx = '../data/initialVx.mat';
                load(pathToInitialVx);
                
                % (option 2) run the full, standard Vx computation
                %[dataOut, tau, extraOuts] = ...
                %  HJIPDE_solve(data0, obj.timeDisc, obj.schemeData, ...
                %   minWith, obj.HJIextraArgs);
            else
                if strcmp(obj.updateMethod, 'HJI') 
                    % Use typical HJI solver.
                    [dataOut, tau, extraOuts] = ...
                      HJIPDE_solve(data0, obj.timeDisc, obj.schemeData, ...
                       minWith, obj.HJIextraArgs);
                elseif strcmp(obj.updateMethod, 'globalQ')
                    % Use Q-based algorithm with initial Q constructed
                    % *globally*. 
                    [dataOut, tau, extraOuts] = ...
                      HJIPDE_solve_globalQ(data0, lxOld, obj.lCurr, ...
                        obj.updateEpsilon, obj.timeDisc, obj.schemeData, ...
                        minWith, obj.HJIextraArgs);
                elseif strcmp(obj.updateMethod, 'localQ')
                    % For local update, specify where we update values from.
                    obj.HJIextraArgs.inheritVals = obj.inheritVals;
                    
                    % Use Q-based algorithm with initial Q constructed
                    % *locally* near newly sensed regions.
                    [dataOut, tau, extraOuts] = ...
                      HJIPDE_solve_localQ(data0, lxOld, obj.lCurr, ...
                        obj.updateEpsilon, obj.timeDisc, obj.schemeData, ...
                        minWith, obj.HJIextraArgs);
                end
            end
            
            % --- compare solution with ground truth? --- %
            if obj.runComparison
                % Grab the ground truth update at same timestep for comparison.
                groundTruth = obj.valueFunCellArr{currTime};

                % Compare the local update to the ground-truth.
                obj.compareSolutions(dataOut(:,:,:,end), groundTruth(:,:,:,end));
            end

            % --- save out computed value function? --- %
            if obj.saveValueFuns 
                obj.valueFunCellArr{end+1} = dataOut;
                obj.lxCellArr{end+1} = obj.lCurr;
            end
            
            % Update internal variables.
            obj.valueFun = dataOut;
            obj.computeTimes = tau;
            obj.firstCompute = false;
            if exist('extraOuts', 'var') && isfield(extraOuts, 'maxQSize')
              obj.maxQSize = [obj.maxQSize, extraOuts.maxQSize];
            else
              obj.maxQSize = [obj.maxQSize, 0];
            end
        end
        
        %% Compare normal and local update solution
        function compareSolutions(obj, VxNormal, VxLocal)
            % we want intersection to be empty.
            intersect = max(VxNormal, -VxLocal);
            %intersect(find(intersect < 0))
            theta = pi/2;
            [gPlot, plotData] = proj(obj.grid, intersect, [0 0 1], theta);
            h = visSetIm(gPlot, plotData, 'k', 0);
            delete(h);
        end
        
        %% Checks if state x is at the safety boundary. If it is, returns
        %  the optimal safety control to take. 
        function [uOpt, onBoundary] = checkAndGetSafetyControl(obj, x)
            % Grab the value at state x from the most recent converged 
            % value function.
            value = eval_u(obj.grid, obj.valueFun(:,:,:,end), x);
            tol = 0.1;
            
            % If the value is close to zero, we are close on the safety
            % boundary.
            if value < tol 
                deriv = computeGradients(obj.grid, obj.valueFun(:,:,:,end));
                % value of the derivative at that particular state
                current_deriv = eval_u(obj.grid, deriv, x);
                uOpt = obj.dynSys.optCtrl(x, current_deriv, obj.uMode); 
                onBoundary = true;
            else
                uOpt = zeros(2, 1);
                onBoundary = false;
            end
        end
        
        %% Get shape that represents sensed part of obstacle.
        function sensedObsShape = getSensedObs(obj, senseData)
            % NOTE: assumes circular sensing radius.
            center = senseData(:,1);
            radius = senseData(1,2);
            sensingShape = shapeCylinder(obj.grid, 3, center, radius);
            
            % Union the sensed region with the actual obstacle.
            sensedObsShape = shapeIntersection(sensingShape, obj.lReal);
        end
        
        %% Used to analyze if the error induced by warm-starting vanishes.
        % Let V(x) be the (converged) value function computed with the old 
        %   sensing information (i.e. using the old l(x)). 
        % Let V'(x) be the new value function we are computing based on new
        %   sensing information (i.e. using the new l'(x))
        % Typically, we would initialize:
        %       V'(x,T) = l'(x)
        % Warm-starting allows us to initialize with:
        %       V'(x,T) = V(x)
        % Now we can define the error in the initial value function as:
        %       k(x) = V(x) / l'(x)
        % This defines an error set that we would like to ensure goes to
        % the empty set as t-->infinity. This function performs that check.
        function checkIfErrorVanishes(obj, senseData, theta)
            % Construct the new l'(x) from senseData.
            center = senseData(:,1);
            radius = senseData(1,2);
            sensingShape = -shapeCylinder(obj.grid, 3, center, radius);

            % Union the sensed region with the actual obstacle.
            unionL = shapeUnion(sensingShape, obj.lReal);
            if isnan(obj.lCurr)
                lprimex = unionL;
            else
                lprimex = shapeIntersection(unionL, obj.lCurr);
            end
            
            % Grab the prior value function.
            Vx = obj.valueFun(:,:,:,end);
        
            % Get the error in the initial value function.
            kx = shapeDifference(Vx, lprimex); 
            
            % visualize that set
            % contourf plots all values that are ABOVE zero
            figure(2);
            extraArgs.LineWidth = 2;
            [gPlot, dataPlot] = proj(obj.grid, lprimex, [0 0 1], theta);
            visSetIm(gPlot, dataPlot, [1,0,0], 0, extraArgs);
            title('lprime(x) sub-zero level set');
            
            figure(3);
            extraArgs.LineWidth = 2;
            [gPlot, dataPlot] = proj(obj.grid, Vx, [0 0 1], theta);
            visSetIm(gPlot, dataPlot, [0,0,1], 0, extraArgs);
            title('V(x) sub-zero level set');
            
            figure(4);
            [gPlot, dataPlot] = proj(obj.grid, kx, [0 0 1], theta);
            visSetIm(gPlot, dataPlot, [0,1,0], 0, extraArgs);
            title('k(x) sub-zero level set');
            
            % Put grid and dynamic systems into schemeData.
            scheme.grid = obj.grid;
            scheme.dynSys = obj.dynSys;
            scheme.accuracy = 'high'; % Set accuracy.
            scheme.uMode = 'max';
            scheme.dMode = 'min';

            % Convergence information
            extraArgs.stopConverge = 1;
            extraArgs.convergeThreshold = .01; 
            extraArgs.visualize.valueSet = 1;
            %obj.HJIextraArgs.visualize.initialValueSet = 1;
            extraArgs.visualize.figNum = 5; %set figure number
            extraArgs.visualize.deleteLastPlot = true; %delete previous plot as you update
            extraArgs.visualize.sliceLevel = theta;
            extraArgs.ignoreBoundary = 1;
            
            % Time vector.
            t0 = 0;
            tMax = 100; 
            times = t0:obj.dt:tMax; 
                        
            % Setup solver parameters.
            minWith = 'set';
            
            % Solve infinite-horizon BRS computation.
            [dataOut, tau, extraOuts] = ...
              HJIPDE_solve(kx, times, scheme, minWith, extraArgs);
        end
    end
end

