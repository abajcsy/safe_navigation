classdef SafetyModule < handle
    %AVOIDSET Computes and stores an avoid set.
    % Find the set of all states that can avoid a given set of	
    % states despite the disturbance for a time	duration of	T
    % 
    % min_u max_d min_{t \in [0,T]} l(x(t))
    %   s. t.   \dot{x} = f(x,u,d,t)
    %           L = {x : l(x) <= 0}
    
    properties
        % Computation grid & dynamics
        grid            % (obj) Computation grid struct
        dt              % (float) Timestep in discretization
        computeTimes    % (float arr) Stores the computation times
        dynSys          % (obj) Dynamical system (dubins car)
        uMode           % (string) is control max or min-ing l(x)? 
        dMode           % (string) is disturbance max or min-ing l(x)?
        
        % Cost function representation
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
        envType         % (string) what kind of environment are we doing safety for?
        
        % Utility variables and flags
        valueFunCellArr % (cell arr) stores sequence of V(x) 
        lxCellArr       % (cell arr) stores sequence of l(x)
        QSizeCellArr    % (cell arr) all queue size achieved for each timestep
        fovCellArr      % (cell arr) stores the results of FMM
        solnTimes       % (array) total time to compute solution at each step (in seconds)
        updateTimeArr   % (array) simulation timestamp for updating safe set 
    end
    
    methods
        %% Constructor. 
        % NOTE: Assumes DubinsCar or KinVehicle2D dynamics!
        function obj = SafetyModule(grid, dynSys, uMode, dMode, dt, ...
                updateEpsilon, warmStart, envType, updateMethod, tMax)
            % Setup computation grid.
            obj.grid = grid;
            
            obj.dt = dt;
            obj.updateEpsilon = updateEpsilon;
            obj.computeTimes = [];
            obj.envType = envType;
            
            % Store the current estimate of the cost function (from
            % sensing).
            obj.lCurr = [];
            
            % Before the problem starts, dont initialize the value
            % function.
            obj.valueFun = NaN;
            obj.warmStart = warmStart;
            
            % flag to tell us if this is the first time we have computed
            % lCurr or valueFun
            obj.firstCompute = true;
            
            % Setup dynamical system.
            obj.dynSys = dynSys;
            obj.uMode = uMode;
            
            % Setup distrubance if we are computing with it.
            if ~isempty(dMode)
                obj.dMode = dMode;
            end
            
            % Time vector.
            t0 = 0;
            obj.timeDisc = t0:obj.dt:tMax; 
            
            % Put grid and dynamic systems into schemeData.
            obj.schemeData.grid = obj.grid;
            obj.schemeData.dynSys = obj.dynSys;
            obj.schemeData.accuracy = 'high'; % Set accuracy.
            obj.schemeData.uMode = obj.uMode;
            if ~isempty(dMode)
                obj.schemeData.dMode = obj.dMode;
            end
            
            % Save out sequence of value functions as system moves through
            % space as well as the cost functions and the max Q size (if using Q method). 
            obj.valueFunCellArr = [];
            obj.lxCellArr = [];
            obj.QSizeCellArr = [];
            obj.solnTimes = [];
            obj.fovCellArr = [];
            obj.updateTimeArr = [];
            
            % Specify which update method we are using.
            obj.updateMethod = updateMethod;
            
            % since we have a finite compute grid, we may not want to 
            % trust values near the boundary of grid
            obj.HJIextraArgs.ignoreBoundary = 0; 
            
            % Convergence information
            if strcmp(obj.updateMethod, 'HJI')
                obj.HJIextraArgs.stopConverge = 1;
                obj.HJIextraArgs.convergeThreshold = obj.updateEpsilon;
            elseif strcmp(obj.updateMethod, 'localQ')
                obj.HJIextraArgs.stopConverge = 0;
                if obj.grid.dim == 3
                    obj.schemeData.hamFunc = @dubins3Dham_localQ;
                    obj.schemeData.partialFunc = @dubins3Dpartial_localQ;
                elseif obj.grid.dim == 4 %4D 
                    obj.schemeData.hamFunc = @plane4Dham_localQ;
                    obj.schemeData.partialFunc = @plane4Dpartial_localQ;
                else
                    error('I cannot run safety computation with a %dD system!', obj.grid.dim);
                end
            else
                msg = strcat('Your update method: ', obj.updateMethod, ... 
                    'is not a valid option.');
                error(msg);
            end
            
            fprintf('------ Avoid Set Problem Setup -------\n');
            fprintf('   dynamical system: %dD\n', length(obj.grid.N));
            fprintf('   update method: %s\n', obj.updateMethod);
            fprintf('   warm start: %d\n', obj.warmStart);
            fprintf('   stopConverge: %d\n', obj.HJIextraArgs.stopConverge);
            fprintf('   updateEpsilon: %.3f\n', obj.updateEpsilon);
            fprintf('--------------------------------------\n');
            
        end
        
        %% Computes avoid set. 
        % Inputs:
        %   occuMap             - (2D array) of occupancies in environment
        %                          -1 obs, +1 free
        %   gMap                - grid structure associated with occuMap
        % Outputs:
        %   dataOut             - infinite-horizon (converged) value function 
        function dataOut = computeAvoidSet(obj, signedDist, currTime)
            
            % Store the signed distance based on sensing info 
            % (for plotting, analysis etc.)
            %obj.fovCellArr{end+1} = signedDist;
            
            % ------------- CONSTRUCT l(x) ----------- %
            lxOld = obj.lCurr;
            if isempty(obj.lCurr) || strcmp(obj.envType, 'slam')
                % SLAM always gives us the full history of what the system 
                % has seen to be free, so just record that corresponding
                % signed distance function.
                obj.lCurr = signedDist;
            else
                obj.lCurr = shapeIntersection(signedDist, obj.lCurr);
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
                    if obj.grid.dim == 3
                        % if our system is 3D
                        data0 = obj.valueFun(:,:,:,end);
                    elseif obj.grid.dim == 4
                        % if our system is 4D
                        data0 = obj.valueFun(:,:,:,:,end);
                    else
                        error('Cannot update safe set for %dD system!', obj.grid.dim);
                    end
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
%                 whatRepo = what('safe_navigation');
%                 repo = whatRepo.path;
%                 %repo = '/Users/somil/Documents/Research/Projects/safe_navigation/safe_navigation';
%                 if obj.grid.dim == 3
%                     if strcmp(obj.envType, 'hand') 
%                         % if we are doing 3D system, we are doing the same
%                         % simulated environment --> can use this initial Vx
%                         pathToInitialVx = strcat(repo, '/initial_sets/vx3D_hand_withd_313121.mat');
%                     else
%                         error('You must recompute initial Vx for grid shape: %d, %d, %d', ...
%                                 obj.grid.shape(1), obj.grid.shape(2), obj.grid.shape(3));
%                     end
%                 elseif obj.grid.dim == 4
%                     if strcmp(obj.envType, 'sbpd')
%                         % if we are doing 4D simulation, it's in the
%                         % stanford building data set environment.
%                         if isequal(obj.grid.shape, [31 31 21 11])
%                             pathToInitialVx = strcat(repo, '/initial_sets/vx4D_SBPD_31312111.mat'); 
%                         elseif isequal(obj.grid.shape, [41 41 11 11])
%                             pathToInitialVx = strcat(repo, '/initial_sets/vx4D_SBPD_41411111.mat'); 
%                         elseif isequal(obj.grid.shape, [41 41 21 11])
%                             pathToInitialVx = strcat(repo, '/initial_sets/vx4D_SBPD_41412111.mat'); 
%                         else
%                             error('You must recompute initial Vx for grid shape: %d, %d, %d, %d', ...
%                                 obj.grid.shape(1), obj.grid.shape(2), obj.grid.shape(3), obj.grid.shape(4));
%                         end
%                     elseif strcmp(obj.envType, 'hand')
%                         % if we are doing 4D hand-designed environment,
%                         % load pre-mapped safe set.
%                         if isequal(obj.grid.shape, [31 31 21 11])
%                             pathToInitialVx = strcat(repo, '/initial_sets/vx4D_hand_31312111.mat');
%                         else
%                             error('You must recompute initial Vx for grid shape: %d, %d, %d, %d', ...
%                                 obj.grid.shape(1), obj.grid.shape(2), obj.grid.shape(3), obj.grid.shape(4));
%                         end
%                     elseif strcmp(obj.envType, 'slam')
%                         error('You need to compute the initial SLAM safe set!');
%                     else
%                         error('You must recompute initial Vx for env type: %s', obj.envType);
%                     end
%                 else
%                     error('We only support computation for 3D or 4D currently!');
%                 end
%                 load(pathToInitialVx);
%                 total_compute_t = 0;
                
                % (option 2) run the full, standard Vx computation
                firstHJIextraArgs = obj.HJIextraArgs;
                firstHJIextraArgs.stopConverge = 1;
                firstHJIextraArgs.convergeThreshold = 0.01;
                if obj.grid.dim == 3
                    firstHJIextraArgs.visualize.plotData.plotDims = [1 1 0];
                    firstHJIextraArgs.visualize.plotData.projpt = [0];
                elseif obj.grid.dim == 4
                    firstHJIextraArgs.visualize.plotData.plotDims = [1 1 0 0];
                    firstHJIextraArgs.visualize.plotData.projpt = [0 0.5];
                else
                    error('Unsure what states to project for visualization for %dD system.', ...
                        obj.grid.dim);
                end
                firstHJIextraArgs.visualize.valueSet = 1;
                firstWarmStart = false;
                [dataOut, tau, extraOuts] = ...
                HJIPDE_solve_warm(data0, lxOld, obj.lCurr, ...
                  obj.timeDisc, obj.schemeData, minWith, ...
                  firstWarmStart, firstHJIextraArgs);
            else
                %start_t = now;
                tic
                if strcmp(obj.updateMethod, 'HJI') 
                    % Use typical HJI solver (with or without warm start).
                    %[dataOut, tau, extraOuts] = ...
                    % HJIPDE_solve(data0, obj.timeDisc, obj.schemeData, ...
                    %    minWith, obj.HJIextraArgs);
                    [dataOut, tau, extraOuts] = ...
                     HJIPDE_solve_warm(data0, lxOld, obj.lCurr, ...
                       obj.timeDisc, obj.schemeData, minWith, ...
                       obj.warmStart, obj.HJIextraArgs);    
                elseif strcmp(obj.updateMethod, 'localQ')
                    % Use Q-based algorithm with initial Q constructed
                    % *locally* near newly sensed regions.
                    [dataOut, tau, extraOuts] = ...
                      HJIPDE_solve_localQ(data0, lxOld, obj.lCurr, ...
                        obj.updateEpsilon, obj.timeDisc, obj.schemeData, ...
                        minWith, obj.HJIextraArgs);
                end
                total_compute_t = toc;
            end

            % only save out the final, 'converged' value function
            if obj.grid.dim == 3
                obj.valueFunCellArr{end+1} = dataOut(:,:,:,end);
            else
                obj.valueFunCellArr{end+1} = dataOut(:,:,:,:,end);
            end
            obj.lxCellArr{end+1} = obj.lCurr;
            
            % Update internal variables.
            obj.valueFun = dataOut;
            obj.computeTimes = tau;
            obj.solnTimes = [obj.solnTimes, total_compute_t];
            obj.updateTimeArr = [obj.updateTimeArr, currTime];
            if exist('extraOuts', 'var') && isfield(extraOuts, 'QSizes')
                obj.QSizeCellArr{end+1} = extraOuts.QSizes;
            else
                error('No field extraOuts.QSizes!');
            end
            
            % We've computed our first BRT.
            obj.firstCompute = false;
        end
        
        %% Checks if state x is at the safety boundary. If it is, returns
        %  the optimal safety control to take. 
        function [uOpt, onBoundary, current_deriv] = checkAndGetSafetyControl(obj, x, tol)
            % Grab the value at state x from the most recent converged 
            % value function.
            if obj.grid.dim == 3
                vx = obj.valueFun(:,:,:,end);
            else
                vx = obj.valueFun(:,:,:,:,end);
            end
            value = eval_u(obj.grid, vx, x);
            
            % If the value is close to zero, we are close to the safety
            % boundary.
            if value < tol 
                deriv = computeGradients(obj.grid, vx);
                % value of the derivative at that particular state
                current_deriv = eval_u(obj.grid, deriv, x);
                % NOTE: need all 5 arguments (including NaN's) to get 
                % correct optimal control!
                uOpt = obj.dynSys.optCtrl(NaN, x, current_deriv, obj.uMode, NaN); 
                onBoundary = true;
                if iscell(uOpt)
                    uOpt = cell2mat(uOpt);
                end
            else
                current_deriv = [0.0;0.0;0.0;0.0];
                uOpt = zeros(length(x), 1);
                onBoundary = false;
            end
        end
        
        %% Gets the optimal control to apply at state x.
        function uOpt = getSafetyControl(obj, x)
            if obj.grid.dim == 3
                vx = obj.valueFun(:,:,:,end);
            else
                vx = obj.valueFun(:,:,:,:,end);
            end
            deriv = computeGradients(obj.grid, vx);
            % value of the derivative at that particular state
            current_deriv = eval_u(obj.grid, deriv, x);
            % NOTE: need all 5 arguments (including NaN's) to get 
            % correct optimal control!
            uOpt = obj.dynSys.optCtrl(NaN, x, current_deriv, obj.uMode, NaN);
            uOpt = cell2mat(uOpt);
        end
        
    end
end

