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
        
        % Utility variables and flags
        valueFunCellArr % (cell arr) stores sequence of V(x) 
        lxCellArr       % (cell arr) stores sequence of l(x)
        QSizeCellArr    % (cell arr) all queue size achieved for each timestep
        fovCellArr      % (cell arr) stores the results of FMM
        solnTimes       % (array) total time to compute solution at each step
        updateTimeArr   % (array) simulation timestamp for updating safe set
        
        unionL_2D_FMM           % Signed distance for the sensed occupancy map
    end
    
    methods
        %% Constructor. 
        % NOTE: Assumes DubinsCar or KinVehicle2D dynamics!
        function obj = SafetyModule(grid, dynSys, uMode, dt, ...
                updateEpsilon, warmStart, updateMethod, tMax)
            % Setup computation grid.
            obj.grid = grid;
            
            obj.dt = dt;
            obj.updateEpsilon = updateEpsilon;
            obj.computeTimes = [];
            
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
            
            % Time vector.
            t0 = 0;
            obj.timeDisc = t0:obj.dt:tMax; 
            
            % Put grid and dynamic systems into schemeData.
            obj.schemeData.grid = obj.grid;
            obj.schemeData.dynSys = obj.dynSys;
            obj.schemeData.accuracy = 'high'; % Set accuracy.
            obj.schemeData.uMode = obj.uMode;
            
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
                obj.schemeData.hamFunc = @dubins3Dham_localQ;
                obj.schemeData.partialFunc = @dubins3Dpartial_localQ;
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
            if isempty(obj.lCurr)
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
                    if length(obj.grid.N) == 2
                        % if our system is 2D
                        data0 = obj.valueFun(:,:,end);
                    else
                        % if our system is 3D
                        data0 = obj.valueFun(:,:,:,end);
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
                repo = what('safe_navigation');
                pathToInitialVx = strcat(repo.path, '/data/initialVx.mat');
                %pathToInitialVx = '../data/initialVx.mat';
                load(pathToInitialVx);
                total_compute_t = 0;
                
                % (option 2) run the full, standard Vx computation
                %firstHJIextraArgs = obj.HJIextraArgs;
                %firstHJIextraArgs.stopConverge = 1;
                %firstHJIextraArgs.convergeThreshold = 0.01;
                %[dataOut, tau, extraOuts] = ...
                %  HJIPDE_solve_warm(data0, lxOld, obj.lCurr, ...
                %    obj.timeDisc, obj.schemeData, minWith, firstHJIextraArgs);
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
            obj.valueFunCellArr{end+1} = dataOut(:,:,:,end);
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
        function [uOpt, onBoundary] = checkAndGetSafetyControl(obj, x, tol)
            % Grab the value at state x from the most recent converged 
            % value function.
            value = eval_u(obj.grid, obj.valueFun(:,:,:,end), x);
            
            % If the value is close to zero, we are close to the safety
            % boundary.
            if value < tol 
                deriv = computeGradients(obj.grid, obj.valueFun(:,:,:,end));
                % value of the derivative at that particular state
                current_deriv = eval_u(obj.grid, deriv, x);
                % NOTE: need all 5 arguments (including NaN's) to get 
                % correct optimal control!
                uOpt = obj.dynSys.optCtrl(NaN, x, current_deriv, obj.uMode, NaN); 
                onBoundary = true;
            else
                uOpt = zeros(length(x), 1);
                onBoundary = false;
            end
        end
        
    end
end

