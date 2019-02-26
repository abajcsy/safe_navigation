classdef OccuMap < handle
    % OccuMap represents a 2D occupancy map of an environment.
    
    properties
        grid            % (obj) Computation grid struct
        lReal           % (float arr) Cost function representing TRUE environment
        boundLow        % (x,y) lower corner of boundary obstacle (for numerics)
        boundUp         % (x,y) upper corner of boundary obstacle (for numerics)
        firstCompute    % (bool) flag to see if this is the first time we have done computation
        
        gFMM                    % Computation grid struct for FMM
        occupancy_map_safety    % (array) occupancy map used by safety module with -1 (obs) +1 (free)
        occupancy_map_plan      % (array) occupancy map ysed by planning with -1 (obs) +1 (free)
        sensed_region           % (array) map of sensed states with -1 (unsensed) +1 (sensed)
        signed_dist_safety      % signed distance function representing sensed environment
        
        occuMapSafeCellArr      % (cell arr) saves out the set of sensed states.
    end
    
    methods
        %% Constructor
        % Inputs:
        %   grid        -- (struct) computation grid struct with min/max,
        %                   dimensions, etc.
        %   obstacles   -- (cell arr) lower & upper bounds of each obstacle
        %                   used to construct ground-truth environment.
        %                   NOTE: supports only square obstacles right now.
        function obj = OccuMap(grid, obstacles)
            % Computation grid
            obj.grid = grid;
            obj.firstCompute = true;
            
            % Initialize signed-distance and occupancy grid variables.
            obj.gFMM                 = [];
            obj.signed_dist_safety   = [];
            obj.occupancy_map_safety = [];
            obj.occupancy_map_plan   = ones(transpose(obj.grid.N(1:2))); % everything is initially free for the planner
            obj.sensed_region        = -ones(transpose(obj.grid.N(1:2))); % everything is unsensed
            obj.occuMapSafeCellArr   = {};
            
            % Create ground-truth obstacle costmap.
            obj.lReal = [];
            for i=1:length(obstacles)
                lowObs = [obstacles{i}{1}; obj.grid.min(3)];
                upObs = [obstacles{i}{2}; obj.grid.max(3)];
                obsShape = shapeRectangleByCorners(obj.grid,lowObs,upObs);
                if isempty(obj.lReal)
                    obj.lReal = obsShape;
                else
                    obj.lReal = shapeUnion(obj.lReal, obsShape);
                end
            end

            % For numerics -- add a 1-grid-cell-sized obstacle along the 
            % edge of the compute grid. 
            offsetX = (obj.grid.max(1) - obj.grid.min(1))/obj.grid.N(1);
            offsetY = (obj.grid.max(2) - obj.grid.min(2))/obj.grid.N(2);
            if length(obj.grid.N) == 3
                obj.boundLow = [obj.grid.min(1)+offsetX, obj.grid.min(2)+offsetY, obj.grid.min(3)];
                obj.boundUp = [obj.grid.max(1)-offsetX, obj.grid.max(2)-offsetY, obj.grid.max(3)];
            else
                obj.boundLow = [obj.grid.min(1)+offsetX, obj.grid.min(2)+offsetY];
                obj.boundUp = [obj.grid.max(1)-offsetX, obj.grid.max(2)-offsetY];
            end
            % NOTE: need to negate the default shape function to make sure
            %       compute region is assigned (+) and boundary obstacle is (-)
            lBoundary = -shapeRectangleByCorners(obj.grid,obj.boundLow,obj.boundUp);
            
            % Incorporate boundary obstacle into the ground-truth obstacle
            obj.lReal = shapeUnion(obj.lReal, lBoundary);
        end
        
        %% Based on sensing, update occupancy grids and signed distances.
        % Inputs:
        %   senseData [vector]      - if rectangle sensing region, 
        %                           (x,y) coords of lower left sensing box and
        %                           (x,y) coords of upper right sensing box.
        %                           if circle sensing region, 
        %                           (x,y) coords of center and radius
        %   senseShape [string]     - either 'rectangle' or 'circle'
        function updateMapAndCost(obj, senseData, senseShape)
            % Construct cost function for region outside sensing radius
            % or 'known' environment.
            % 
            % NOTE: need to negate the default shape function to make sure
            %       free space is assigned (+) and unknown space is (-)
            if strcmp(senseShape, 'rectangle')
                lowSenseXY = senseData{1}(1:2);
                upSenseXY = senseData{2};
                lowObs = [lowSenseXY;obj.gridLow(3)];
                upObs = [upSenseXY;obj.gridUp(3)];
                sensingShape = -shapeRectangleByCorners(obj.grid,lowObs,upObs);
                
                % Record which states we have sensed. 
                % (+1 sensed, -1 unsensed)
                [~, dataSense] = proj(obj.grid, sensingShape, [0 0 1], 0);
                obj.sensed_region = max(obj.sensed_region, sign(dataSense));
                
                % Update occupancy grid with newly sensed obstacles.
                [~,realObs] = proj(obj.grid, obj.lReal, [0 0 1], 0);
                obj.occupancy_map_plan(find((obj.sensed_region > 0).*(realObs < 0))) = -1;
                
                % Union the sensed region with the actual obstacle.
                unionL = shapeUnion(sensingShape, obj.lReal);
                % Project the slice of unionL and create an occupancy map
                [obj.gFMM, dataFMM] = proj(obj.grid, unionL, [0 0 1], 0);
                obj.occupancy_map_safety = sign(dataFMM);
            elseif strcmp(senseShape, 'circle') % if circular sensing region
                center = senseData{1}(1:2);
                radius = senseData{2}(1);
                sensingShape = -shapeCylinder(obj.grid, 3, center, radius);
                
                % Record which states we have sensed. 
                % (+1 sensed, -1 unsensed)
                [~, dataSense] = proj(obj.grid, sensingShape, [0 0 1], 0);
                obj.sensed_region = max(obj.sensed_region, sign(dataSense));
                
                % Update occupancy grid with newly sensed obstacles.
                [~,realObs] = proj(obj.grid, obj.lReal, [0 0 1], 0);
                obj.occupancy_map_plan(find((obj.sensed_region > 0).*(realObs < 0))) = -1;
                
                % Union the sensed region with the actual obstacle.
                unionL = shapeUnion(sensingShape, obj.lReal);
                % Project the slice of unionL and create an occupancy map
                [obj.gFMM, dataFMM] = proj(obj.grid, unionL, [0 0 1], 0);
                obj.occupancy_map_safety = sign(dataFMM);
            elseif strcmp(senseShape, 'camera') % if camera sensing region
                % It is assumed that the obstacle is only position
                % dependent in this computation.
                
                % If this is the first computation, use a circle; 
                % otherwise use a camera
                if obj.firstCompute
                  sensingShape = -shapeCylinder(obj.grid, 3, senseData{1}(1:2), senseData{2}(2));
                  
                  % Record which states we have sensed. 
                  % (+1 sensed, -1 unsensed)
                  [~, dataSense] = proj(obj.grid, sensingShape, [0 0 1], 0);
                  obj.sensed_region = max(obj.sensed_region, sign(dataSense));

                  % Update occupancy grid with newly sensed obstacles.
                  [~,realObs] = proj(obj.grid, obj.lReal, [0 0 1], 0);
                  obj.occupancy_map_plan(find((obj.sensed_region > 0).*(realObs < 0))) = -1;
                  
                  % Union the sensed region with the actual obstacle.
                  unionL = shapeUnion(sensingShape, obj.lReal);
                  % Project the slice of unionL and create an occupancy map
                  [obj.gFMM, dataFMM] = proj(obj.grid, unionL, [0 0 1], 0);
                  obj.occupancy_map_safety = sign(dataFMM);
                  
                  % Do the actual camera sensing next time.
                  obj.firstCompute = false;
                else
                  % Project the slice of obstacle
                  [obj.gFMM, obsSlice] = proj(obj.grid, obj.lReal, [0 0 1], 0);
                  [obj.occupancy_map_safety, dataSense] = ...
                      generate_camera_sensing_region(obj.gFMM, obsSlice, ...
                      senseData{2}(1), senseData{1}(1:2), senseData{1}(3));
                  
                  % Record which states we have sensed. 
                  % (+1 sensed, -1 unsensed)                  
                  obj.sensed_region = max(obj.sensed_region, dataSense);
                  
                  % Update occupancy grid with newly sensed obstacles.
                  obj.occupancy_map_plan(find((obj.sensed_region > 0).*(obsSlice < 0))) = -1;
                end
            elseif strcmp(senseShape, 'lidar') % if lidar sensing region
                % It is assumed that the obstacle is only position
                % dependent in this computation.
                % Project the slice of obstacle
                [obj.gFMM, obsSlice] = proj(obj.grid, obj.lReal, [0 0 1], 0);
                [obj.occupancy_map_safety, dataSense] = ...
                    generate_lidar_sensing_region(obj.gFMM, obsSlice, ...
                    senseData{2}(1), senseData{1}(1:2));
              
                % Record which states we have sensed. 
                % (+1 sensed, -1 unsensed)                  
                obj.sensed_region = max(obj.sensed_region, dataSense);

                % Update occupancy grid with newly sensed obstacles.
                obj.occupancy_map_plan(find((obj.sensed_region > 0).*(obsSlice < 0))) = -1;
            else
               error('Unrecognized sensor type');
            end
            
            % Save out the sensed region for plotting. 
            obj.occuMapSafeCellArr{end+1} = obj.occupancy_map_safety;
            
            % We will use the FMM code to get the signed distance function. 
            % Since the FMM code works only on 2D, we will take a slice of 
            % the grid, compute FMM, and then project it back to a 3D array.
            unionL_2D_FMM = compute_fmm_map(obj.gFMM, obj.occupancy_map_safety);
            obj.signed_dist_safety = repmat(unionL_2D_FMM, 1, 1, obj.grid.N(3));
        end
    end
end

