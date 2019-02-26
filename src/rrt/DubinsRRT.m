classdef DubinsRRT < handle
    
    properties
        grid        % (x,y,theta) compute grid
        dynSys      % dynamical system object
        occuGrid    % occupancy grid 
        pathGenerator  % dubins path generator
    end
    
    methods
        %% Constructor.
        function obj = DubinsRRT(grid, dynSys, occuGrid)
            obj.grid = grid;
            obj.dynSys = dynSys;
            obj.occuGrid = occuGrid;
            obj.pathGenerator = DubinsPath(dynSys, occuGrid);
        end
        
        %% Creates the RRT starting from xinit. 
        % Inputs:
        %   xinit   -- initial state (x,y,theta)
        %   xgoal   -- goal state (x,y,theta)
        % Outputs:
        %   nodes    -- list of nodes representing RRT from xinit to goal.
        function nodes = buildRRT(obj, xinit, xgoal)
            nodes = NodeList(xinit);
            iterations = 1000;
            i = 1;
            while i <= iterations 
                % gets random, collision-free state
                xrand = obj.getRandState();
                
                % find closest point to collision-free state
                [~, xclosest] = nodes.findNNDubins(xrand, obj.pathGenerator);
                
                % generate dubins curve from xrand to the neighbor
                [exists, ~, ~]= obj.pathGenerator.getDubinsPath(xclosest, xrand);
                if exists
                    nodes.insert(bestPt);
                    % if we found a path to the goal, terminate
                    if isequal(bestPt, xgoal)
                        break;
                    end
                end
                i = i + 1;
            end
        end
        
        %% Gets a random collision-free state
        % Outputs:
        %   xrand   -- random state (x,y,theta) sampled from compute grid
        function xrand = getRandState(obj)
            % gets a random value from the integers 1 to n.
            linIdx = randsample(prod(obj.grid.shape), 1);
            [x,y,theta] = ind2sub(obj.grid.shape, linIdx);
            gridIdx = [x,y,theta];
            collisionFree = obj.collisionCheck(gridIdx);
            while ~collisionFree
                linIdx = randsample(prod(obj.grid.shape), 1);
                [x,y,theta] = ind2sub(obj.grid.shape, linIdx);
                gridIdx = [x,y,theta];
                collisionFree = obj.collisionCheck(gridIdx);
            end
            % convert from grid index to real state value
            xrand = [];
            for i=1:obj.grid.dim
                xrand = [xrand, obj.grid.xs{i}(gridIdx(i))];
            end
        end
        
        %% Collision-check a point
        % Inputs:
        %   gridIdx         -- 3D index into obstacle map that represents 
        %                   x,y,theta state to be collision-checked
        % Outputs:
        %   collisionFree   -- true if state is obs-free, false else 
        function collisionFree = collisionCheck(obj, gridIdx)
            val = obj.occuGrid(gridIdx(1), gridIdx(2));
            % obstacle map has values > 0 for free-space
            %                         < 0 for obstacle
            if val <= 0
                collisionFree = false;
            else
                collisionFree = true;
            end
        end
    end
   

end 