classdef RRT < handle
    properties
        grid        % (x,y,theta) compute grid
        occuGrid    % (arr) occupancy grid full of -1's (obstacle) and +1's (free)
        pathHandle  % figure handle for path
        path        % (cell arr) current best path
        maxIter     % (int) maximum number of iterations before timing-out
        dx          % (float) how finely to collision-check along edges
    end
    
    methods
        %% Constructor.
        function obj = RRT(grid, occuGrid, maxIter, dx)
            load('seed.mat');
            % set random seed.
            rng(s);
            obj.grid = grid;
            obj.occuGrid = occuGrid;
            obj.path = [];
            obj.maxIter = maxIter;
            obj.dx = dx;
        end
        
        %% Replans path
        function path = replan(obj, xinit, xgoal)
            if ~isempty(obj.path)
                % sanity check -- is our old path still collision-free?
                collFree = obj.collisionCheckPath();
            else
                % definitely need to replan if we dont have a path.
                collFree = false;
            end
            
            % if old path isn't collision-free need to replan.
            if ~collFree
                showTree = false;
                nodes = obj.build(xinit, xgoal, showTree);
                obj.path = nodes.getPath(xgoal);
            end
            path = obj.path;
        end
        
        %% Creates the RRT starting from xinit. 
        % Inputs:
        %   xinit    -- initial state (x,y)
        %   xgoal    -- goal state (x,y)
        %   showTree -- bool flag to visualize tree as it grows
        % Outputs:
        %   nodes    -- list of nodes representing RRT from xinit to goal.
        function nodes = build(obj, xinit, xgoal, showTree)
            nodes = NodeList(xinit);
            i = 1;
            figure(1);
            goalEps = 0.3;
            hold on;
            while i <= obj.maxIter
                % gets random, collision-free state
                xrand = obj.getRandState();
                
                % find closest point to collision-free state
                [~,closestNode] = nodes.findNN(xrand);
                xclosest = closestNode.data;
                
                % if random point is already in tree, continue.
                if isequal(xclosest, xrand)
                    continue;
                end
                
                % check if path from the random point to the closest one is
                % collision-free
                collFree = obj.collisionCheckLine(xclosest, xrand);
                
                if collFree
                    
                    % plotting
                    if showTree
                        line([xclosest(1), xrand(1)], [xclosest(2), xrand(2)], ...
                            'Color', [0.5,0.5,0.5],  'LineWidth', 1);
                        pause(0.01);
                        xlim([obj.grid.min(1),obj.grid.max(1)]);
                        ylim([obj.grid.min(2),obj.grid.max(2)]);
                    end
                    
                    % insert collision-free point into tree.
                    nodes.insert(xrand);
                    
                    % if we found a path to the goal, terminate
                    dToGoal = norm(xrand - xgoal);
                    %fprintf("dist to goal: %f\n", dToGoal);
                    if dToGoal < goalEps %isequal(xrand, xgoal) 
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
            linIdx = randsample(prod(obj.grid.shape(1:2)), 1);
            [r,c] = ind2sub(obj.grid.shape(1:2), linIdx);
            gridIdx = [r,c];
            collisionFree = obj.collisionCheckPt(gridIdx);
            while ~collisionFree
                linIdx = randsample(prod(obj.grid.shape(1:2)), 1);
                [r,c] = ind2sub(obj.grid.shape(1:2), linIdx);
                gridIdx = [r,c];
                collisionFree = obj.collisionCheckPt(gridIdx);
            end
            % convert from grid index to real state value
            xrand = [obj.grid.xs{1}(r,c); obj.grid.xs{2}(r,c)];
        end
        
        %% Collision-check a point
        % Inputs:
        %   gridIdx         -- 3D index into obstacle map that represents 
        %                      (x,y) state to be collision-checked
        % Outputs:
        %   collisionFree   -- true if state is obs-free, false else 
        function collisionFree = collisionCheckPt(obj, gridIdx)
            val = obj.occuGrid(gridIdx(1), gridIdx(2));
            % obstacle map has values > 0 for free-space
            %                         < 0 for obstacle
            if val <= 0
                collisionFree = false;
            else
                collisionFree = true;
            end
        end
        
        %% Collision-check a line from xstart to xend.
        % Inputs:
        %   xstart          -- (x,y) start state of line
        %   xend            -- (x,y) end state of line
        %   dx              -- step size to move along line for collision
        %                       checking
        % Outputs:
        %   collisionFree   -- true if state is obs-free, false else 
        function collisionFree = collisionCheckLine(obj, xstart, xend)
            collisionFree = true;
            
            % line parametrization where 0 <= t <= 1
            % x = xstart + t*(diff);
            diff = xend - xstart;
            t = 0;
            x = xstart;
            while t < 1
                gridIdx = obj.stateToGrid(x);
                val = obj.occuGrid(gridIdx(1), gridIdx(2));
                if val <= 0
                    collisionFree = false;
                    return;
                end
                t = t+obj.dx;
                x = xstart + t*diff;
            end
        end
        
        %% Collision checks the current path
        function collisionFree = collisionCheckPath(obj)
            for i=1:length(obj.path)-1
                prevx = obj.path{i};
                nextx = obj.path{i+1};
                collFree = obj.collisionCheckLine(prevx,nextx);
                if ~collFree
                    collisionFree = false;
                    return;
                end
            end
            collisionFree = true;
        end
        
        %% Converts from (x,y) state to (i,j) grid index.
        function gridIdx = stateToGrid(obj, x)
            error = sqrt((obj.grid.xs{1} - x(1)).^2 + (obj.grid.xs{2} - x(2)).^2);
            [~,idx] = min(error(:));
            [row,col] = ind2sub(size(error),idx);
            gridIdx = [row, col];
            % Note:
            % to get the real-world (x,y) state that is closest to the 
            % [row, col], do:
            % xclosest = obj.grid.xs{1}(row,col)
            % yclosest = obj.grid.xs{2}(row,col)
        end
        
        %% Updates occupancy grid 
        function updateOccuGrid(obj, newOccuGrid)
            obj.occuGrid = newOccuGrid;
        end
        
        %% Plots the occupancy grid.
        function plotOccuGrid(obj)
            hold on
            [h,w] = size(obj.occuGrid);
            for i=1:h
                for j=1:w
                    % if at obstacle
                    if obj.occuGrid(i,j) < 0
                        state = [obj.grid.xs{1}(i,j), obj.grid.xs{2}(i,j)];
                        scatter(state(1), state(2), 'ks', 'filled');
                    end
                end
            end
            xlim([obj.grid.min(1),obj.grid.max(1)]);
            ylim([obj.grid.min(2),obj.grid.max(2)]);
        end
        
    end
   
end 