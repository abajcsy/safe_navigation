classdef KDTree < handle
    
    properties
        root        % (Node) root node
        numDims     % (int) num dimensions
    end
    
    methods
        %% Constructor.
        function obj = KDTree(xinit, numDims)
            obj.root = Node(xinit);
            obj.numDims = numDims;
        end
        
        %% Inserts data into tree.
        function insert(obj, x)
            obj.root = obj.insertHelper(x, obj.root, 1);
        end
        
        %% Helper function to insert point into tree.
        %   x       -- (array) point data
        %   tNode   -- (Node) that we are currently comparing to
        %   cutDim  -- (int) which dimension we are comparing with
        function tNode = insertHelper(obj, x, tNode, cutDim)
            nextCutDim = mod(cutDim, obj.numDims) + 1; 
            
            goLeft = x(cutDim) < tNode.data(cutDim);
            
            % add the state to the tree.
            if isempty(tNode)
                tNode = Node(x);
            elseif isequal(tNode.data, x)
                warning('Will not add duplicate node to tree.');
            elseif isempty(tNode.left) && isempty(tNode.right)
                newNode = Node(x);
                newNode.parent = tNode;
                if goLeft
                    tNode.left = newNode;
                else
                    tNode.right = newNode;
                end
            elseif isempty(tNode.left) && goLeft
                newNode = Node(x);
                newNode.parent = tNode;
                tNode.left = newNode;
            elseif isempty(tNode.right) && ~goLeft
                newNode = Node(x);
                newNode.parent = tNode;
                tNode.right = newNode;
            elseif goLeft
                tNode.left = obj.insertHelper(x,tNode.left, nextCutDim);
            else
                tNode.right = obj.insertHelper(x,tNode.right, nextCutDim);
            end
            
            % add the state to the tree.
%             if isempty(tNode)
%                 tNode = Node(x);
%             elseif isequal(tNode.data, x)
%                 warning('Will not add duplicate node to tree.');
%             elseif x(cutDim) < tNode.data(cutDim)
%                 tNode.left = obj.insertHelper(x,tNode.left, nextCutDim);
%             else
%                 tNode.right = obj.insertHelper(x,tNode.right, nextCutDim);
%             end
        end
        
        %% Finds closest *Euclidian* point to x in tree.
        function [bestDist, bestPt] = findNNEuclid(obj, x)
            [bestDist, bestPt] = obj.findNNEuclidHelper(x, obj.root, 1, [], Inf);
        end
        
        % Helper function to find euclidian closest point to x in tree.
        %   x       -- (array) point to find
        %   tNode   -- (Node) that we are currently comparing to
        %   cutDim  -- (int) which dimension we are comparing with
        function [bestDist, bestPt] = findNNEuclidHelper(obj, x, ...
                tNode, cutDim, bestPt, bestDist)
            % what if we are at null?
            if isempty(tNode)
                return;
            end
            
            % if we found the exact query point!
            if isequal(x, tNode.data)
                bestDist = 0.0;
                bestPt = tNode.data;
            end
            
            % we are at a leaf
            if isempty(tNode.left) && isempty(tNode.right)
                d = norm(tNode.data-x);
                % see if we have a better candidate pt
                if d < bestDist
                    bestDist = d;
                    bestPt = tNode.data;
                end
                return;
            end
            
            % see if we should search left of right first
            if x(cutDim) < tNode.data(cutDim)
                searchLeft = true;
            else
                searchLeft = false;
            end
            
            % recurse through the tree to find closest pt.
            nextCutDim = mod(cutDim, obj.numDims) + 1;  
            if searchLeft
                if (x(cutDim) - bestDist) <= tNode.data(cutDim)
                    [bestDist, bestPt] = obj.findNNEuclidHelper(x, tNode.left, nextCutDim, bestPt, bestDist);
                end
                if (x(cutDim) + bestDist) > tNode.data(cutDim)
                    [bestDist, bestPt] = obj.findNNEuclidHelper(x, tNode.right, nextCutDim, bestPt, bestDist);
                end
            else
                if (x(cutDim) + bestDist) > tNode.data(cutDim)
                    [bestDist, bestPt] = obj.findNNEuclidHelper(x, tNode.right, nextCutDim, bestPt, bestDist);
                end
                if (x(cutDim) - bestDist) <= tNode.data(cutDim)
                    [bestDist, bestPt] = obj.findNNEuclidHelper(x, tNode.left, nextCutDim, bestPt, bestDist);
                end
            end    
        end
        
        %% Returns a Node with data corresponding to state
        % if no such node exists, returns []
        function node = getNode(obj, x)
            node = obj.getNodeHelper(x, obj.root, 1);
        end
        
        function node = getNodeHelper(obj, x, tNode, cutDim)
            % what if we are at null?
            if isempty(tNode)
                node = [];
                return;
            end
            
            % if we found the exact query point!
            if isequal(x, tNode.data)
                node = tNode;
                return;
            end
            
            % see if we should search left of right first
            if x(cutDim) < tNode.data(cutDim)
                searchLeft = true;
            else
                searchLeft = false;
            end
            
            % recurse through the tree to find closest pt.
            nextCutDim = mod(cutDim, obj.numDims) + 1;  
            if searchLeft
                node = obj.getNodeHelper(x, tNode.left, nextCutDim);
                node = obj.getNodeHelper(x, tNode.right, nextCutDim);
            else
                node = obj.getNodeHelper(x, tNode.right, nextCutDim);
                node = obj.getNodeHelper(x, tNode.left, nextCutDim);
            end    
        end
        
%         %% Finds closest *Dubins* point to x in tree.
%         function [bestDist, bestPt] = findNNDubins(obj, x, pathGenerator)
%             [bestDist, bestPt] = obj.findNNDubinsHelper(x, obj.root, 1, [], Inf, pathGenerator);
%         end
%         
%         % Helper function to find closest dubins point to x in tree.
%         %   x       -- (array) point to find
%         %   tNode   -- (Node) that we are currently comparing to
%         %   cutDim  -- (int) which dimension we are comparing with
%         function [bestDist, bestPt] = findNNDubinsHelper(obj, x, ...
%                 tNode, cutDim, bestPt, bestDist, pathGenerator)
%             % what if we are at null?
%             if isempty(tNode)
%                 return;
%             end
%             % we are at a leaf
%             if isempty(tNode.left) && isempty(tNode.right)
%                 % query the dubins path generator to find a path from leaf 
%                 % to target point
%                [exists, ~, pathLen] = pathGenerator.getDubinsPath(tNode.data, x);
%                 % see if we have a better candidate pt
%                 if exists && sum(pathLen) < bestDist
%                     bestDist = totalCost;
%                     bestPt = tNode.data;
%                 end
%                 return;
%             end
%             
%             % see if we should search left of right first
%             if x(cutDim) < tNode.data(cutDim)
%                 searchLeft = true;
%             else
%                 searchLeft = false;
%             end
%             
%             % recurse through the tree to find closest pt.
%             nextCutDim = mod(cutDim, obj.numDims) + 1;  
%             if searchLeft
%                 if (x(cutDim) - bestDist) <= tNode.data(cutDim)
%                     [bestDist, bestPt] = obj.findNNDubinsHelper(x, tNode.left, nextCutDim, bestPt, bestDist, pathGenerator);
%                 end
%                 if (x(cutDim) + bestDist) > tNode.data(cutDim)
%                     [bestDist, bestPt] = obj.findNNDubinsHelper(x, tNode.right, nextCutDim, bestPt, bestDist, pathGenerator);
%                 end
%             else
%                 if (x(cutDim) + bestDist) > tNode.data(cutDim)
%                     [bestDist, bestPt] = obj.findNNDubinsHelper(x, tNode.right, nextCutDim, bestPt, bestDist, pathGenerator);
%                 end
%                 if (x(cutDim) - bestDist) <= tNode.data(cutDim)
%                     [bestDist, bestPt] = obj.findNNDubinsHelper(x, tNode.left, nextCutDim, bestPt, bestDist, pathGenerator);
%                 end
%             end    
%         end
    end
end

