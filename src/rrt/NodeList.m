classdef NodeList < handle
    % List of nodes.
    
    properties
        nodes
    end
    
    methods
        %% Constructor
        function obj = NodeList(xinit)
            obj.nodes = [Node(xinit)];
        end
        
        %% Inserts data into list.
        function insert(obj,x)
            newNode = Node(x);
            [~, closestNode] = obj.findNN(x);
            
            % if NOT inserting first node
            if ~isempty(closestNode)
                newNode.parent = closestNode;
            end
            obj.nodes = [obj.nodes, newNode];
        end
        
        %% Finds closest *Euclidian* point to x in list.
        function [minDist, closestNode] = findNN(obj, x)
            minDist = Inf;
            closestNode = [];
            for i=1:length(obj.nodes)
                dist = norm(x - obj.nodes(i).data);
                if dist < minDist
                    minDist = dist;
                    closestNode = obj.nodes(i);
                end
            end
        end
        
        %% Gets shortest Euclidian path through tree
        % Search backwards from goal to start to find the 
        % optimal least cost path
        function path = getPath(obj, xgoal)
            % NOTE: need xgoal to be in the tree...
            [~, goalNode] = obj.findNN(xgoal);
            if isempty(goalNode)
                error("Goal point is not in tree!");
            end
            revPath = {goalNode.data};
            currNode = goalNode.parent;
            while ~isempty(currNode)
                revPath{end+1} = currNode.data;
                currNode = currNode.parent;
            end
            % revserse order of points to get them from start to goal.
            path = flip(revPath);
        end
	end
end

