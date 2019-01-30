classdef Node < handle
    %NODE Node in AStar algorithm.
    
    properties
        parent      % [Node] parent node of this node
        position    % [vector] (X,Y) position of node
        totalCost   % [float] total cost of the node.
        dToStart    % [float] distance between the current node and the start node.
        dToGoal     % [float] the heuristic--estimated distance from the current node to the end node.
    end
    
    methods
        %% Constructor.
        function obj = Node(parent, position)
            % NOTE: parent can be NaN if this node has no parent.
            obj.parent = parent;
            obj.position = position;

            obj.totalCost = 0;
            obj.dToStart = 0;
            obj.dToGoal = 0;
        end
        
        %% Checks for equality between this node and another node.
        function bool = equal(obj, otherNode)
            bool = isequal(obj.position, otherNode.position);
        end
        
        %% Returns distance between positions of current and other node.
        function dist = distance(obj, otherNode)
            dist = (obj.position(1) - otherNode.position(1))^2 + ...
                (obj.position(2) - otherNode.position(2))^2;
        end
    end
end

