classdef Node < handle
   
    properties
        data        % (array)
        parent      % (Node) empty array [] if null, else the parent node.
        left        % (Node) empty array [] if null, else the left child.
        right       % (Node) empty array [] if null, else the right child.
    end
    
    methods
        %% Constructor
        function obj = Node(data)
            obj.data = data;
            obj.parent = [];
            obj.left = [];
            obj.right = [];
        end
        
        %% Left child. 
        function setLeftChild(obj,lChild)
            obj.left = lChild;
        end
        
        %% Right child. 
        function setRightChild(obj,rChild)
            obj.right = rChild;
        end
    end
end

