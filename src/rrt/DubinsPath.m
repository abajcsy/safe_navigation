classdef DubinsPath < handle
    % Computes shortest-distance dubins path from start to goal.
    % Assumptions:
    %   - car can only move forward
    %   - car can only move with unit velocity
    %   - minimal turning radius is 1 rad/s
    %
    % Equations based on:
    % http://mems.eng.uci.edu/files/2014/04/Dubins_Set_Robotics_2001.pdf
    properties
        dynSys 
        occuGrid
    end
    
    methods
        %% Constructor.
        function obj = DubinsPath(dynSys, occuGrid)
            obj.dynSys = dynSys;
            obj.occuGrid = occuGrid;
        end
        
        %% Find a collision-free dubins path from start state to end state
        % Inputs:
        %   xstart  -- starting state 
        %   xend    -- end state to find a collision-free path to
        % Outputs:
        %   exists  -- false, if no path, true is exists a path
        %   ctrl    -- [] if no path, sequence of controls if a path
        %   pathLen -- [] if no path, each path length. Apply corresponding
        %               ctrl for pathLen duration.
        function [exists, ctrl, pathLen] = getDubinsPath(obj, xstart, xend)
            % TODO: should this take as flag whether or not we want to
            % collision check?
            
            turnRad = obj.dynSys.wMax;
            dx = xend(1) - xstart(1);
            dy = xend(2) - xstart(2);
            
            % distance is shrunk by r, this make lengh calculation very easy
            d = sqrt(dx^2+dy^2)/turnRad; 
            theta = mod(atan2(dy,dx),2*pi);
            alpha = mod(xstart(3)-theta, 2*pi);
            beta = mod(xend(3)-theta, 2*pi);
            
            % compute all six possible paths
            [ctrl1,pathlen1] = obj.dubinsLSL(alpha, beta, d);
            [ctrl2,pathlen2] = obj.dubinsRSR(alpha, beta, d);
            [ctrl3,pathlen3] = obj.dubinsRSL(alpha, beta, d);
            [ctrl4,pathlen4] = obj.dubinsLSR(alpha, beta, d);
            [ctrl5,pathlen5] = obj.dubinsRLR(alpha, beta, d);
            [ctrl6,pathlen6] = obj.dubinsLRL(alpha, beta, d);
            
            ctrls = {ctrl1,ctrl2,ctrl3,ctrl4,ctrl5,ctrl6};
            pathLens = {pathlen1,pathlen2,pathlen3,pathlen4,pathlen5,pathlen6};
            
            % TODO: add in collision-checking!
            
            % find the best cost (i.e. shortest distance) path
            bestCtrl = [];
            bestCost = Inf;
            bestIdx = -1;
            for i=1:6
                cost = sum(pathLens{i});
                if cost < bestCost
                    bestCost = cost;
                    bestCtrl = ctrls{i};
                    bestIdx = i;
                end
            end
            
            % return if no path exists, or best path. 
            if isinf(bestCost)
                exists = false;
                ctrl = [];
                pathLen = [Inf, Inf, Inf]; 
            else
                exists = true;
                ctrl = bestCtrl;
                pathLen = pathLens{bestIdx};
            end
        end
        
        %% Computes LEFT-STRAIGHT-LEFT dubins path
        % The car moves for t+p+q length at unit velocity
        % 
        % Outputs:
        %   ctrl    -- sequence of angular controls to apply
        %   pathlen    -- [length of first path, second path, third part of path]
        function [ctrl,pathlen] = dubinsLSL(obj, alpha, beta, d)
            pSquared = 2 + d^2 - 2*cos(alpha - beta) + 2*d*(sin(alpha) - sin(beta));
            turnRad = obj.dynSys.wMax;
            if(pSquared < 0)
                ctrl = [];
                pathlen = [Inf, Inf, Inf];
                return;
            else
                p = sqrt(pSquared);
                atanStuff = atan2(cos(beta) - cos(alpha), d + sin(alpha) - sin(beta));
                t = mod(-alpha + atanStuff, 2*pi);
                q = mod(beta - atanStuff, 2*pi);
                ctrl = [turnRad,0,turnRad];
                pathlen = [t,p,q];
                return;
            end
        end
        
        %% Computes RIGHT-STRAIGHT-RIGHT dubins path
        % The car moves for t+p+q length at unit velocity
        % 
        % Outputs:
        %   ctrl    -- sequence of angular controls to apply
        %   pathlen    -- [length of first path, second path, third part of path]
        function [ctrl,pathlen] = dubinsRSR(obj, alpha, beta, d)
            pSquared = 2 + d^2 - 2*cos(alpha - beta) + 2*d*(sin(beta) - sin(alpha));
            turnRad = obj.dynSys.wMax;
            if(pSquared < 0)
                ctrl = [];
                pathlen = [Inf, Inf, Inf];
                return;
            else
                p = sqrt(pSquared);
                atanStuff = atan2(cos(alpha) - cos(beta), d - sin(alpha) + sin(beta));
                t = mod(alpha - atanStuff, 2*pi);
                q = mod(mod(-beta, 2*pi) + atanStuff, 2*pi);
                ctrl = [-turnRad,0,-turnRad];
                pathlen = [t,p,q];
                return;
            end
        end
        
        %% Computes RIGHT-STRAIGHT-LEFT dubins path
        % The car moves for t+p+q length at unit velocity
        % 
        % Outputs:
        %   ctrl    -- sequence of angular controls to apply
        %   pathlen    -- [length of first path, second path, third part of path]
        function [ctrl,pathlen] = dubinsRSL(obj, alpha, beta, d)
            pSquared = -2 + d^2 + 2*cos(alpha - beta) + 2*d*(sin(alpha) + sin(beta));
            turnRad = obj.dynSys.wMax;
            if(pSquared < 0)
                ctrl = [];
                pathlen = [Inf, Inf, Inf];
                return;
            else
                p = sqrt(pSquared);
                atanStuff = atan2(-cos(alpha)-cos(beta), d + sin(alpha) + cos(beta)) - ... 
                            atan2(-2,p);
                t = mod(-alpha + atanStuff, 2*pi);
                q = mod(mod(-beta, 2*pi) + atanStuff, 2*pi);
                ctrl = [-turnRad,0,turnRad];
                pathlen = [t,p,q];
                return;
            end
        end
        
        
        %% Computes LEFT-STRAIGHT-RIGHT dubins path
        % The car moves for t+p+q length at unit velocity
        % 
        % Outputs:
        %   ctrl    -- sequence of angular controls to apply
        %   pathlen    -- [length of first path, second path, third part of path]
        function [ctrl,pathlen] = dubinsLSR(obj, alpha, beta, d)
            pSquared = d^2 - 2 + 2*cos(alpha - beta) - 2*d*(sin(alpha) + sin(beta));
            turnRad = obj.dynSys.wMax;
            if(pSquared < 0)
                ctrl = [];
                pathlen = [Inf, Inf, Inf];
                return;
            else
                p = sqrt(pSquared);
                atanStuff = atan2(cos(alpha)+cos(beta), d - sin(alpha) - cos(beta)) + ... 
                            atan2(2,p);
                t = mod(alpha - atanStuff, 2*pi);
                q = mod(mod(beta, 2*pi) - atanStuff, 2*pi);
                ctrl = [turnRad,0,-turnRad];
                pathlen = [t,p,q];
                return;
            end
        end
        
        %% Computes RIGHT-LEFT-RIGHT dubins path
        % The car moves for t+p+q length at unit velocity
        % 
        % Outputs:
        %   ctrl    -- sequence of angular controls to apply
        %   pathlen    -- [length of first path, second path, third part of path]
        function [ctrl,pathlen] = dubinsRLR(obj, alpha, beta, d)
            tmp_lrl = (6 - d^2 + 2*cos(alpha - beta) + 2*d*(sin(alpha) - sin(beta))) / 8;
            turnRad = obj.dynSys.wMax;
            if(abs(tmp_lrl) > 1)
                ctrl = [];
                pathlen = [Inf, Inf, Inf];
                return;
            else
                p = mod(acos(tmp_lrl), 2*pi);
                t = mod((alpha - atan2(cos(alpha)-cos(beta), d-sin(alpha)+sin(beta)) + p/2), 2*pi);
                q = mod(alpha - beta - t + p, 2*pi);
                ctrl = [-turnRad,turnRad,-turnRad];
                pathlen = [t,p,q];
                return;
            end
        end
        
        %% Computes LEFT-RIGHT-LEFT dubins path
        % The car moves for t+p+q length at unit velocity
        % 
        % Outputs:
        %   ctrl    -- sequence of angular controls to apply
        %   pathlen    -- [length of first path, second path, third part of path]
        function [ctrl,pathlen] = dubinsLRL(obj, alpha, beta, d)
            tmp_lrl = (6 - d^2 + 2*cos(alpha - beta) + 2*d*(sin(alpha) - sin(beta))) / 8.;
            turnRad = obj.dynSys.wMax;
            if(abs(tmp_lrl) > 1)
                ctrl = [];
                pathlen = [Inf, Inf, Inf];
                return;
            else
                p = mod(acos(tmp_lrl), 2*pi);
                t = mod((-alpha + atan2(-cos(alpha)+cos(alpha), d+sin(alpha)-sin(beta)) + p/2), 2*pi);
                q = mod((mod(beta, 2*pi) - alpha + mod(2*p, 2*pi)), 2*pi);
                ctrl = [turnRad,-turnRad,turnRad];
                pathlen = [t,p,q];
                return;
            end
        end
    end

end