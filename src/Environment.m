classdef Environment < handle
    %ENVIRONMENT Represents (x,y) planar environment.
    
    properties
        lowEnv      % lower (x,y) corner of environment
        upEnv       % upper (x,y) corner of environment
        lowObs      % lower (x,y) of the rectangular obstacle 
        upObs       % upper (x,y) of the rectangular obstacle
        avoidSet    % stores avoidSet computed by HJIPDE_solve()
    end
    
    methods
        %% Constructor.
        function obj = Environment(lowEnv, upEnv, lowObs, upObs)
            obj.lowEnv = lowEnv;
            obj.upEnv = upEnv;
            obj.lowObs = lowObs;
            obj.upObs = upObs;
        end
        
        %% Plots the environment with the obstacle.
        % Return: h -- figure handle
        function h = plotEnvironment(obj)
            % draw *actual* obstacle
            width = obj.upObs(1) - obj.lowObs(1);
            height = obj.upObs(2) - obj.lowObs(2);
            obsCoord = [obj.lowObs(1), obj.lowObs(2), width, height];
            h = rectangle('Position', obsCoord, 'Linewidth', 2.0, 'LineStyle', '--'); 
            %h = rectangle('Position', obsCoord, 'Linewidth', 0.5, 'FaceColor',[0,0,0]);
            
            % setup the figure axes to represent the entire environment
            xlim([obj.lowEnv(1) obj.upEnv(1)]);
            ylim([obj.lowEnv(2) obj.upEnv(2)]);
            
            xlabel('x1');
            ylabel('x2');
            set(gca,'TickLength',[0 0]);
            box on
        end
        
        %% Computes intersection of obstacle box and sensing box. 
        % Returns: coordinates of box at the intersection, or NaN's.
        function [hasInter, lowXY, upXY] = obstacleIntersection(obj,lowSenseXY,upSenseXY)
            hasInter = false;
            lowXY = NaN;
            upXY = NaN;
            
            obsWidth = obj.upObs(1) - obj.lowObs(1);
            obsHeight = obj.upObs(2) - obj.lowObs(2);
            
            senseWidth = upSenseXY(1) - lowSenseXY(1);
            senseHeight = upSenseXY(2) - lowSenseXY(2);
            
            if lowSenseXY(1) < obj.lowObs(1) + obsWidth && ...
               lowSenseXY(1) + senseWidth > obj.lowObs(1) && ...
               lowSenseXY(2) < obj.lowObs(2) + obsHeight && ...
               lowSenseXY(2) + senseHeight > obj.lowObs(2)
                % Determine the coordinates of the intersection rectangle.
                lowXY = [max(lowSenseXY(1), obj.lowObs(1)); max(lowSenseXY(2), obj.lowObs(2))];
                upXY = [min(upSenseXY(1), obj.upObs(1)); min(upSenseXY(2), obj.upObs(2))];
                hasInter = true;
            end
            
        end
    end
end

