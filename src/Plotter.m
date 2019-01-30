classdef Plotter
    %PLOTTER Plots level sets and environment and stuff.
    
    properties
        lowEnv      % lower (x,y) corner of environment
        upEnv       % upper (x,y) corner of environment
        lowObs      % if rectangle: lower left (x,y) point
                    % if circle: center (x,y) of circle
        upObs       % if rectangle: upper right (x,y) point
                    % if circle: radius of circle 
        obsShape    % shape of the obstacle (rectangle or circle)
    end
    
    methods
        %% Constructor.
        function obj = Plotter(lowEnv, upEnv, lowObs, upObs, obsShape)
            obj.lowEnv = lowEnv;
            obj.upEnv = upEnv;
            obj.lowObs = lowObs;
            obj.upObs = upObs;
            obj.obsShape = obsShape;
        end
        
        %% Plots the environment with the obstacle.
        % Output: 
        %   e - figure handle
        function e = plotEnvironment(obj)
            % draw *actual* obstacle
            if strcmp(obj.obsShape, 'rectangle')
                width = obj.upObs(1) - obj.lowObs(1);
                height = obj.upObs(2) - obj.lowObs(2);
                obsCoord = [obj.lowObs(1), obj.lowObs(2), width, height];
                e = rectangle('Position', obsCoord, 'Linewidth', 2.0, 'LineStyle', '--'); 
            else
                center = [obj.lowObs(1), obj.lowObs(2)];
                rad = obj.upObs(1);
                e = viscircles(center,rad, 'Color', 'k', 'Linewidth', 2.0, 'LineStyle', '--');
            end
            
            % Setup the figure axes to represent the entire environment
            xlim([obj.lowEnv(1) obj.upEnv(1)]);
            ylim([obj.lowEnv(2) obj.upEnv(2)]);
            
            xlabel('x');
            ylabel('y');
            set(gca,'TickLength',[0 0]);
            box on
        end
        
        %% Plot level set of arbitrary function
        % Inputs:
        %   g [array]     - grid corresponding to data
        %   func [array]  - data for function to visualize
        %   theta [float] - angle for which to plot the level set
        %   visSet [bool] - if true, plots 2D slice of func.
        %                   Otherwise plots 3D.
        %   edgeColor [vector or string] - color of level set boundary
        %   cmap [string] - name of colormap to use
        % Outputs: 
        %   plots level set in (x,y) for fixed theta.
        function h = plotFuncLevelSet(obj, g, func, theta, visSet, edgeColor, cmap)
            
            % Grab slice at theta.
            [gPlot, dataPlot] = proj(g, func, [0 0 1], theta);
            extraArgs.LineWidth = 2;

            % Visualize final set.
            % NOTE: plot -data because by default contourf plots all values
            % that are ABOVE zero, but inside our obstacle we have values
            % BELOW zero.
            if visSet
                h = visSetIm(gPlot, -dataPlot, edgeColor, 0, extraArgs);
                %h = visSetIm(g, func, edgeColor, 0, extraArgs);
            else
                alpha = 0.5;
                h = visFuncIm(gPlot, dataPlot, edgeColor, alpha); %, edgeColor, 0.5);
                xlabel('V(x)');
            end

            colormap(flipud(cmap));
            xlabel('x');
            ylabel('y');
            %grid off
            set(gca,'TickLength',[0 0]);
        end
        
        %% Plots dubins car point and heading.
        % Inputs:
        %   x [vector]  - 3D state of dubins car
        % Ouput:
        %   c   - handle for figure
        function c = plotCar(obj, x)
            c = plot(x(1), x(2), 'ko','MarkerSize', 5, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k');
            
            % Plot heading.
            center = x(1:2);
            % Rotation matrix.
            R = [cos(x(3)) -sin(x(3)); 
                 sin(x(3)) cos(x(3))];
            % Heading pt.
            hpt = [0.5; 0];
            hptRot = R*hpt + center;
            p2 = plot([center(1) hptRot(1)], [center(2) hptRot(2)], 'LineWidth', 1.5);
            p2.Color(4) = 1.0;
            
            % Setup the figure axes to represent the entire environment
            xlim([obj.lowEnv(1) obj.upEnv(1)]);
            ylim([obj.lowEnv(2) obj.upEnv(2)]);
        end
        
        %% Plots sensing radius centered around car position (x)
        % Inputs:
        %   x [vector]  - 3D state of dubins car
        %   senseRad [float] - sensing radius
        % Ouput:
        %   s   - handle for figure
        function s = plotSensing(obj, x, senseRad, senseShape)
            if strcmp(senseShape,'rectangle')
                lowx = x(1)-senseRad;
                lowy = x(2)-senseRad;
                s = rectangle('Position',[lowx,lowy,senseRad*2,senseRad*2], 'EdgeColor', [0,0,1,0.5], 'LineWidth', 2);
            else
                %s = viscircles([x(1),x(2)],senseRad, 'Color', [0,0.2,1,0.5], 'LineWidth', 2);
                pos = [x(1)-senseRad x(2)-senseRad senseRad*2 senseRad*2];
                s = rectangle('Position', pos,'Curvature',[1,1], 'FaceColor',[0,0.2,1,0.3], 'EdgeColor', [0,0.2,1,0]);
            end
            % Setup the figure axes to represent the entire environment
            xlim([obj.lowEnv(1) obj.upEnv(1)]);
            ylim([obj.lowEnv(2) obj.upEnv(2)]);
        end
        
        %% Plots waypoints.
        function plotWaypts(obj, waypts, simWidth, simHeight)
            X = [];
            Y = [];
            for i=1:length(waypts)
                pt = waypts{i};
                X = [X,pt(1)];
                Y = [Y,pt(2)];
            end
            hold on
            plot(X,Y, 'k', 'LineWidth', 1.5);
            scatter(X,Y,'filled', 'k');
            xlim([0,simWidth]);
            ylim([0,simHeight]);
        end
        
        %% Plots sensed obstacle.
        % Plots all parts that are <= 0.
        function plotObsShape(obj, obsShape)
           [numX, numY, ~] = size(obsShape);
           for i=1:numX
               for j=1:numY
                   if obsShape(i,j,1) <= 0
                       scatter(i,j,'filled', 'k');
                   end
               end
           end
        end
        
        %% Plots how a set becomes a cost function function.
        function plotSetToCostFun(obj, g, func, theta, edgeColor)
             % Grab slice at theta.
            [gPlot, dataPlot] = proj(g, func, [0 0 1], theta);
            extraArgs.LineWidth = 2;

            % Visualize final set.
            % NOTE: plot -data because by default contourf plots all values
            % that are ABOVE zero, but inside our obstacle we have values
            % BELOW zero.
            h = visSetIm(gPlot, -dataPlot, edgeColor, 0, extraArgs);
            zlim([-3,3]);
            
            
            % Set up video
            path = '/home/abajcsy/hybrid_ws/src/planning_sandbox/imgs/lab_mtg_imgs/';
            video_filename = [path datestr(now,'YYYYMMDD_hhmmss') '.avi'];
            vout = VideoWriter(video_filename,'Uncompressed AVI');

            % Open video for writing
            try
                vout.open;
            catch
                error('cannot open file for writing')
            end

            % Grab initial frame, write to video
            set(gcf, 'color', 'w');
            current_frame = getframe(gcf); %gca does just the plot
            writeVideo(vout,current_frame);
            
            % Start from viewing top-down 2D set view.
            az = 0;
            el = 90; 
            view(az, el);
            % we want to get to view(-20, 10)
            while az > -20 || el > 10 
                if az <= -20
                    az = -20;
                else
                    az = az - 2;
                end
                if el <= 10
                    el = 20;
                else
                     el = el - 2;
                end
                view(az, el);
                %pause(0.05);
                current_frame = getframe(gcf); %gca does just the plot
                writeVideo(vout,current_frame);
            end
            
            % After rotating to 3D, plot the corresponding cost function in
            % 3D
            alpha = 0.5;
            visFuncIm(gPlot, dataPlot, edgeColor, alpha);
            zlabel('l(x,y,\theta=\pi/2)');
            current_frame = getframe(gcf); %gca does just the plot
            writeVideo(vout,current_frame);
            
            i = 0;
            while i < 100
                current_frame = getframe(gcf); %gca does just the plot
                writeVideo(vout,current_frame);
                i = i +1;
            end
                
            % Close video
            vout.close
        end
    end
end

