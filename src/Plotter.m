classdef Plotter < handle
    %PLOTTER Plots level sets and environment and stuff.
    
    properties
        lowEnv      % lower (x,y) corner of environment
        upEnv       % upper (x,y) corner of environment
        lowObs      % if rectangle: lower left (x,y) point
                    % if circle: center (x,y) of circle
        upObs       % if rectangle: upper right (x,y) point
                    % if circle: radius of circle 
        obsShape    % shape of the obstacle (rectangle or circle)
        
        % figure handles
        figh
        envh
        senseh
        carh
        vxh
        firstPlot 
    end
    
    methods
        %% Constructor.
        function obj = Plotter(lowEnv, upEnv, lowObs, upObs, obsShape)
            obj.lowEnv = lowEnv;
            obj.upEnv = upEnv;
            obj.lowObs = lowObs;
            obj.upObs = upObs;
            obj.obsShape = obsShape;
            obj.envh = NaN;
            obj.senseh = NaN;
            obj.carh = NaN;
            obj.vxh = NaN;
            obj.firstPlot = true;
        end
        
        %% Updates the plot (environment, sensing, set, car)
        %  Input:
        %       x       -- state of dyn system
        %       setObj  -- avoid set object
        function updatePlot(obj, x, xgoal, setObj)
            % Delete old plots
            if ~obj.firstPlot
                delete(obj.envh);
%                 delete(obj.senseh);
                delete(obj.carh);
%                 delete(obj.vxh);
            else
                obj.figh = figure(1);
                obj.firstPlot = false;
                c = [0.1,0.8,0.5];
                scatter(xgoal(1),xgoal(2),[],c,'filled');
            end
            
            % Visualize environment and sensing and car
            obj.envh = obj.plotEnvironment();
%             obj.senseh = obj.plotSensing(setObj.gFMM, setObj.unionL_2D_FMM);
            obj.carh = obj.plotCar(x);
            obj.plotBoundaryPadding(setObj.boundLow, setObj.boundUp);

            % Plot value function
            extraArgs.edgeColor = [1,0,0];

%             if length(x) == 3
%                 extraArgs.theta = x(3);
%                 funcToPlot = setObj.valueFun(:,:,:,end);
%             else
%                 funcToPlot = setObj.valueFun(:,:,end);
%             end
% 
%             visSet = true;
%             obj.vxh = obj.plotFuncLevelSet(setObj.grid, funcToPlot, visSet, extraArgs);
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
            
            xlabel('$p_x$', 'Interpreter','latex');
            ylabel('$p_y$', 'Interpreter','latex');
            set(gca,'TickLength',[0 0]);
            box on
        end
        
        %% Plot level set of arbitrary function
        % Inputs:
        %   g [array]     - grid corresponding to data
        %   func [array]  - data for function to visualize
        %   visSet [bool] - if true, plots 2D slice of func.
        %                   Otherwise plots 3D.
        %   extraArgs
        %           .theta [float] - (if 3D system) angle for which to plot the level set
        %           .edgeColor [vector or string] - color of level set boundary
        %           .cmap [string] - name of colormap to use
        % Outputs: 
        %   plots level set in (x,y) for fixed theta.
        function h = plotFuncLevelSet(obj, g, func, visSet, extraArgs)
            
            if isfield(extraArgs, 'theta')
                % Grab slice at theta.
                [gPlot, dataPlot] = proj(g, func, [0 0 1], extraArgs.theta);
            else
                gPlot = g;
                dataPlot = func;
            end
            
            % grab the edge color
            if isfield(extraArgs, 'edgeColor')
                edgeColor = extraArgs.edgeColor;
            else
                edgeColor = [1,0,0];
            end
            
            % grab the color map
            if isfield(extraArgs, 'cmap')
                cmap = extraArgs.cmap;
            else
                cmap = 'hot';
            end
            
            % Visualize final set.
            % NOTE: plot -data if using contourf plots because it uses all values
            % that are ABOVE zero, but inside our obstacle we have values
            % BELOW zero.
            if visSet
                h = visSetIm(gPlot, dataPlot, edgeColor, 0);
            else
                alpha = 0.5;
                h = visFuncIm(gPlot, dataPlot, edgeColor, alpha); %, edgeColor, 0.5);
                xlabel('V(x)');
            end

            colormap(flipud(cmap));
            xlabel('$p_x$', 'Interpreter','latex');
            ylabel('$p_y$', 'Interpreter','latex');
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
            
            if length(x) == 3
                % Rotation matrix.
                R = [cos(x(3)) -sin(x(3)); 
                     sin(x(3)) cos(x(3))];
                % Heading pt.
                hpt = [0.5; 0];
                hptRot = R*hpt + center;
                p2 = plot([center(1) hptRot(1)], [center(2) hptRot(2)], 'k', 'LineWidth', 1.5);
                p2.Color(4) = 1.0;
            end
            
            % Setup the figure axes to represent the entire environment
            xlim([obj.lowEnv(1) obj.upEnv(1)]);
            ylim([obj.lowEnv(2) obj.upEnv(2)]);
        end
        
        %% Plots sensing radius centered around car position (x)
        % Inputs:
        %   grid  - 
        %   signed_distance_map - 
        % Ouput:
        %   h   - handle for figure
        function h = plotSensing(obj, grid, signed_distance_map)
            s = contourf(grid.xs{1}, grid.xs{2}, signed_distance_map, [0, 0], 'FaceColor',[1,1,1], 'EdgeColor', [1,1,1]);
            h = fill(s(1,2:end),s(2,2:end),'m','FaceColor',[0,0.2,1], 'FaceAlpha',0.3, 'EdgeColor', [1,1,1]);
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
        
        %% Plots the 1-grid-cell-sized padding around the compute grid.
        %  (used for sanity checking)
        function plotBoundaryPadding(obj, boundLow, boundUp)
            width = boundUp(1) - boundLow(1);
            height = boundUp(2) - boundLow(2);
            boundCoord = [boundLow(1), boundLow(2), width, height];
            rectangle('Position', boundCoord, 'Linewidth', 1.0, 'LineStyle', ':'); 
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
            repo = what('safe_navigation');
            path = strcat(repo.path, '/imgs/lab_mtg_imgs/');
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

