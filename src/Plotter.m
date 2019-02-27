classdef Plotter < handle
    %PLOTTER Plots level sets and environment and stuff.
    
    properties
        lowEnv      % (arr) lower (x,y) corner of environment
        upEnv       % (arr) upper (x,y) corner of environment
        obstacles   % (cell arr) lower & upper corners of obstacles
        
        % figure handles
        figh
        envh
        senseh
        carh
        vxh
        trajh
        firstPlot   % (bool) if this is the first time we plotted
        
        % occupancy map fig handles
        safemaph
        planmaph
        
        boundLow
        boundUp
    end
    
    methods
        %% Constructor.
        function obj = Plotter(lowEnv, upEnv, boundLow, boundUp, obstacles)
            obj.lowEnv = lowEnv;
            obj.upEnv = upEnv;
            obj.obstacles = obstacles;
            obj.envh = NaN;
            obj.senseh = NaN;
            obj.carh = [];
            obj.vxh = NaN;
            obj.trajh = [];
            obj.firstPlot = true;
            obj.safemaph = [];
            obj.planmaph = [];
            obj.boundLow = boundLow;
            obj.boundUp = boundUp;
        end
        
        %% Updates the plot (environment, sensing, set, car)
        %  Input:
        %       x       -- state of dyn system
        %       xgoal   -- goal state 
        %       setObj  -- avoid set object
        function updatePlot(obj, x, xgoal, valueFun, g, gMap, ...
                occuMap, path, usedUOpt)
            % Delete old plots
            if ~obj.firstPlot
                delete(obj.senseh);
                %delete(obj.carh{1});
                delete(obj.carh{2});
                delete(obj.vxh);
            else
                obj.figh = figure(1);
                obj.firstPlot = false;
                c = [0.1,0.8,0.5];
                scatter(xgoal(1),xgoal(2),[],c,'filled');
            end
            %figure(obj.figh);
            
            % Plot value function
            extraArgs.edgeColor = [1,0,0];

            if length(x) == 3
                extraArgs.theta = x(3);
                funcToPlot = valueFun(:,:,:,end);
            else
                funcToPlot = valueFun(:,:,end);
            end

            visSet = true;
            obj.vxh = obj.plotFuncLevelSet(g, funcToPlot, visSet, extraArgs);
            %[gPlot, dataPlot] = proj(g, funcToPlot, [0 0 1], extraArgs.theta);
            %obj.vxh = visSetIm(gPlot, dataPlot, extraArgs.edgeColor , 0);
            
            
            % Visualize environment 
            obj.envh = obj.plotEnvironment();
                
            % Note: we just grab a slice of signed_dist at any theta
            obj.senseh = obj.plotSensing(gMap, occuMap);
            obj.carh = obj.plotCar(x, usedUOpt);
            obj.plotBoundaryPadding(obj.boundLow, obj.boundUp);
            obj.plotTraj(path);
        end
        
        %% Plots the occupancy map as understood by the safety module.
        function updateOccuMapSafe(obj, g, safeMap)
            if isempty(obj.safemaph)
                obj.safemaph = figure(2);
            end
            figure(obj.safemaph);
            % clear figure
            clf(obj.safemaph);
            colormap('gray');
            contourf(g.xs{1}, g.xs{2}, safeMap, -1:2:1);
            xlim([obj.lowEnv(1) obj.upEnv(1)]);
            ylim([obj.lowEnv(2) obj.upEnv(2)]);
        end
        
        %% Plots the occupancy map as understood by the planner.
        function updateOccuMapPlan(obj, g, planMap)
            if isempty(obj.planmaph)
                obj.planmaph = figure(3);
            end
            figure(obj.planmaph);
            % clear figure
            clf(obj.planmaph);
            contourf(g.xs{1}, g.xs{2}, planMap, -1:2:1);
            colormap('gray');
            xlim([obj.lowEnv(1) obj.upEnv(1)]);
            ylim([obj.lowEnv(2) obj.upEnv(2)]);
        end
        
        
        %% Plots the environment with the obstacle.
        % Output: 
        %   e - figure handle
        function e = plotEnvironment(obj)
            
            for i=1:length(obj.obstacles)
                lowObs = obj.obstacles{i}{1};
                upObs = obj.obstacles{i}{2};
                width = upObs(1) - lowObs(1);
                height = upObs(2) - lowObs(2);
                obsCoord = [lowObs(1), lowObs(2), width, height];
                %e = rectangle('Position', obsCoord, 'Linewidth', 2.0, 'LineStyle', '--'); 
                e = rectangle('Position', obsCoord, ...
                    'FaceColor', [0.9,0.9,0.9], 'EdgeColor', [0.5,0.5,0.5]); 
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
                %[~, h] = contourf(gPlot.xs{1}, gPlot.xs{2}, dataPlot, 0:0.1:5);
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
        function c = plotCar(obj, x, usedUOpt)
            % color car red if we applied safe control
            if usedUOpt
                carColor = 'r';
            else
                carColor = 'k';
            end
            c = {};
            if ~isempty(obj.carh)
                if isequal(obj.carh{1}.MarkerFaceColor, [0, 0, 0]) 
                    set(obj.carh{1}, 'Color', [0.7,0.7,0.7]);
                    set(obj.carh{1}, 'MarkerFaceColor', [0.7,0.7,0.7]);
                    set(obj.carh{1}, 'MarkerEdgeColor', [0.7,0.7,0.7]);
                else
                    set(obj.carh{1}, 'Color', [1.0,0.5,0.5]);
                    set(obj.carh{1}, 'MarkerFaceColor', [1.0,0.5,0.5]);
                    set(obj.carh{1}, 'MarkerEdgeColor', [1.0,0.5,0.5]);
                end
            end
            c{1} = plot(x(1), x(2), 'ko','MarkerSize', 5, ...
                'MarkerEdgeColor', carColor, 'MarkerFaceColor', carColor);
            
            % Plot heading.
            center = x(1:2);
            
            if length(x) == 3
                % Rotation matrix.
                R = [cos(x(3)) -sin(x(3)); 
                     sin(x(3)) cos(x(3))];
                % Heading pt.
                hpt = [0.5; 0];
                hptRot = R*hpt + center;
                p2 = plot([center(1) hptRot(1)], [center(2) hptRot(2)], carColor, 'LineWidth', 1.5);
                p2.Color(4) = 1.0;
                c{2} = p2;
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
            posIdx = find(signed_distance_map > 0);
            h = scatter(grid.xs{1}(posIdx),grid.xs{2}(posIdx), 30, ...
                'MarkerFaceColor', [0,0.2,1], 'MarkerFaceAlpha', 0.3, 'MarkerEdgeColor', 'none');
            
            %s = contourf(grid.xs{1}, grid.xs{2}, signed_distance_map, [0, 0], 'FaceColor',[1,1,1], 'EdgeColor', [1,1,1]);
            %h = fill(s(1,2:end),s(2,2:end),'m','FaceColor',[0,0.2,1], 'FaceAlpha',0.3, 'EdgeColor', [1,1,1]);
            % Setup the figure axes to represent the entire environment
            xlim([obj.lowEnv(1) obj.upEnv(1)]);
            ylim([obj.lowEnv(2) obj.upEnv(2)]);
        end
        
        %% Plots optimal path
        function plotTraj(obj, path)
            if ~isempty(obj.trajh)
                for i=1:length(obj.trajh)
                    delete(obj.trajh(i));
                end
            end
            obj.trajh = [];
            i = 1;
            xvals = [];
            yvals = [];
            while i < length(path)
                x = path{i};
                xvals = [xvals, x(1)];
                yvals = [yvals, x(2)];
                xnext = path{i+1};
                % plot the line segments.
                h = line([x(1), xnext(1)], [x(2), xnext(2)], 'Color', 'blue', 'LineWidth', 1.5);
                obj.trajh = [obj.trajh, h];
                i = i+1;
            end
            
            if length(path) ~= 0
              % get final waypt.
              x = path{i};
              xvals = [xvals, x(1)];
              yvals = [yvals, x(2)];
              % plot the waypts.
              s = scatter(xvals, yvals, 15, 'b', 'filled');
              obj.trajh = [obj.trajh, s];
            end
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
            rectangle('Position', boundCoord, 'EdgeColor', [0.5,0.5,0.5], ...
                'LineWidth', 1.0, 'LineStyle', ':'); 
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

