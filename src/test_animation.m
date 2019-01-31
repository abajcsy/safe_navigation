clf
clc
clear all

% Setup environment bounds.
lowEnv = [0;0];
upEnv = [10;7];

% Grab the ground truth value functions over time
repo = what('safe_navigation');
valFunPath = strcat(repo.path, '/data/groundTruthValueFuns.mat');
load(valFunPath, 'valueFunCellArr');
valueFunCellArr = valueFunCellArr;

% Setup lower and upper computation domains and discretization.
gridLow = [lowEnv;-pi];
gridUp = [upEnv;pi];
N = [31;31;21];
pdDims = 3;     
grid = createGrid(gridLow, gridUp, N, pdDims);

% Create lower and upper bounds on rectangle.
obsShape = 'rectangle';
lowRealObs = [4;1];
upRealObs = [7;4];

% Get the plotter
plt = Plotter(lowEnv, upEnv, lowRealObs, upRealObs, obsShape);

% Get the dynamical system
x_init = [2.0; 2.5; pi/2];
wMax = 1;
vrange = [0.5,1];

% Define dynamic system.            
dynSys = Plane(x_init, wMax, vrange);
x = dynSys.x;
dt = 0.05;

% Sensing data
senseShape = 'circle';
senseRad = 1.5;

visSet = true;
cmapHot = 'hot';
hold on;
view(-15,32);

% gif saving info
axis tight manual
filename = 'testAnimated.gif';
set(gcf, 'color', 'w');

% set z info
zlim([-3,3]);


% Put grid and dynamic systems into schemeData.
schemeData.grid = grid;
schemeData.dynSys = dynSys;
schemeData.accuracy = 'high'; % Set accuracy.
schemeData.uMode = 'max';
schemeData.dMode = 'min';

% Convergence information
HJIextraArgs.visualize.valueSet = 1;
HJIextraArgs.stopConverge = 0;
HJIextraArgs.ignoreBoundary = 1; 
timeDisc = 0:dt:100; 
minWith = 'minVWithL';

for i=1:length(valueFunCellArr)
    valueFun = valueFunCellArr{i};
    
    u = getControl(i);
    dynSys.updateState(u, dt, dynSys.x);
    x = dynSys.x;
    
    % Visualize standard value funs.
   % zlabel('V(z)');
    %valueFunc = plt.plotFuncLevelSet(grid, valueFun(:,:,:,end), x(3), false, [1,0,0], cmapHot);
    valueSet = plt.plotFuncLevelSet(grid, valueFun(:,:,:,end), x(3), true, [1,0,0], cmapHot);
    
    % Visualize cost fun.
    %zlabel('l(z)');
    %costFunc = plt.plotFuncLevelSet(grid, valueFun(:,:,:,1), x(3), false, [0.6,0.6,0.7], cmapHot);
    
    % Plot environment, car, and sensing.
    %envHandle = plt.plotEnvironment();
    senseVis = plt.plotSensing(x, senseRad, senseShape);
    carVis = plt.plotCar(x);

    % Visualize computation process.
    %[dataOut, tau, extraOuts] = ...
    %    HJIPDE_solve(valueFun(:,:,:,1), timeDisc, schemeData, minWith, HJIextraArgs);
            
    %drawnow
    
    % Capture the plot as an image 
    frame = getframe(1); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    % Write to the GIF File 
    if i == 1 
        imwrite(imind,cm,filename,'gif', 'DelayTime',0.1, 'Loopcount',inf); 
    else 
        imwrite(imind,cm,filename,'gif', 'DelayTime',0.1, 'WriteMode','append'); 
    end 
    
    %pause(0.05);
    
    if i~= length(valueFunCellArr)
        delete(valueFunc);
        delete(valueSet);
        delete(senseVis);
        delete(carVis);
    end
end