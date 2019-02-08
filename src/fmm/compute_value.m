function u = compute_value(g, target, speed, obs, dom_map)
% u = compute_value(g, target, speed, obs, dom_map)
%
% Computes value function from target to every point on the domain defined
% by dom_map
%
% Currently, domain must be [-L, L]^2 (No immediate plans to change this)
%
% INPUTS
%   g:          grid structure
%   target:   target set (u = time to reach this point)
%   speed:    speed profile (N by N)
%   obs:      obstacle map (N by N, negative value means obstacle)
%   dom_map:  domain map (N by N)
%
% OUTPUT
%   u:        time to reach value function (N by N)
%
% Mo Chen, 2013-06-07

% tic;

checkGrid(g); % Check grid compatibility

N = g.N(1);
L = g.max(1);
infty = 1e6;

if nargin<4, obs = ones(N); end % Default obstacle is no obstacles
if isempty(obs), obs = ones(N); end % An empty obstacle also means no obstacles
if nargin<5, dom_map = -ones(N); end % Default domain is all of the space defined by the grid

% Initialize value variable and PDE boundary condition
u = zeros(N); 
bdryCond = -ones(N);

if numel(target) == 2           % If target is a single point, then set target to nearest grid point
    [xi0, yi0] = xy2inds(target(1),target(2),g);
    bdryCond(xi0,yi0) = 0;
elseif size(target) == [N N]    % If target is specified in an N by N matrix (negative means inside)
    bdryCond(target<0) = 0;
end

% If speed profile is a scalar, then change it to match size of grid
if numel(speed) == 1, speed = speed*ones(N); end

% If size of speed profile does not match that of grid, resample
if ~(size(speed) == [N N])
    speedvx = linspace(1,size(speed,1),N);
    speedvy = linspace(1,size(speed,2),N);
    [speedVx, speedVy] = ndgrid(speedvx, speedvy);
    speed = interpn(speed, speedVx, speedVy,'nearest');
end

% Set points inside obstacles and outside domain to have infinite value and
% 0 speed
bdryCond(obs<0) = infty;
bdryCond(dom_map>0) = infty;

speed(obs<0) = 0;
speed(dom_map>0) = 0;

% --- Run fast marching method ---
% Get path of current file
funDir = mfilename('fullpath');

% Compile mex file
mex([funDir(1:end-13) 'cversion/mexEikonalFMM.cpp']);


% Run FMM code
mexEikonalFMM(u,bdryCond,speed,L);

u(u>0.95*infty) = nan;

end