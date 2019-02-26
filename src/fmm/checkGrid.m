function checkGrid(g)
% checkGrid(g)
%
% Checks the grid structure to make sure it's compatible with 2D FMM
% calculations, which must be done on a [-L,L]^2 domain
%
% Input: g - grid structure
%
% An error will occur if grid structure is not compatible
% 
% Mo Chen, Sept. 26, 2013
if g.dim ~= 2, error('Dimension of the problem must be 2!'); end

for i = 1:g.dim
    if g.max(i) + g.min(i), error('Domain must be [-L, L]^2!'); end
end

if g.N(2) - g.N(1), error('# of grid points must be the same in each dimension!'); end
end