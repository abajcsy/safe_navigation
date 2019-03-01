function alpha = plane4Dpartial_localQ(t, data, derivMin, derivMax, schemeData, dim, Q)
% hamValue = plane4Dham(t, data, deriv, schemeData)
%   Hamiltonian function for a 4D plane (Dubins car with finite acceleration)
%
% Inputs:
%   schemeData - problem parameters
%     .grid:   grid structure
%     .wMax:   max turn rate
%     .arange: acceleration range
%
% Dynamics:
%   \dot x      = v \cos \theta
%   \dot y      = v \sin \theta
%   \dot \theta = w
%   \dot v      = a
%     |w| <= wMax
%     arange(1) <= a <= arange(2)
%

checkStructureFields(schemeData, 'grid',  'dynSys')

g = schemeData.grid;
wMax = schemeData.dynSys.wMax;
arange = schemeData.dynSys.aRange;

switch dim
  case 1
    alpha = abs(g.xs{4}(Q).*cos(g.xs{3}(Q)));
    
  case 2
    alpha = abs(g.xs{4}(Q).*sin(g.xs{3}(Q)));
    
  case 3
    alpha = wMax;
    
  case 4
    alpha = max(abs(arange));
    
  otherwise
    error([ 'Partials only exist in dimensions 1-4' ]);
end
end