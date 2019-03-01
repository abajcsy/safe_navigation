function hamValue = plane4Dham_localQ(t, data, deriv, schemeData, Q)
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

v_Q = g.xs{4}(Q);
theta_Q = g.xs{3}(Q);

hamValue = deriv{1} .* (v_Q.*cos(theta_Q)) + ...
  deriv{2} .* (v_Q.*sin(theta_Q)) + ...
  -wMax * abs(deriv{3}) + ...
  (deriv{4}>=0).*deriv{4}*arange(1) + (deriv{4}<0).*deriv{4}*arange(2);

hamValue = -hamValue;
end