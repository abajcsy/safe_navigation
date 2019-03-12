function hamValue = dubins3Dham_localQ(t, data, deriv, schemeData, Q)
% hamValue = dubins3Dham(t, data, deriv, schemeData)
%   Hamiltonian function for Dubins car used with the level set toolbox
%
% Inputs:
%   schemeData - problem parameters
%     .grid:   grid structure
%     .vrange: speed range of vehicle
%     .wMax:   turn rate bounds (w \in [-wMax, wMax])
%     .uMode:  'min' or 'max' (defaults to 'min')
%     .dMax:   disturbance bounds (see below)
%     .dMode: 'min' or 'max' (defaults to 'max')
%     .tMode: 'backward' or 'forward'
%
% Dynamics:
%   \dot x      = v * cos(\theta) + d1
%   \dot y      = v * sin(\theta) + d2
%   \dot \theta = u + d3
% (d1, d2) \in disk of radius dMax(1)
% d3 \in [-dMax(2), dMax(2)]
%
% Mo Chen, 2016-05-21

checkStructureFields(schemeData, 'grid');

g = schemeData.grid;

%% Speed range
vrange = schemeData.dynSys.vrange;
if isscalar(vrange)
  vrange = [vrange vrange];
end
vMin = vrange(1);
vMax = vrange(2);

%% Defaults: min over control, max over disturbance, backward reachable set
if ~isfield(schemeData, 'uMode')
  schemeData.uMode = 'min';
end

if ~isfield(schemeData, 'dMode')
  schemeData.dMode = 'max';
end

if ~isfield(schemeData, 'tMode')
  schemeData.tMode = 'backward';
end

% Extract the derivs and grid for Q only
theta_Q = g.xs{3}(Q);
deriv1_Q = deriv{1};
deriv2_Q = deriv{2};
deriv3_Q = deriv{3};

%% Modify Hamiltonian control terms based on uMode
if strcmp(schemeData.uMode, 'min')
  % the speed when the determinant term (terms multiplying v) is positive
  % or negative
  v_when_det_pos = vMin;
  v_when_det_neg = vMax;
  % turn rate term
  wTerm = -schemeData.dynSys.wMax*abs(deriv3_Q);
elseif strcmp(schemeData.uMode, 'max')
  v_when_det_pos = vMax;
  v_when_det_neg = vMin;
  wTerm = schemeData.dynSys.wMax*abs(deriv3_Q);
else
  error('Unknown uMode! Must be ''min'' or ''max''')
end

%% Modify Hamiltonian control terms based on dMode
if strcmp(schemeData.dMode, 'min')
  dMax_x = schemeData.dynSys.dMax(1);
  dMax_y = schemeData.dynSys.dMax(2);
  dTerm = -dMax_x * abs(deriv1_Q) -dMax_y * abs(deriv2_Q);
else
  error('Unknown uMode! Must be ''min'' or ''max''')
end


%% Hamiltonian control terms
% Speed control
hamValue = (deriv1_Q.*cos(theta_Q) + deriv2_Q.*sin(theta_Q) >= 0) .* ...
  (deriv1_Q.*cos(theta_Q) + deriv2_Q.*sin(theta_Q)) * v_when_det_pos + ...
  (deriv1_Q.*cos(theta_Q) + deriv2_Q.*sin(theta_Q) < 0) .* ...
  (deriv1_Q.*cos(theta_Q) + deriv2_Q.*sin(theta_Q)) * v_when_det_neg;

% turn rate control
hamValue = hamValue + wTerm;

% Add the effect of disturbance
hamValue = hamValue + dTerm;

%% Backward or forward reachable set
if strcmp(schemeData.tMode, 'backward')
  hamValue = -hamValue;
elseif ~strcmp(schemeData.tMode, 'forward')
  error('tMode must be ''forward'' or ''backward''!')
end
end
