function uOpt = optCtrl(obj, ~, ~, deriv, uMode, ~)
% uOpt = optCtrl(obj, t, y, deriv, uMode)
%     Dynamics of the Plane4D
%         \dot{x}_1 = x_4 * cos(x_3) + d_1
%         \dot{x}_2 = x_4 * sin(x_3) + d_2
%         \dot{x}_3 = u_1 = u_1
%         \dot{x}_4 = u_2 = u_2

%% Input processing
if nargin < 5
  uMode = 'min';
end

if ~iscell(deriv)
  deriv = num2cell(deriv);
end

uOpt = cell(obj.nu, 1);

%% Optimal control
if strcmp(uMode, 'max')
  if any(obj.dims == 3)
    uOpt{1} = (deriv{obj.dims==3}>=0)*obj.wMax + ...
      (deriv{obj.dims==3}<0)*(-obj.wMax);
  end
  
  if any(obj.dims == 4)
    if obj.x(4) >= obj.vRange(2)
      uRange = [obj.aRange(1), 0.0];
    elseif obj.x(4) <= obj.vRange(1)
      uRange = [0.0, obj.aRange(2)];
    else
      uRange = obj.aRange;
    end
    uOpt{2} = (deriv{obj.dims==4}>=0)*uRange(2) + ...
      (deriv{obj.dims==4}<0)*uRange(1);
  end

else
  error('Unknown uMode!')
end

end