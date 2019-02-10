function dx = dynamics(obj, t, x, u, ~, ~)
% Dynamics of the 2D kinematic vehicle
%    \dot{x}_1 = v_x
%    \dot{x}_2 = v_y
%        u = (v_x, v_y)


if iscell(x)
    dx = {u{1}+obj.drift, u{2}};
else
    dx = [u(1)+obj.drift; u(2)];
end

end