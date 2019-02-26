% Returns control to apply to dubins car at a particular time.
% u = [v, omega]
function u = getHandCodedControl(t)
    if t >= 1 && t < 35
        u = [1.0,-1.0];
    elseif t >= 35 && t < 40
        u = [1.0,0.0];
    else
        u = [1.0,0.0];
    end
    if t >= 1 && t < 20
        u = [1.0, 0.0];
    elseif t >= 20 && t < 40
        u = [1.0,-1.0];
    elseif t >= 40 && t < 50
        u = [1.0,1.0];
    elseif t >=50 && t < 70
        u = [1.0,-1.0];
    elseif t >= 70 && t < 120
        u = [1.0,0.0];
    elseif t >= 120 && t < 150
        u = [1.0,-1.0];
    else
        u = [1.0,0.0];
    end
end
