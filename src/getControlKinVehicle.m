% Returns control to apply to kinematic car at a particular time.
% u = [vx, vy];
function u = getControlKinVehicle(t, drift)
    if t > 0 && t < 40
        u = [-drift,1.0];
    elseif t >= 40 && t < 150
        u = [0,0];
    else
        u = [-drift,-1.0];
    end
    
end
