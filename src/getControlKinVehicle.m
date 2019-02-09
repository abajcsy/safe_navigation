% Returns control to apply to kinematic car at a particular time.
% u = [vx, vy];
function u = getControlKinVehicle(t, drift)
    if t > 0 && t < 60
        u = [-drift,1.0];
    else %if t >= 90 
        u = [0,0];
    end
    
end
