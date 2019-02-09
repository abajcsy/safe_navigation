classdef KinVehicle2D < DynSys
  properties
    drift
    vMax
  end
  
  methods
    function obj = KinVehicle2D(x, vMax, drift)
      % obj = KinVehicleND(x, vMax)
      % 
      % This vehicle has drift in the x-direction. 
      % Dynamics: (2D example)
      %    \dot{x}_1 = v_x + drift 
      %    \dot{x}_2 = v_y
      %         v_x^2 + v_y^2 <= vMax^2
      
      %% State could be of any number of dimensions
      if ~iscolumn(x)
        x = x';
      end
      
      obj.nx = length(x);
      obj.nu = obj.nx;
      
      %% Velocity
      if nargin < 2
        vMax = 1;
      end
      
      obj.x = x;
      obj.xhist = obj.x;
         
      obj.drift = drift;
      obj.vMax = vMax;
    end
    
  end % end methods
end % end classdef
