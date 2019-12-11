within VehicleDynamics;

package Deprecated "Deprecated functions from previous VehicleDynamics with should be replace by Modelica Standard Library"
    
    function interpolate1 "Interpolate linearly between two points"
        input Real x "abszissa value to be interpolated";
        input Real x1 "abszissa value of point 1";
        input Real y1 "ordinate value of point 1";
        input Real x2 "abszissa value of point 2";
        input Real y2 "ordinate value of point 2";
        output Real y "ordinate value of x, i.e., y(x)";
      algorithm
        y := y1 + (y2 - y1) * ((x - x1) / (x2 - x1));
    end interpolate1;

    function interpolate2 "Interpolate quadratically between two points, such that y(0)=0"
        input Real x "abszissa value to be interpolated";
        input Real x1 "abszissa value of point 1 (<> 0 required)";
        input Real y1 "ordinate value of point 1";
        input Real x2 "abszissa value of point 2 (x2 <> x1 required)";
        input Real y2 "ordinate value of point 2";
        output Real y "ordinate value of x, i.e., y(x)";
      algorithm
        y := ((y2 * x1 / x2 - y1 * x2 / x1) * (x1 - x) / (x1 - x2) + y1 * x / x1) * x / x1;
    end interpolate2;
    
end Deprecated;