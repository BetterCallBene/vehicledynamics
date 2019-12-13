within VehicleDynamics;
package Chassis "Description"
    package Interfaces "Description"
        model Linkages "Interface for linkages" 
            Modelica.Mechanics.MultiBody.Interfaces.Frame_a  frame_C  annotation(
                Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
            Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_U  annotation(
                Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        end Linkages;
    end Interfaces;
end Chassis;