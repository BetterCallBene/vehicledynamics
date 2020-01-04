within VehicleDynamics;

package Interfaces "Vehicle dynamic interfaces"

    partial model Chassis "Interface for Chassis"
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_SW;
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b BQR;
    end Chassis;

    partial model Suspension "Interface for Suspension"
        Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_C  annotation(
      Placement(visible = true, transformation(origin = { 0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = { 0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_L annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_R annotation(
      Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation  
    end Suspension;

    partial model SteerableSuspension "Interface for Suspension wtih steering"
        extends Suspension;
        Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_SW;
    end SteerableSuspension;

    partial model Linkages "Interface for linkages" 
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a  frame_C  annotation(
                Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_U  annotation(
                Placement(visible = true, transformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    end Linkages;
    
end Interfaces;