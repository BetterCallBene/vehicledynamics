within VehicleDynamics;

package UnitTests "Unit tests of VehicleDynamics"
    package WheelAndSuspension "Description"
        model SuspensionTest
    VehicleDynamics.Suspensions.MacPherson macPherson annotation(
        Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world annotation(
        Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VehicleDynamics.Wheels.RillTyre.Wheel wheel_RF(leftWheel = false)  annotation(
        Placement(visible = true, transformation(origin = {58, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VehicleDynamics.Wheels.RillTyre.Wheel wheel_LF annotation(
        Placement(visible = true, transformation(origin = {-68, 2}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 0, 1.0}) annotation(
        Placement(visible = true, transformation(origin = {-4, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(macPherson.frame_Wheel_R, wheel_RF.carrierFrame) annotation(
        Line(points = {{8, 2}, {48, 2}}, color = {95, 95, 95}));
      connect(macPherson.frame_Wheel_L, wheel_LF.carrierFrame) annotation(
        Line(points = {{-12, 2}, {-58, 2}}, color = {95, 95, 95}));
      connect(fixedTranslation.frame_b, macPherson.frame_C) annotation(
        Line(points = {{6, -50}, {-2, -50}, {-2, -10}}));
      connect(world.frame_b, fixedTranslation.frame_a) annotation(
        Line(points = {{-68, -76}, {-14, -76}, {-14, -50}, {-14, -50}}, color = {95, 95, 95}));
    end SuspensionTest;
    end WheelAndSuspension;
end UnitTests;

