within VehicleDynamics;

package UnitTests "Unit tests of VehicleDynamics"
    package WheelAndSuspension "Description"
    model SuspensionTest "Description"
    VehicleDynamics.Suspensions.ParameterSets.MacPherson macPhersonData annotation(
      Placement(visible = true, transformation(origin = {-76, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VehicleDynamics.Suspensions.Components.MacPherson macPherson_LF(macPhersonData = macPhersonData.macPherson_LF)  annotation(
      Placement(visible = true, transformation(origin = {-48, 20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  VehicleDynamics.Steering.RackSteering rackSteering(rRL_LF = macPhersonData.macPherson_LF.rRL, rRL_RF = macPhersonData.macPherson_RF.rRL, rackSteeringData = macPhersonData.rackSteering)  annotation(
      Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  VehicleDynamics.Suspensions.Components.MacPherson2 macPherson_RF(macPhersonData = macPhersonData.macPherson_RF)  annotation(
      Placement(visible = true, transformation(origin = {52, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
        Placement(visible = true, transformation(origin = {-60, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 0, 1})  annotation(
        Placement(visible = true, transformation(origin = {-12, -58}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VehicleDynamics.Wheels.RillTyre.Wheel wheel_R(leftWheel = false)  annotation(
        Placement(visible = true, transformation(origin = {86, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VehicleDynamics.Wheels.RillTyre.Wheel wheel_L annotation(
        Placement(visible = true, transformation(origin = {-84, 20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
      connect(rackSteering.frame_X_LF, macPherson_LF.frame_S) annotation(
        Line(points = {{-10, 56}, {-48, 56}, {-48, 30}, {-48, 30}}, color = {95, 95, 95}));
      connect(rackSteering.frame_X_RF, macPherson_RF.frame_S) annotation(
        Line(points = {{10, 56}, {52, 56}, {52, 30}, {52, 30}}));
      connect(world.frame_b, fixedTranslation.frame_a) annotation(
        Line(points = {{-50, -58}, {-22, -58}, {-22, -58}, {-22, -58}}, color = {95, 95, 95}));
      connect(fixedTranslation.frame_b, rackSteering.frame_C) annotation(
        Line(points = {{-2, -58}, {0, -58}, {0, 40}, {0, 40}}));
  connect(macPherson_RF.frame_U, wheel_R.carrierFrame) annotation(
        Line(points = {{62, 20}, {76, 20}}));
  connect(macPherson_LF.frame_U, wheel_L.carrierFrame) annotation(
        Line(points = {{-58, 20}, {-74, 20}, {-74, 20}, {-74, 20}}));
  connect(fixedTranslation.frame_b, macPherson_LF.frame_C) annotation(
        Line(points = {{-2, -58}, {-38, -58}, {-38, 20}, {-38, 20}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, macPherson_RF.frame_C) annotation(
        Line(points = {{-2, -58}, {42, -58}, {42, 20}, {42, 20}}, color = {95, 95, 95}));
    end SuspensionTest;
    end WheelAndSuspension;
end UnitTests;