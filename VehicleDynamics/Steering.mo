within VehicleDynamics;

package Steering "Steering systems"
  model RackSteering "Standard steering system"
    import SI = Modelica.SIunits;
    parameter SI.Position rRL_LF[3] = {-0.16, 0.54, 0};
    parameter SI.Position rRL_RF[3] = {-0.16, -0.54, 0};
    parameter Real ratioWheelToRack = 22 "rotatational to translational ratio [m/rad]";
    parameter SI.Inertia iSW = 0.2 "Steering inertia";
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(animation = false, r = (rRL_LF + rRL_RF) / 2) annotation(
      Placement(visible = true, transformation(origin = {24, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(animation = true, n = rRL_LF - rRL_RF, useAxisFlange = true) annotation(
      Placement(visible = true, transformation(origin = {-26, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Mechanics.Rotational.Components.IdealGearR2T wheelToRack(ratio = 1 / ratioWheelToRack) annotation(
      Placement(visible = true, transformation(origin = {-18, -48}, extent = {{16, -16}, {-16, 16}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation(
      Placement(visible = true, transformation(origin = {100, -48}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -48}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_C annotation(
      Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_X_LF annotation(
      Placement(visible = true, transformation(origin = {-60, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-60, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_X_RF annotation(
      Placement(visible = true, transformation(origin = {-60, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-60, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J = iSW) annotation(
      Placement(visible = true, transformation(origin = {62, -48}, extent = {{16, -16}, {-16, 16}}, rotation = 0)));
    //   Utilities.Forces.SpringDamperTableRot1D springDamper(data=data_R) annotation(
    //   Placement(visible = true, transformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    //   parameter Utilities.Forces.Utilities.ForceTable1D data_R
    //     annotation (extent=[23, -2; 43, 18]);
    Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c = 100, d = 10000)  annotation(
      Placement(visible = true, transformation(origin = {22, -48}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
  connect(inertia.flange_a, flange_a) annotation(
      Line(points = {{78, -48}, {100, -48}}));
  connect(inertia.flange_b, springDamper.flange_a) annotation(
      Line(points = {{46, -48}, {32, -48}, {32, -48}, {32, -48}}));
  connect(springDamper.flange_b, wheelToRack.flangeR) annotation(
      Line(points = {{12, -48}, {-2, -48}}));
  connect(wheelToRack.flangeT, prismatic.axis) annotation(
      Line(points = {{-34, -48}, {-34, 34}}, color = {0, 127, 0}));
  connect(prismatic.frame_b, frame_X_LF) annotation(
      Line(points = {{-36, 40}, {-60, 40}, {-60, -100}, {-60, -100}}, color = {95, 95, 95}));
  connect(prismatic.frame_b, frame_X_RF) annotation(
      Line(points = {{-36, 40}, {-60, 40}, {-60, 100}, {-60, 100}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_a, frame_C) annotation(
      Line(points = {{34, 40}, {98, 40}, {98, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, prismatic.frame_a) annotation(
      Line(points = {{14, 40}, {-16, 40}, {-16, 40}, {-16, 40}}));
  end RackSteering;
end Steering;