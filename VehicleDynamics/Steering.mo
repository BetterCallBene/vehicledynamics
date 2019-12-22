within VehicleDynamics;

package Steering "Steering systems"
  model RackSteering "Standard steering system"
    import SI = Modelica.SIunits;
    parameter SI.Position rRL_1[3] = {-0.16, 0.54, 0};
    parameter SI.Position rRL_2[3] = {-0.16, -0.54, 0};
    parameter Real ratioWheelToRack = 22 "rotatational to translational ratio [m/rad]";
    parameter SI.Inertia iSW = 0.2 "Steering inertia";
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(animation = false, r = (rRL_1 + rRL_2) / 2) annotation(
      Placement(visible = true, transformation(origin = {24, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(animation = true, n = rRL_1 - rRL_2, useAxisFlange = true) annotation(
      Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.Rotational.Components.IdealGearR2T wheelToRack(ratio = 1 / ratioWheelToRack) annotation(
      Placement(visible = true, transformation(origin = {-18, -48}, extent = {{16, -16}, {-16, 16}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_a annotation(
      Placement(visible = true, transformation(origin = {100, -48}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, -48}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_C annotation(
      Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_X_1 annotation(
      Placement(visible = true, transformation(origin = {-60, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-60, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_X_2 annotation(
      Placement(visible = true, transformation(origin = {-60, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-60, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.Rotational.Components.Inertia inertia(J = iSW) annotation(
      Placement(visible = true, transformation(origin = {62, -48}, extent = {{16, -16}, {-16, 16}}, rotation = 0)));
    //   Utilities.Forces.SpringDamperTableRot1D springDamper(data=data_R) annotation(
    //   Placement(visible = true, transformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    //   parameter Utilities.Forces.Utilities.ForceTable1D data_R
    //     annotation (extent=[23, -2; 43, 18]);
    Modelica.Mechanics.Rotational.Components.SpringDamper springDamper(c = 1, d = 1)  annotation(
      Placement(visible = true, transformation(origin = {22, -48}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    connect(frame_C, fixedTranslation.frame_b) annotation(
      Line(points = {{100, 0}, {34, 0}, {34, 42}, {34, 42}}));
    connect(prismatic.frame_a, fixedTranslation.frame_a) annotation(
      Line(points = {{-40, 10}, {-40, 42}, {14, 42}}, color = {95, 95, 95}));
    connect(prismatic.frame_a, frame_X_2) annotation(
      Line(points = {{-40, 10}, {-40, 10}, {-40, 100}, {-60, 100}}, color = {95, 95, 95}));
    connect(prismatic.frame_a, frame_X_1) annotation(
      Line(points = {{-40, 10}, {-60, 10}, {-60, -100}, {-60, -100}}, color = {95, 95, 95}));
  connect(inertia.flange_a, flange_a) annotation(
      Line(points = {{78, -48}, {100, -48}}));
  connect(inertia.flange_b, springDamper.flange_a) annotation(
      Line(points = {{46, -48}, {32, -48}, {32, -48}, {32, -48}}));
  connect(springDamper.flange_b, wheelToRack.flangeR) annotation(
      Line(points = {{12, -48}, {-2, -48}}));
  connect(wheelToRack.flangeT, prismatic.axis) annotation(
      Line(points = {{-34, -48}, {-34, -8}}, color = {0, 127, 0}));
  end RackSteering;
end Steering;