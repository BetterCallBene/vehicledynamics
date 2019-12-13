within VehicleDynamics;

package Suspensions "Suspensions, models ready to be used as front or rear suspensions."
      model MacPherson "MacPherson strut linkage"
        import SI = Modelica.SIunits;
        extends Chassis.Interfaces.Linkages;

        parameter SI.Position[3] rCL1 = {0.1070, 0.55, -0.0380} "|Geometry| Vector from origin of frame_C to front link mount in chassis resolved in frame_C (at initial time)";
        parameter SI.Position[3] rCL2 = {-0.3070, 0.55, -0.0380} "|Geometry| Vector from origin of frame_C to rear link mount in chassis resolved in frame_C (at initial time)";
        parameter SI.Position[3] rCS = {-0.0295, 0.850, 0.5670} "|Geometry| Vector from origin of frame_C to strut mount in chassis resolved in frame_C (at initial time)";
        parameter SI.Position[3] rUS = {-0.0070, 0.8733, 0.1380} "|Geometry| Vector from origin of frame_C to strut mount in uppright resolved in frame_C (at initial time)";
        parameter SI.Position[3] rUL1L2 = {-0.0070, 0.8733, -0.0380} "|Geometry| Vector from origin of frame_C to low spindleUtilities.Joints.Joint resolved in frame_C (at initial time)";
        parameter SI.Position[3] rUW = {0, 0.9, 0} "|Geometry| Vector from origin of frame_C to wheel centre resolved in frame_C (at initial time)";
        parameter SI.Position[3] rRL3 = {-0.12, 0.85, 0.08} "|Geometry| Vector from origin of frame_C to origin of frame_S resolved in frame_C (at initial time)";
        parameter SI.Position[3] rUL3 = {0, 0.9, 0} "|Geometry| Vector from origin of frame_C to lower ball of steering rod resolved in frame_C (at initial time)";
        parameter SI.Length q0S = 0.05 "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_S annotation(
      Placement(visible = true, transformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation upper(r = rCS)  annotation(
      Placement(visible = true, transformation(origin = {-80, 20}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower(r = rCL1)  annotation(
      Placement(visible = true, transformation(origin = {-80, -18}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Revolute innerJoint(n = rCL1 - rCL2)  annotation(
      Placement(visible = true, transformation(origin = {-66, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation frontBar(r = rUL1L2 - rCL1)  annotation(
      Placement(visible = true, transformation(origin = {-36, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Assemblies.JointUPS MacPherson(n1_a = n_a)  annotation(
      Placement(visible = true, transformation(origin = {-14, -8}, extent = {{20, -20}, {-20, 20}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel struct(s_unstretched = q0Strut, d = 1, c = 1) annotation(
      Placement(visible = true, transformation(origin = {30, 44}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation springRod(r = rUS - rUL1L2)  annotation(
      Placement(visible = true, transformation(origin = {30, 0}, extent = {{-18, -18}, {18, 18}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Assemblies.JointSSR steeringJoint(rod1Mass=0, n_b = rS, rRod2_ib = rUL3 - rUL1L2) annotation(
      Placement(visible = true, transformation(origin = {80, 50}, extent = {{-20, 20}, {20, -20}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation outerRod(r = rUW - rUL1L2)  annotation(
      Placement(visible = true, transformation(origin = {78, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    parameter SI.Length q0Strut = Modelica.Math.Vectors.length(rCS - rUS) + q0S;
    parameter SI.Position rS[3] = rCS - rUL1L2;
    parameter SI.Position rU[3] = rUW - rUL1L2 "Position vector from frame_L1L2 to frame_U, resolved in frame_C";
    parameter Real n_a[3] = cross(rS, rU) "First rotation axis of universalUtilities.Joints.Joint in springJoint, resolved in frame_C";
  equation
  connect(frame_C, lower.frame_a) annotation(
      Line(points = {{-100, 0}, {-80, 0}, {-80, -8}}));
  connect(innerJoint.frame_b, frontBar.frame_a) annotation(
      Line(points = {{-56, -52}, {-46, -52}}));
  connect(frontBar.frame_b, MacPherson.frame_a) annotation(
      Line(points = {{-26, -52}, {-14, -52}, {-14, -28}}));
  connect(upper.frame_b, MacPherson.frame_b) annotation(
      Line(points = {{-80, 28}, {-5, 28}, {-5, 12}, {-14, 12}}));
  connect(MacPherson.frame_ib, struct.frame_a) annotation(
      Line(points = {{6, 8}, {6, 66}, {30, 66}, {30, 54}}, color = {95, 95, 95}));
  connect(springRod.frame_b, struct.frame_b) annotation(
      Line(points = {{30, 18}, {30, 34}}, color = {95, 95, 95}));
  connect(frame_C, upper.frame_a) annotation(
      Line(points = {{-100, 0}, {-80, 0}, {-80, 12}}));
  connect(MacPherson.frame_ia, steeringJoint.frame_b) annotation(
      Line(points = {{6, -24}, {6, -80}, {80, -80}, {80, 30}}));
  connect(innerJoint.frame_a, lower.frame_b) annotation(
      Line(points = {{-76, -52}, {-80, -52}, {-80, -28}}));
  connect(steeringJoint.frame_a, frame_S) annotation(
      Line(points = {{80, 70}, {80, 100}, {0, 100}}, color = {95, 95, 95}));
  connect(outerRod.frame_b, frame_U) annotation(
      Line(points = {{88, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(steeringJoint.frame_ib, springRod.frame_a) annotation(
      Line(points = {{60, 34}, {60, -48}, {30, -48}, {30, -18}}, color = {95, 95, 95}));
  connect(springRod.frame_a, outerRod.frame_a) annotation(
      Line(points = {{30, -18}, {30, -48}, {68, -48}, {68, 0}}, color = {95, 95, 95}));
    annotation(
      Diagram,
      Icon);
    end MacPherson;


  model FunctionalTestMacPherson "Description"
    inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation

  end FunctionalTestMacPherson;
end Suspensions;