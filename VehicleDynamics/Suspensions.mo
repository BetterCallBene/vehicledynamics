within VehicleDynamics;

package Suspensions "Suspensions, models ready to be used as front or rear suspensions."
  model MacPherson
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
    parameter SI.Position rRL_1[3]= {-0.16, 0.54, 0} "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
    parameter SI.Position rRL_2[3]= {-0.16, -0.54, 0} "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";

    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_S annotation(
      Placement(visible = true, transformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation upper(animation = true, r = rCS) annotation(
        Placement(visible = true, transformation(origin = {-78, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Joints.Revolute innerJoint(animation = true, n = rCL1 - rCL2) annotation(
        Placement(visible = true, transformation(origin = {-52, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower(animation = true, r = rCL1) annotation(
        Placement(visible = true, transformation(origin = {-78, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation frontBar(animation = true, r = rUL1L2 - rCL1) annotation(
        Placement(visible = true, transformation(origin = {-24, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation rearBar(animation = true, r = rCL2 - rUL1L2)  annotation(
        Placement(visible = true, transformation(origin = {-56, -46}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel struct(animation = true,c = 1, d = 1000, s_unstretched = q0Strut) annotation(
        Placement(visible = true, transformation(origin = {0, 34}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
    Modelica.Mechanics.MultiBody.Joints.Assemblies.JointSSR jointSSR(n_b = rCS - rUL1L2,rRod2_ib = rRL_1 - rUL1L2, rod1Length = Modelica.Math.Vectors.length(rl)) annotation(
        Placement(visible = true, transformation(origin = {54, 46}, extent = {{-20, 20}, {20, -20}}, rotation = -90)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation springRod(animation = false, r = rUS - rUL1L2) annotation(
        Placement(visible = true, transformation(origin = {17, -5}, extent = {{-17, -17}, {17, 17}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Joints.Assemblies.JointUPS MacPherson(n1_a = n_a, nAxis_ia = {0,  0,1}) annotation(
        Placement(visible = true, transformation(origin = {-44, 18}, extent = {{20, -20}, {-20, 20}}, rotation = -90)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation outerRod(r = rUW - rUL1L2) annotation(
        Placement(visible = true, transformation(origin = {74, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
    parameter SI.Length q0Strut = Modelica.Math.Vectors.length(rCS - rUS) + q0S;
    parameter SI.Position rS[3] = rCS - rUL1L2;
    parameter SI.Position rU[3] = rUW - rUL1L2 "Position vector from frame_L1L2 to frame_U, resolved in frame_C";
    parameter Real n_a[3] = cross(rS, rU) "First rotation axis of universalUtilities.Joints.Joint in springJoint, resolved in frame_C";
    parameter SI.Position rr[3]=rRL_2 - (rRL_1 + rRL_2)/2;
    parameter SI.Position rl[3]=rRL_1 - (rRL_1 + rRL_2)/2;
  equation
  connect(lower.frame_b, innerJoint.frame_a) annotation(
      Line(points = {{-68, -82}, {-62, -82}}, color = {95, 95, 95}));
  connect(innerJoint.frame_b, frontBar.frame_a) annotation(
      Line(points = {{-42, -82}, {-34, -82}}, color = {95, 95, 95}));
  connect(frontBar.frame_b, rearBar.frame_a) annotation(
      Line(points = {{-14, -82}, {-12, -82}, {-12, -46}, {-46, -46}}));
  connect(upper.frame_b, MacPherson.frame_b) annotation(
      Line(points = {{-68, 48}, {-44, 48}, {-44, 38}}, color = {95, 95, 95}));
  connect(frontBar.frame_b, MacPherson.frame_a) annotation(
      Line(points = {{-14, -82}, {-12, -82}, {-12, -38}, {-44, -38}, {-44, -2}}, color = {95, 95, 95}));
  connect(springRod.frame_a, outerRod.frame_a) annotation(
      Line(points = {{17, -22}, {17, -50}, {64, -50}, {64, 0}}, color = {95, 95, 95}));
  connect(frame_C, upper.frame_a) annotation(
      Line(points = {{-100, 0}, {-88, 0}, {-88, 48}}));
  connect(frame_C, lower.frame_a) annotation(
      Line(points = {{-100, 0}, {-88, 0}, {-88, -82}}));
  connect(MacPherson.frame_ib, struct.frame_a) annotation(
      Line(points = {{-24, 34}, {-10, 34}}, color = {95, 95, 95}));
  connect(struct.frame_b, springRod.frame_b) annotation(
      Line(points = {{10, 34}, {17, 34}, {17, 12}}));
  connect(outerRod.frame_b, frame_U) annotation(
      Line(points = {{84, 0}, {100, 0}, {100, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(MacPherson.frame_ia, jointSSR.frame_b) annotation(
      Line(points = {{-24, 2}, {-24, -30}, {54, -30}, {54, 26}}));
  connect(springRod.frame_a, jointSSR.frame_ib) annotation(
      Line(points = {{17, -22}, {17, -50}, {34, -50}, {34, 4}, {34, 30}}, color = {95, 95, 95}));
  connect(jointSSR.frame_a, frame_S) annotation(
      Line(points = {{54, 66}, {0, 66}, {0, 100}, {0, 100}}, color = {95, 95, 95}));
  end MacPherson;

  package Tests "Tests of suspensions"
    model MacPhersonBasic "Description"
    import SI=Modelica.SIunits;
    inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter SI.Position rRL_1[3]= {-0.16, 0.54, 0} "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
  parameter SI.Position rRL_2[3]= {-0.16, -0.54, 0} "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
  VehicleDynamics.Suspensions.MacPherson macPherson annotation(
      Placement(visible = true, transformation(origin = {18, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(animation = false, r = rRFrame) annotation(
      Placement(visible = true, transformation(origin = {-74, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp ramp(height = 0.1) annotation(
      Placement(visible = true, transformation(origin = {-88, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(animation = true, n = rRL_1 - rRL_2, useAxisFlange = true) annotation(
      Placement(visible = true, transformation(origin = {-38, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sources.Position position annotation(
      Placement(visible = true, transformation(origin = {-42, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body(I_11 = 1, I_22 = 1, I_33 = 1, m = 1)  annotation(
      Placement(visible = true, transformation(origin = {60, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
  parameter SI.Position rRFrame[3]= (rRL_1 + rRL_2) / 2;
  equation
    connect(ramp.y, position.s_ref) annotation(
      Line(points = {{-76, 80}, {-54, 80}, {-54, 80}, {-54, 80}}, color = {0, 0, 127}));
    connect(position.flange, prismatic.axis) annotation(
      Line(points = {{-32, 80}, {-32, 55}, {-30, 55}, {-30, 30}}, color = {0, 127, 0}));
  connect(world.frame_b, fixedTranslation.frame_a) annotation(
      Line(points = {{-68, -76}, {-84, -76}, {-84, 22}, {-84, 22}, {-84, 22}}, color = {95, 95, 95}));
  connect(world.frame_b, macPherson.frame_C) annotation(
      Line(points = {{-68, -76}, {-31, -76}, {-31, -72}, {2, -72}, {2, -18}, {8, -18}}));
  connect(fixedTranslation.frame_b, prismatic.frame_a) annotation(
      Line(points = {{-64, 22}, {-48, 22}, {-48, 24}, {-48, 24}}, color = {95, 95, 95}));
  connect(prismatic.frame_b, macPherson.frame_S) annotation(
      Line(points = {{-28, 24}, {18, 24}, {18, -8}, {18, -8}}, color = {95, 95, 95}));
  connect(macPherson.frame_U, body.frame_a) annotation(
      Line(points = {{28, -18}, {50, -18}, {50, -18}, {50, -18}}, color = {95, 95, 95}));
  end MacPhersonBasic;
  end Tests;
end Suspensions;