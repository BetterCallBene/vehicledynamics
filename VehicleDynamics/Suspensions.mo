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
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel struct(c = 1, d = 1, s_unstretched = q0Strut) annotation(
      Placement(visible = true, transformation(origin = {30, 44}, extent = {{10, 10}, {-10, -10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation springRod(r = rUS - rUL1L2) annotation(
      Placement(visible = true, transformation(origin = {30, 0}, extent = {{-18, -18}, {18, 18}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation outerRod(r = rUW - rUL1L2) annotation(
      Placement(visible = true, transformation(origin = {72, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Assemblies.JointSSR steeringJoint(n_b = rCS - rUL1L2, rRod2_ib = rUL3 - rUL1L2, rod1Mass = 0) annotation(
      Placement(visible = true, transformation(origin = {76, 50}, extent = {{-20, 20}, {20, -20}}, rotation = -90)));
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
    connect(frame_C, upper.frame_a) annotation(
      Line(points = {{-100, 0}, {-80, 0}, {-80, 12}}));
    connect(innerJoint.frame_a, lower.frame_b) annotation(
      Line(points = {{-76, -52}, {-80, -52}, {-80, -28}}));
    connect(MacPherson.frame_ib, struct.frame_a) annotation(
      Line(points = {{6, 8}, {6, 66}, {30, 66}, {30, 54}}, color = {95, 95, 95}));
    connect(springRod.frame_b, struct.frame_b) annotation(
      Line(points = {{30, 18}, {30, 34}}, color = {95, 95, 95}));
  connect(springRod.frame_a, outerRod.frame_a) annotation(
      Line(points = {{30, -18}, {30, -48}, {62, -48}, {62, -26}}, color = {95, 95, 95}));
  connect(outerRod.frame_b, frame_U) annotation(
      Line(points = {{82, -26}, {94, -26}, {94, 0}, {100, 0}}, color = {95, 95, 95}));
    connect(steeringJoint.frame_ib, springRod.frame_a) annotation(
      Line(points = {{56, 34}, {56, -48}, {30, -48}, {30, -18}}, color = {95, 95, 95}));
    connect(steeringJoint.frame_a, frame_S) annotation(
      Line(points = {{76, 70}, {76, 100}, {0, 100}}, color = {95, 95, 95}));
    connect(MacPherson.frame_ia, steeringJoint.frame_b) annotation(
      Line(points = {{6, -24}, {6, -80}, {76, -80}, {76, 30}}));
    annotation(
      Diagram,
      Icon);
    end MacPherson;


  model FunctionalTestMacPherson "Description"
    inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VehicleDynamics.Suspensions.MacPherson macPherson annotation(
      Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RackSteeringFrame(r = {-0.16, 0.54, 0.085}) annotation(
      Placement(visible = true, transformation(origin = {-66, -12}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  equation
    connect(world.frame_b, macPherson.frame_C) annotation(
      Line(points = {{-68, -76}, {-58, -76}, {-58, -40}, {-14, -40}, {-14, 0}, {-10, 0}}, color = {95, 95, 95}));
    connect(world.frame_b, RackSteeringFrame.frame_a) annotation(
      Line(points = {{-68, -76}, {-66, -76}, {-66, -22}, {-66, -22}}));
  connect(RackSteeringFrame.frame_b, macPherson.frame_S) annotation(
      Line(points = {{-66, -2}, {-66, 20}, {0, 20}, {0, 10}}, color = {95, 95, 95}));
  end FunctionalTestMacPherson;

  model ExampleJointSSR2
  import SI = Modelica.SIunits;
  inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(visible = true, transformation(origin = {-72, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter SI.Position[3] rCL1 = {0.1070, 0.55, -0.0380} "|Geometry| Vector from origin of frame_C to front link mount in chassis resolved in frame_C (at initial time)";
    parameter SI.Position[3] rCL2 = {-0.3070, 0.55, -0.0380} "|Geometry| Vector from origin of frame_C to rear link mount in chassis resolved in frame_C (at initial time)";
    parameter SI.Position[3] rCS = {-0.0295, 0.850, 0.5670} "|Geometry| Vector from origin of frame_C to strut mount in chassis resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUS = {-0.0070, 0.8733, 0.1380} "|Geometry| Vector from origin of frame_C to strut mount in uppright resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUL1L2 = {-0.0070, 0.8733, -0.0380} "|Geometry| Vector from origin of frame_C to low spindleUtilities.Joints.Joint resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUW = {0, 0.9, 0} "|Geometry| Vector from origin of frame_C to wheel centre resolved in frame_C (at initial time)";
    parameter SI.Position[3] rRL3 = {-0.12, 0.85, 0.08} "|Geometry| Vector from origin of frame_C to origin of frame_S resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUL3 = {0, 0.9, 0} "|Geometry| Vector from origin of frame_C to lower ball of steering rod resolved in frame_C (at initial time)";
    parameter SI.Length q0S = 0.05 "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
    
  Modelica.Mechanics.MultiBody.Joints.Assemblies.JointSSR jointSSR(rRod2_ib = {0.5, 0, 0}, rod1Length = 0.5)  annotation(
      Placement(visible = true, transformation(origin = {22, 2}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(animation = false, r = {0.5, 0.5, 0})  annotation(
      Placement(visible = true, transformation(origin = {-66, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(n = {0, 1, 0}, useAxisFlange = true)  annotation(
      Placement(visible = true, transformation(origin = {-28, 54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sources.Position position annotation(
      Placement(visible = true, transformation(origin = {-52, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.Ramp ramp(height = -0.3)  annotation(
      Placement(visible = true, transformation(origin = {-90, 82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation dist(r = {0, -0.5, 0})  annotation(
      Placement(visible = true, transformation(origin = {38, 52}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 1)  annotation(
      Placement(visible = true, transformation(origin = {68, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
    parameter SI.Length q0Strut = Modelica.Math.Vectors.length(rCS - rUS) + q0S;
    parameter SI.Position rS[3] = rCS - rUL1L2;
    parameter SI.Position rU[3] = rUW - rUL1L2 "Position vector from frame_L1L2 to frame_U, resolved in frame_C";
    parameter Real n_a[3] = cross(rS, rU) "First rotation axis of universalUtilities.Joints.Joint in springJoint, resolved in frame_C";
  equation
    connect(world.frame_b, jointSSR.frame_b) annotation(
      Line(points = {{-62, -54}, {42, -54}, {42, 2}}, color = {95, 95, 95}));
    connect(world.frame_b, fixedTranslation.frame_a) annotation(
      Line(points = {{-62, -54}, {-62, -19}, {-76, -19}, {-76, 56}}));
    connect(fixedTranslation.frame_b, prismatic.frame_a) annotation(
      Line(points = {{-56, 56}, {-38, 56}, {-38, 54}}));
    connect(prismatic.frame_b, jointSSR.frame_a) annotation(
      Line(points = {{-18, 54}, {2, 54}, {2, 2}}, color = {95, 95, 95}));
    connect(position.flange, prismatic.axis) annotation(
      Line(points = {{-42, 86}, {-20, 86}, {-20, 60}}, color = {0, 127, 0}));
    connect(ramp.y, position.s_ref) annotation(
      Line(points = {{-78, 82}, {-66, 82}, {-66, 86}, {-64, 86}}, color = {0, 0, 127}));
  connect(jointSSR.frame_ib, dist.frame_a) annotation(
      Line(points = {{38, 22}, {38, 22}, {38, 42}, {38, 42}}));
  connect(dist.frame_b, body.frame_a) annotation(
      Line(points = {{38, 62}, {58, 62}, {58, 76}, {58, 76}}, color = {95, 95, 95}));
  end ExampleJointSSR2;

  model ExampleJointSSR
  import SI = Modelica.SIunits;
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
      Placement(visible = true, transformation(origin = {-72, -54}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter SI.Position[3] rCL1 = {0.1070, 0.55, -0.0380} "|Geometry| Vector from origin of frame_C to front link mount in chassis resolved in frame_C (at initial time)";
    parameter SI.Position[3] rCL2 = {-0.3070, 0.55, -0.0380} "|Geometry| Vector from origin of frame_C to rear link mount in chassis resolved in frame_C (at initial time)";
    parameter SI.Position[3] rCS = {-0.0295, 0.850, 0.5670} "|Geometry| Vector from origin of frame_C to strut mount in chassis resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUS = {-0.0070, 0.8733, 0.1380} "|Geometry| Vector from origin of frame_C to strut mount in uppright resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUL1L2 = {-0.0070, 0.8733, -0.0380} "|Geometry| Vector from origin of frame_C to low spindleUtilities.Joints.Joint resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUW = {0, 0.9, 0} "|Geometry| Vector from origin of frame_C to wheel centre resolved in frame_C (at initial time)";
    parameter SI.Position[3] rRL3 = {-0.12, 0.85, 0.08} "|Geometry| Vector from origin of frame_C to origin of frame_S resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUL3 = {0, 0.9, 0} "|Geometry| Vector from origin of frame_C to lower ball of steering rod resolved in frame_C (at initial time)";
    parameter SI.Length q0S = 0.05 "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
    Modelica.Mechanics.Translational.Sources.Position position annotation(
        Placement(visible = true, transformation(origin = {-42, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Blocks.Sources.Ramp ramp(height = 0.1)  annotation(
        Placement(visible = true, transformation(origin = {-88, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(animation = false, r = rRFrame)  annotation(
      Placement(visible = true, transformation(origin = {-48, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(animation = true,n = rRL_1 - rRL_2, useAxisFlange = true)  annotation(
      Placement(visible = true, transformation(origin = {-16, 26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation upper(animation = true, r = rCS) annotation(
      Placement(visible = true, transformation(origin = {-16, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute innerJoint(animation = true, n = rCL1 - rCL2) annotation(
      Placement(visible = true, transformation(origin = {8, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower(animation = true, r = rCL1) annotation(
      Placement(visible = true, transformation(origin = {-22, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation frontBar(animation = true, r = rUL1L2 - rCL1) annotation(
      Placement(visible = true, transformation(origin = {44, -78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation rearBar(animation = true, r = rCL2 - rUL1L2)  annotation(
      Placement(visible = true, transformation(origin = {38, -50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel struct(animation = true,c = 1, d = 1000, s_unstretched = q0Strut) annotation(
      Placement(visible = true, transformation(origin = {38, -20}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Parts.Body body(animation = true, m = 1)  annotation(
      Placement(visible = true, transformation(origin = {146, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Assemblies.JointSSR jointSSR(n_b = rCS - rUL1L2,rRod2_ib = rRL_1 - rUL1L2, rod1Length = Modelica.Math.Vectors.length(rl)) annotation(
      Placement(visible = true, transformation(origin = {90, 18}, extent = {{-20, 20}, {20, -20}}, rotation = -90)));
    protected
    parameter SI.Length q0Strut = Modelica.Math.Vectors.length(rCS - rUS) + q0S;
    parameter SI.Position rS[3] = rCS - rUL1L2;
    parameter SI.Position rU[3] = rUW - rUL1L2 "Position vector from frame_L1L2 to frame_U, resolved in frame_C";
    parameter Real n_a[3] = cross(rS, rU) "First rotation axis of universalUtilities.Joints.Joint in springJoint, resolved in frame_C";
    parameter SI.Position rRL_1[3]={-0.16,0.54,0};
    parameter SI.Position rRL_2[3]={-0.16,-0.54,0};
    parameter SI.Position rRFrame[3]= (rRL_1 + rRL_2) / 2;
    parameter SI.Position rr[3]=rRL_2 - (rRL_1 + rRL_2)/2;
    parameter SI.Position rl[3]=rRL_1 - (rRL_1 + rRL_2)/2;
  equation
    connect(world.frame_b, fixedTranslation.frame_a) annotation(
      Line(points = {{-62, -54}, {-58, -54}, {-58, 26}}, color = {95, 95, 95}));
    connect(fixedTranslation.frame_b, prismatic.frame_a) annotation(
      Line(points = {{-38, 26}, {-26, 26}}, color = {95, 95, 95}));
    connect(ramp.y, position.s_ref) annotation(
      Line(points = {{-76, 80}, {-54, 80}, {-54, 80}, {-54, 80}}, color = {0, 0, 127}));
    connect(position.flange, prismatic.axis) annotation(
      Line(points = {{-32, 80}, {-8, 80}, {-8, 32}, {-8, 32}}, color = {0, 127, 0}));
    connect(lower.frame_b, innerJoint.frame_a) annotation(
      Line(points = {{-12, -80}, {-2, -80}}, color = {95, 95, 95}));
    connect(world.frame_b, lower.frame_a) annotation(
      Line(points = {{-62, -54}, {-32, -54}, {-32, -80}, {-32, -80}}, color = {95, 95, 95}));
    connect(world.frame_b, upper.frame_a) annotation(
      Line(points = {{-62, -54}, {-26, -54}, {-26, -22}}, color = {95, 95, 95}));
    connect(innerJoint.frame_b, frontBar.frame_a) annotation(
      Line(points = {{18, -80}, {26, -80}, {26, -78}, {34, -78}}, color = {95, 95, 95}));
    connect(frontBar.frame_b, rearBar.frame_a) annotation(
      Line(points = {{54, -78}, {48, -78}, {48, -50}, {48, -50}, {48, -50}}));
    connect(upper.frame_b, struct.frame_a) annotation(
      Line(points = {{-6, -22}, {28, -22}, {28, -20}, {28, -20}}, color = {95, 95, 95}));
    connect(struct.frame_b, frontBar.frame_b) annotation(
      Line(points = {{48, -20}, {54, -20}, {54, -78}, {54, -78}}, color = {95, 95, 95}));
    connect(frontBar.frame_b, body.frame_a) annotation(
      Line(points = {{54, -78}, {136, -78}, {136, -76}}, color = {95, 95, 95}));
    connect(frontBar.frame_b, jointSSR.frame_b) annotation(
      Line(points = {{54, -78}, {90, -78}, {90, -2}}));
    connect(prismatic.frame_b, jointSSR.frame_a) annotation(
      Line(points = {{-6, 26}, {49, 26}, {49, 38}, {90, 38}}, color = {95, 95, 95}));
  end ExampleJointSSR;

  model ExampleMacPherson
  import SI = Modelica.SIunits;
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
      Placement(visible = true, transformation(origin = {-132, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter SI.Position[3] rCL1 = {0.1070, 0.55, -0.0380} "|Geometry| Vector from origin of frame_C to front link mount in chassis resolved in frame_C (at initial time)";
    parameter SI.Position[3] rCL2 = {-0.3070, 0.55, -0.0380} "|Geometry| Vector from origin of frame_C to rear link mount in chassis resolved in frame_C (at initial time)";
    parameter SI.Position[3] rCS = {-0.0295, 0.850, 0.5670} "|Geometry| Vector from origin of frame_C to strut mount in chassis resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUS = {-0.0070, 0.8733, 0.1380} "|Geometry| Vector from origin of frame_C to strut mount in uppright resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUL1L2 = {-0.0070, 0.8733, -0.0380} "|Geometry| Vector from origin of frame_C to low spindleUtilities.Joints.Joint resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUW = {0, 0.9, 0} "|Geometry| Vector from origin of frame_C to wheel centre resolved in frame_C (at initial time)";
    parameter SI.Position[3] rRL3 = {-0.12, 0.85, 0.08} "|Geometry| Vector from origin of frame_C to origin of frame_S resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUL3 = {0, 0.9, 0} "|Geometry| Vector from origin of frame_C to lower ball of steering rod resolved in frame_C (at initial time)";
    parameter SI.Length q0S = 0.05 "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
    parameter SI.Position[3] rRackSteering = {-0.16, 0.54, 0.085} "|Geometry| Vector rRackSteering";
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower(r = rCL1)  annotation(
      Placement(visible = true, transformation(origin = {-88, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation upper(r = rCS)  annotation(
      Placement(visible = true, transformation(origin = {-82, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute innerJoint(n = rCL1 - rCL2)  annotation(
      Placement(visible = true, transformation(origin = {-58, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation frontBar(r = rUL1L2 - rCL1)  annotation(
      Placement(visible = true, transformation(origin = {-22, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Assemblies.JointUPS MacPherson(n1_a = n_a)  annotation(
      Placement(visible = true, transformation(origin = {6, -52}, extent = {{20, -20}, {-20, 20}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation springRod(r = rUS - rUL1L2) annotation(
      Placement(visible = true, transformation(origin = {38, 20}, extent = {{-18, -18}, {18, 18}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Body body(I_11 = 1, I_22 = 1, I_33 = 1, m = 1)  annotation(
      Placement(visible = true, transformation(origin = {154, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation outerRod(r = rUW - rUL1L2) annotation(
      Placement(visible = true, transformation(origin = {62, -84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Assemblies.JointSSR steeringJoint(n_b = rCS - rUL1L2, rRod2_ib = rUL3 - rUL1L2, rod1Length = Modelica.Math.Vectors.length(rUL3 - rRackSteering), rod1Mass = 0) annotation(
      Placement(visible = true, transformation(origin = {76, 46}, extent = {{-20, 20}, {20, -20}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RackSteeringFrame(r = rRFrame) annotation(
      Placement(visible = true, transformation(origin = {-51, 75}, extent = {{25, -25}, {-25, 25}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel struct(c = 1, d = 1000, s_unstretched = q0Strut) annotation(
      Placement(visible = true, transformation(origin = {-8, 48}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(n = -(rRL_1 + rRL_2) / 2)  annotation(
      Placement(visible = true, transformation(origin = {0, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation RackLeft(r = rl)  annotation(
      Placement(visible = true, transformation(origin = {30, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
    parameter SI.Length q0Strut = Modelica.Math.Vectors.length(rCS - rUS) + q0S;
      parameter SI.Position rS[3] = rCS - rUL1L2;
      parameter SI.Position rU[3] = rUW - rUL1L2 "Position vector from frame_L1L2 to frame_U, resolved in frame_C";
      parameter Real n_a[3] = cross(rS, rU) "First rotation axis of universalUtilities.Joints.Joint in springJoint, resolved in frame_C";
    parameter SI.Position rRL_1[3]={-0.16,0.54,0.085};
    parameter SI.Position rRL_2[3]={-0.16,-0.54,0.085};
    parameter SI.Position rRFrame[3]= (rRL_1 + rRL_2) / 2.0;
    parameter SI.Position rr[3]=rRL_2 - (rRL_1 + rRL_2)/2.0;
    parameter SI.Position rl[3]=rRL_1 - (rRL_1 + rRL_2)/2.0;
  equation
    connect(world.frame_b, lower.frame_a) annotation(
      Line(points = {{-122, -84}, {-98, -84}}, color = {95, 95, 95}));
    connect(world.frame_b, upper.frame_a) annotation(
      Line(points = {{-122, -84}, {-92, -84}, {-92, -4}}));
    connect(lower.frame_b, innerJoint.frame_a) annotation(
      Line(points = {{-78, -84}, {-68, -84}}, color = {95, 95, 95}));
    connect(innerJoint.frame_b, frontBar.frame_a) annotation(
      Line(points = {{-48, -84}, {-32, -84}}, color = {95, 95, 95}));
  connect(upper.frame_b, MacPherson.frame_b) annotation(
      Line(points = {{-72, -4}, {-27, -4}, {-27, -32}, {6, -32}}));
  connect(frontBar.frame_b, MacPherson.frame_a) annotation(
      Line(points = {{-12, -84}, {6, -84}, {6, -72}}, color = {95, 95, 95}));
  connect(outerRod.frame_a, springRod.frame_a) annotation(
      Line(points = {{52, -84}, {38, -84}, {38, 2}}));
    connect(outerRod.frame_b, body.frame_a) annotation(
      Line(points = {{72, -84}, {142, -84}, {142, -4}, {144, -4}}));
  connect(MacPherson.frame_ia, steeringJoint.frame_b) annotation(
      Line(points = {{26, -68}, {76, -68}, {76, 26}}, color = {95, 95, 95}));
  connect(springRod.frame_a, steeringJoint.frame_ib) annotation(
      Line(points = {{38, 2}, {56, 2}, {56, 30}}, color = {95, 95, 95}));
  connect(world.frame_b, RackSteeringFrame.frame_a) annotation(
      Line(points = {{-122, -84}, {-76, -84}, {-76, 75}}, color = {95, 95, 95}));
  connect(MacPherson.frame_ib, struct.frame_a) annotation(
      Line(points = {{26, -36}, {-18, -36}, {-18, 48}}, color = {95, 95, 95}));
  connect(struct.frame_b, springRod.frame_b) annotation(
      Line(points = {{2, 48}, {19, 48}, {19, 38}, {38, 38}}, color = {95, 95, 95}));
  connect(RackSteeringFrame.frame_b, prismatic.frame_a) annotation(
      Line(points = {{-26, 75}, {-21, 75}, {-21, 76}, {-10, 76}}, color = {95, 95, 95}));
  connect(prismatic.frame_b, RackLeft.frame_a) annotation(
      Line(points = {{10, 76}, {14, 76}, {14, 84}, {20, 84}}, color = {95, 95, 95}));
  connect(RackLeft.frame_b, steeringJoint.frame_a) annotation(
      Line(points = {{40, 84}, {76, 84}, {76, 66}}, color = {95, 95, 95}));
  end ExampleMacPherson;
end Suspensions;