within VehicleDynamics;

package Suspensions "Suspensions, models ready to be used as front or rear suspensions."
  //   model SpringDamperLinearBump3D "Spring-damper in parallel with bumps at ends."
  //     import SI = Modelica.SIunits;
  //     import Frames = Modelica.Mechanics.MultiBody.Frames;
  //     //extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
  //     parameter SI.Position[3] r_rela0 = {0, 0, 0} "Unstretched translational spring vector";
  //     parameter SI.Position[3] r_min = {-0.05, -0.03, -0.06} "vector to bump stop from reference point in negative direction";
  //     parameter SI.Position[3] r_max = {0.05, 0.03, 0.06} "vector to bump stop from reference point in positive direction";
  //     parameter Real bumpFactor = 1000 "scaleFactor of spring force at after bump";
  //     parameter SI.Angle[3] phi_rela0 = {0, 0, 0} "Unstretched rotational spring vector";
  //     parameter Real[6, 6] C = [10000, 0, 0, 0, 0, 0; 0, 10000, 0, 0, 0, 0; 0, 0, 10000, 0, 0, 0; 0, 0, 0, 10000, 0, 0; 0, 0, 0, 0, 10000, 0; 0, 0, 0, 0, 0, 10000] "|Forces|Linear|Stiffness matrix";
  //     parameter Real[6, 6] D = [10000, 0, 0, 0, 0, 0; 0, 10000, 0, 0, 0, 0; 0, 0, 10000, 0, 0, 0; 0, 0, 0, 10000, 0, 0; 0, 0, 0, 0, 10000, 0; 0, 0, 0, 0, 0, 10000] "|Forces|Linear|Damping matrix";
  //     parameter Real eps = 1.E-6 "Guard against division by zero";
  //     parameter Boolean animation = true "True, if animation shall be enabled";
  //     parameter SI.Position width = r_max[2] - r_min[2] "|Animation|if animation = true| Width bushing";
  //     parameter SI.Position length = r_max[1] - r_min[1] "|Animation|if animation = true| Width bushing";
  //     parameter SI.Position height = r_max[3] - r_min[3] "|Animation|if animation = true| Width bushing";
  //     inner Modelica.Mechanics.MultiBody.World world annotation(
  //       Placement(visible = true, transformation(origin = {-84, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //     Modelica.Mechanics.MultiBody.Parts.FixedRotation fixedRotation annotation(
  //       Placement(visible = true, transformation(origin = {-44, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //   protected
  //     Real[3, 3] R_rel;
  //     Real e_a[3](each final unit = "1") "Unit vector on the line connecting the origin of frame_a with the origin of frame_b resolved in frame_a (directed from frame_a to frame_b)";
  //     Modelica.SIunits.Position r_rel_a[3] "Position vector from origin of frame_a to origin of frame_b, resolved in frame_a";
  //   equation
  // //  R_rel   = Frames.relativeRotation(frame_a.R, frame_b.R);
  // //  r_rel_a = Frames.resolve2(frame_a.R, frame_b.r_0 - frame_a.r_0);
  // //  s = noEvent(max(Modelica.Math.Vectors.length(r_rel_a), eps));
  // //  e_a = r_rel_a/s;
  //     connect(world.frame_b, fixedRotation.frame_a) annotation(
  //       Line(points = {{-74, -74}, {-74, -42}, {-54, -42}, {-54, -10}}));
  //   end SpringDamperLinearBump3D;

  model FunctionalTestMacPherson "Description"
    import SI = Modelica.SIunits;
    inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    parameter SI.Position rRL_1[3] = {-0.16, 0.54, 0} "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
    parameter SI.Position rRL_2[3] = {-0.16, -0.54, 0} "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
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
    Modelica.Mechanics.MultiBody.Parts.Body body(I_11 = 1, I_22 = 1, I_33 = 1, m = 1) annotation(
      Placement(visible = true, transformation(origin = {60, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    parameter SI.Position rRFrame[3] = (rRL_1 + rRL_2) / 2;
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
  end FunctionalTestMacPherson;

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
    parameter SI.Position rRL_1[3] = {-0.16, 0.54, 0} "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
    parameter SI.Position rRL_2[3] = {-0.16, -0.54, 0} "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
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
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation rearBar(animation = true, r = rCL2 - rUL1L2) annotation(
      Placement(visible = true, transformation(origin = {-56, -46}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel struct(animation = true, c = 1, d = 1000, s_unstretched = q0Strut) annotation(
      Placement(visible = true, transformation(origin = {0, 34}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
    Modelica.Mechanics.MultiBody.Joints.Assemblies.JointSSR jointSSR(n_b = rCS - rUL1L2, rRod2_ib = rRL_1 - rUL1L2, rod1Length = Modelica.Math.Vectors.length(rl)) annotation(
      Placement(visible = true, transformation(origin = {54, 46}, extent = {{-20, 20}, {20, -20}}, rotation = -90)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation springRod(animation = false, r = rUS - rUL1L2) annotation(
      Placement(visible = true, transformation(origin = {17, -5}, extent = {{-17, -17}, {17, 17}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Joints.Assemblies.JointUPS MacPherson(n1_a = n_a, nAxis_ia = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-44, 18}, extent = {{20, -20}, {-20, 20}}, rotation = -90)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation outerRod(r = rUW - rUL1L2) annotation(
      Placement(visible = true, transformation(origin = {74, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    parameter SI.Length q0Strut = Modelica.Math.Vectors.length(rCS - rUS) + q0S;
    parameter SI.Position rS[3] = rCS - rUL1L2;
    parameter SI.Position rU[3] = rUW - rUL1L2 "Position vector from frame_L1L2 to frame_U, resolved in frame_C";
    parameter Real n_a[3] = cross(rS, rU) "First rotation axis of universalUtilities.Joints.Joint in springJoint, resolved in frame_C";
    parameter SI.Position rr[3] = rRL_2 - (rRL_1 + rRL_2) / 2;
    parameter SI.Position rl[3] = rRL_1 - (rRL_1 + rRL_2) / 2;
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

  model FiveLink "Five links constrain the motion of the upright and reduces the degrees of freedom to one."
    import SI = Modelica.SIunits;
    parameter Real[3] scaleFactor = {1, 1, 1} "To make right hand side Linkage use {1,-1,1}";
    //fivelink
    parameter SI.Position[3] rCL1 = {-2.6430, 0.231, -0.022} "|Geometry|Left fivelink|link 1 to chassis/body";
    parameter SI.Position[3] rCL2 = {-2.5039, 0.4150, 0.174} "|Geometry|Left fivelink|link 2 to chassis/body";
    parameter SI.Position[3] rCL3 = {-2.4617, 0.3750, 0.0637} "|Geometry|Left fivelink|link 3 to chassis/body";
    parameter SI.Position[3] rCL4 = {-2.6617, 0.2950, 0.1637} "|Geometry|Left fivelink|link 4 to chassis/body";
    parameter SI.Position[3] rCL5 = {-2.1617, 0.4750, -0.0637} "|Geometry|Left fivelink|link 5 to chassis/body";
    parameter SI.Position[3] rUL1 = {-2.6170, 0.6672, -0.05327} "|Geometry|Left fivelink|link 1 to upright";
    parameter SI.Position[3] rUL2 = {-2.5178, 0.6678, 0.1607} "|Geometry|Left fivelink|link 2 to upright";
    parameter SI.Position[3] rUL3 = {-2.4270, 0.612, 0.050} "|Geometry|Left fivelink|link 3 to upright";
    parameter SI.Position[3] rUL4 = {-2.6270, 0.612, 0.150} "|Geometry|Left fivelink|link 4 to upright";
    parameter SI.Position[3] rUL5 = {-2.4270, 0.612, -0.010} "|Geometry|Left fivelink|link 5 to upright";
    parameter SI.Position[3] rUW = {-2.557, 0.7373, 0.0290} "|Geometry|Left fivelink|Wheel centre";
    // strut
    parameter SI.Position[3] rCS = {(-2.55) - 0.0621, 0.518, 0.450} "|Geometry|Left strut|Strut to chassis/body";
    parameter SI.Position[3] rUS = {(-2.55) - 0.063305, 0.581, -0.046} "|Geometry|Left strut|Strut ot upright";
    inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
      Placement(visible = true, transformation(origin = {-82, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.Link link1(rA = rCL1, rB = rUW - rUL1, rRod_ia = rUL1 - rCL1) annotation(
      Placement(visible = true, transformation(origin = {-16, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    CutLink link2(rA = rCL2, rB = rUW - rUL2, rodLength = Modelica.Math.Vectors.length(rUL2 - rCL2)) annotation(
      Placement(visible = true, transformation(origin = {-16, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    CutLink link3(rA = rCL3, rB = rUW - rUL3, rodLength = Modelica.Math.Vectors.length(rUL3 - rCL3)) annotation(
      Placement(visible = true, transformation(origin = {-16, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    CutLink link4(rA = rCL4, rB = rUW - rUL4, rodLength = Modelica.Math.Vectors.length(rUL4 - rCL4)) annotation(
      Placement(visible = true, transformation(origin = {-18, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    CutLink link5(rA = rCL5, rB = rUW - rUL5, rodLength = Modelica.Math.Vectors.length(rUL5 - rCL5)) annotation(
      Placement(visible = true, transformation(origin = {-18, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(c = 1, d = 10000, s_unstretched = Modelica.Math.Vectors.length(rCS - rUS) + 0.116) annotation(
      Placement(visible = true, transformation(origin = {-18, 86}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation frameTranslation(animation = false, r = rCS) annotation(
      Placement(visible = true, transformation(origin = {-52, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation frameTranslation2(r = rUS - rUW) annotation(
      Placement(visible = true, transformation(origin = {18, 86}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Body body(m = 1, r_CM = {0, 0, 0}) annotation(
      Placement(visible = true, transformation(origin = {66, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    
  equation
    connect(world.frame_b, link1.frame_a) annotation(
      Line(points = {{-72, -24}, {-60, -24}, {-60, 56}, {-26, 56}}));
    connect(world.frame_b, link2.frame_a) annotation(
      Line(points = {{-72, -24}, {-60, -24}, {-60, 22}, {-26, 22}}, color = {95, 95, 95}));
    connect(world.frame_b, link3.frame_a) annotation(
      Line(points = {{-72, -24}, {-60, -24}, {-60, -6}, {-26, -6}}));
    connect(world.frame_b, link4.frame_a) annotation(
      Line(points = {{-72, -24}, {-72, -24}, {-60, -24}, {-60, -36}, {-28, -36}}, color = {95, 95, 95}));
    connect(world.frame_b, link5.frame_a) annotation(
      Line(points = {{-72, -24}, {-60, -24}, {-60, -66}, {-28, -66}}, color = {95, 95, 95}));
    connect(springDamperParallel.frame_b, frameTranslation.frame_b) annotation(
      Line(points = {{-28, 86}, {-42, 86}, {-42, 84}, {-42, 84}}, color = {95, 95, 95}));
  connect(springDamperParallel.frame_a, frameTranslation2.frame_b) annotation(
      Line(points = {{-8, 86}, {8, 86}, {8, 86}, {8, 86}}, color = {95, 95, 95}));
    connect(link1.frame_b, body.frame_a) annotation(
      Line(points = {{-6, 56}, {56, 56}, {56, -6}}));
    connect(link2.frame_b, body.frame_a) annotation(
      Line(points = {{-6, 22}, {56, 22}, {56, -6}}, color = {95, 95, 95}));
    connect(link3.frame_b, body.frame_a) annotation(
      Line(points = {{-6, -6}, {56, -6}}, color = {95, 95, 95}));
    connect(link4.frame_b, body.frame_a) annotation(
      Line(points = {{-8, -36}, {56, -36}, {56, -6}}));
    connect(link5.frame_b, body.frame_a) annotation(
      Line(points = {{-8, -66}, {56, -66}, {56, -6}}));
    connect(world.frame_b, frameTranslation.frame_a) annotation(
      Line(points = {{-72, -24}, {-62, -24}, {-62, 84}, {-62, 84}}));
  connect(frameTranslation2.frame_a, body.frame_a) annotation(
      Line(points = {{28, 86}, {56, 86}, {56, -6}, {56, -6}}, color = {95, 95, 95}));
  end FiveLink;

  model FiveLink2 "Five links constrain the motion of the upright and reduces the degrees of freedom to one."
    import SI = Modelica.SIunits;
    parameter Real[3] scaleFactor = {1, 1, 1} "To make right hand side Linkage use {1,-1,1}";
    //fivelink
    parameter SI.Position[3] rCL1 = {-2.6430, 0.231, -0.022} "|Geometry|Left fivelink|link 1 to chassis/body";
    parameter SI.Position[3] rCL2 = {-2.5039, 0.4150, 0.174} "|Geometry|Left fivelink|link 2 to chassis/body";
    parameter SI.Position[3] rCL3 = {-2.4617, 0.3750, 0.0637} "|Geometry|Left fivelink|link 3 to chassis/body";
    parameter SI.Position[3] rCL4 = {-2.6617, 0.2950, 0.1637} "|Geometry|Left fivelink|link 4 to chassis/body";
    parameter SI.Position[3] rCL5 = {-2.1617, 0.4750, -0.0637} "|Geometry|Left fivelink|link 5 to chassis/body";
    parameter SI.Position[3] rUL1 = {-2.6170, 0.6672, -0.05327} "|Geometry|Left fivelink|link 1 to upright";
    parameter SI.Position[3] rUL2 = {-2.5178, 0.6678, 0.1607} "|Geometry|Left fivelink|link 2 to upright";
    parameter SI.Position[3] rUL3 = {-2.4270, 0.612, 0.050} "|Geometry|Left fivelink|link 3 to upright";
    parameter SI.Position[3] rUL4 = {-2.6270, 0.612, 0.150} "|Geometry|Left fivelink|link 4 to upright";
    parameter SI.Position[3] rUL5 = {-2.4270, 0.612, -0.010} "|Geometry|Left fivelink|link 5 to upright";
    parameter SI.Position[3] rUW = {-2.557, 0.7373, 0.0290} "|Geometry|Left fivelink|Wheel centre";
    // strut
    parameter SI.Position[3] rCS = {(-2.55) - 0.0621, 0.518, 0.450} "|Geometry|Left strut|Strut to chassis/body";
    parameter SI.Position[3] rUS = {(-2.55) - 0.063305, 0.581, -0.046} "|Geometry|Left strut|Strut ot upright";
    inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
      Placement(visible = true, transformation(origin = {-82, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.Link link1(rA = rCL1, rB = rUW - rUL1, rRod_ia = rUL1 - rCL1) annotation(
      Placement(visible = true, transformation(origin = {-16, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    CutLink2 link2(rA = rCL2, rB = rUW - rUL2, rRod_ia = rUL2 - rCL2) annotation(
      Placement(visible = true, transformation(origin = {-16, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    CutLink2 link3(rA = rCL3, rB = rUW - rUL3, rRod_ia = rUL3 - rCL3) annotation(
      Placement(visible = true, transformation(origin = {-16, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    CutLink2 link4(rA = rCL4, rB = rUW - rUL4, rRod_ia = rUL4 - rCL4) annotation(
      Placement(visible = true, transformation(origin = {-18, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    CutLink2 link5(rA = rCL5, rB = rUW - rUL5, rRod_ia = rUL5 - rCL5) annotation(
      Placement(visible = true, transformation(origin = {-18, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(c = 1, d = 1000, s_unstretched = Modelica.Math.Vectors.length(rCS - rUS) + 0.116) annotation(
      Placement(visible = true, transformation(origin = {-18, 86}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation frameTranslation(animation = false, r = rCS) annotation(
      Placement(visible = true, transformation(origin = {-52, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation frameTranslation2(r = rUS - rUW) annotation(
      Placement(visible = true, transformation(origin = {18, 86}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Body body(m = 1) annotation(
      Placement(visible = true, transformation(origin = {66, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
    
  equation
    connect(world.frame_b, link1.frame_a) annotation(
      Line(points = {{-72, -24}, {-60, -24}, {-60, 56}, {-26, 56}}));
    connect(world.frame_b, link2.frame_a) annotation(
      Line(points = {{-72, -24}, {-60, -24}, {-60, 22}, {-26, 22}}, color = {95, 95, 95}));
    connect(world.frame_b, link3.frame_a) annotation(
      Line(points = {{-72, -24}, {-60, -24}, {-60, -6}, {-26, -6}}));
    connect(world.frame_b, link4.frame_a) annotation(
      Line(points = {{-72, -24}, {-72, -24}, {-60, -24}, {-60, -36}, {-28, -36}}, color = {95, 95, 95}));
    connect(world.frame_b, link5.frame_a) annotation(
      Line(points = {{-72, -24}, {-60, -24}, {-60, -66}, {-28, -66}}, color = {95, 95, 95}));
    connect(springDamperParallel.frame_b, frameTranslation.frame_b) annotation(
      Line(points = {{-28, 86}, {-42, 86}, {-42, 84}, {-42, 84}}, color = {95, 95, 95}));
  connect(springDamperParallel.frame_a, frameTranslation2.frame_b) annotation(
      Line(points = {{-8, 86}, {8, 86}, {8, 86}, {8, 86}}, color = {95, 95, 95}));
    connect(link1.frame_b, body.frame_a) annotation(
      Line(points = {{-6, 56}, {56, 56}, {56, -6}}));
    connect(link2.frame_b, body.frame_a) annotation(
      Line(points = {{-6, 22}, {56, 22}, {56, -6}}, color = {95, 95, 95}));
    connect(link3.frame_b, body.frame_a) annotation(
      Line(points = {{-6, -6}, {56, -6}}, color = {95, 95, 95}));
    connect(link4.frame_b, body.frame_a) annotation(
      Line(points = {{-8, -36}, {56, -36}, {56, -6}}));
    connect(link5.frame_b, body.frame_a) annotation(
      Line(points = {{-8, -66}, {56, -66}, {56, -6}}));
    connect(world.frame_b, frameTranslation.frame_a) annotation(
      Line(points = {{-72, -24}, {-62, -24}, {-62, 84}, {-62, 84}}));
  connect(frameTranslation2.frame_a, body.frame_a) annotation(
      Line(points = {{28, 86}, {56, 86}, {56, -6}, {56, -6}}, color = {95, 95, 95}));
  end FiveLink2;

  model CutLink "Two bushings connected by a massless rod"
    import SI = Modelica.SIunits;
    extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
    parameter SI.Position[3] rA = {0, 0, 0} "Position vector from frame_a to center of bushingA, resolved in frame_a";
    parameter SI.Position[3] rB = {0, 0, 0} "Position vector from center of bushingb to frame_b, resolved in frame_b";
    parameter SI.Length rodLength;
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotA(animation = false, r = rA) annotation(
      Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotB(r = -rB) annotation(
      Placement(visible = true, transformation(origin = {68, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Joints.SphericalSpherical jointSS(rodLength = rodLength, showMass = false) annotation(
      Placement(visible = true, transformation(origin = {-4, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  initial equation

  equation
    connect(jointSS.frame_a, rotA.frame_b) annotation(
      Line(points = {{-14, 0}, {-70, 0}, {-70, 0}, {-70, 0}}, color = {95, 95, 95}));
    connect(jointSS.frame_b, rotB.frame_b) annotation(
      Line(points = {{6, 0}, {58, 0}, {58, 0}, {58, 0}}));
    connect(rotB.frame_a, frame_b) annotation(
      Line(points = {{78, 0}, {100, 0}, {100, 0}, {100, 0}}, color = {95, 95, 95}));
    connect(frame_a, rotA.frame_a) annotation(
      Line(points = {{-100, 0}, {-88, 0}, {-88, 0}, {-90, 0}}));
  end CutLink;

  model CutLink2 "Two bushings connected by a massless rod"
    import SI = Modelica.SIunits;
    extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
    parameter Modelica.Mechanics.MultiBody.Types.Axis n1_a = {0, 0, 1};
    parameter SI.Position[3] rA = {0, 0, 0} "Position vector from frame_a to center of bushingA, resolved in frame_a";
    parameter SI.Position[3] rB = {0, 0, 0} "Position vector from center of bushingb to frame_b, resolved in frame_b";
    parameter SI.Position rRod_ia[3] = {1, 0, 0};
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotA(animation = false, r = rA) annotation(
      Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotB(r = -rB) annotation(
      Placement(visible = true, transformation(origin = {68, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.UniversalSpherical joint(n1_a = n1_a, rRod_ia = rRod_ia)  annotation(
      Placement(visible = true, transformation(origin = {-10, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  initial equation

  equation
    connect(rotB.frame_a, frame_b) annotation(
      Line(points = {{78, 0}, {100, 0}, {100, 0}, {100, 0}}, color = {95, 95, 95}));
    connect(frame_a, rotA.frame_a) annotation(
      Line(points = {{-100, 0}, {-88, 0}, {-88, 0}, {-90, 0}}));
  connect(rotA.frame_b, joint.frame_a) annotation(
      Line(points = {{-70, 0}, {-20, 0}, {-20, 0}, {-20, 0}}, color = {95, 95, 95}));
  connect(joint.frame_b, rotB.frame_b) annotation(
      Line(points = {{0, 0}, {58, 0}, {58, 0}, {58, 0}}, color = {95, 95, 95}));
  end CutLink2;

  model Link "Two bushings connected by a massless rod"
    import SI = Modelica.SIunits;
    extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_ia annotation(
      Placement(visible = true, transformation(origin = {-44, 100}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {-44, 100}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    parameter Modelica.Mechanics.MultiBody.Types.Axis n1_a = {0, 0, 1};
    parameter SI.Position rA[3] = {0, 0, 0};
    parameter SI.Position rB[3] = {0, 0, 0};
    parameter SI.Position rRod_ia[3] = {1, 0, 0};
    Modelica.Mechanics.MultiBody.Joints.Spherical jointB annotation(
      Placement(visible = true, transformation(origin = {22, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Joints.Universal jointA(n_b = cross(n1_a, rRod_ia), n_a = cross(rRod_ia, jointA.n_b)) annotation(
      Placement(visible = true, transformation(origin = {-54, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation rod(r = rRod_ia) annotation(
      Placement(visible = true, transformation(origin = {-14, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotA(animation = false, r = rA) annotation(
      Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotB(r = -rB) annotation(
      Placement(visible = true, transformation(origin = {68, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
    connect(jointA.frame_b, rod.frame_a) annotation(
      Line(points = {{-44, 0}, {-24, 0}}, color = {95, 95, 95}));
    connect(rod.frame_b, jointB.frame_b) annotation(
      Line(points = {{-4, 0}, {12, 0}}, color = {95, 95, 95}));
    connect(jointA.frame_b, frame_ia) annotation(
      Line(points = {{-44, 0}, {-44, 100}}));
    connect(frame_a, rotA.frame_a) annotation(
      Line(points = {{-100, 0}, {-90, 0}, {-90, 0}, {-90, 0}}));
    connect(rotA.frame_b, jointA.frame_a) annotation(
      Line(points = {{-70, 0}, {-64, 0}, {-64, 0}, {-64, 0}}, color = {95, 95, 95}));
    connect(rotB.frame_b, jointB.frame_a) annotation(
      Line(points = {{58, 0}, {32, 0}}));
    connect(rotB.frame_a, frame_b) annotation(
      Line(points = {{78, 0}, {100, 0}}, color = {95, 95, 95}));
  end Link;

  model ExampleMultiLink
    import SI = Modelica.SIunits;
    inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    parameter SI.Position[3] rCL1 = {-0.5550, -0.3690, -0.0320};
    parameter SI.Position[3] rCL2 = {-0.4240, -0.1900, 0.1640};
    parameter SI.Position[3] rCL3 = {-0.3320, -0.1300, 0.0570};
    parameter SI.Position[3] rCU = {0, 0, 0};
    parameter SI.Position[3] rUL1 = {-0.5250, 0.0620, -0.0670};
    parameter SI.Position[3] rUL2 = {-0.4330, 0.0680, 0.1500};
    parameter SI.Position[3] rUL3 = {-0.3350, 0.0116, 0.0400};
    parameter SI.Position[3] rUW = {-0.4650, 0.1330, 0.0200};
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation Link1(r = rCL1) annotation(
      Placement(visible = true, transformation(origin = {-44, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation Link2(r = rCL2) annotation(
      Placement(visible = true, transformation(origin = {-46, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation Link3(r = rCL3) annotation(
      Placement(visible = true, transformation(origin = {-44, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation Link11(r = rUL1 - rCL1) annotation(
      Placement(visible = true, transformation(origin = {-10, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation Link12(r = rCU - rUL1) annotation(
      Placement(visible = true, transformation(origin = {24, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation Link21(r = rUL2 - rCL2) annotation(
      Placement(visible = true, transformation(origin = {-16, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation Link22(r = rCU - rUL2) annotation(
      Placement(visible = true, transformation(origin = {24, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation Link31(r = rUL3 - rCL3) annotation(
      Placement(visible = true, transformation(origin = {-16, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation Link32(r = rCU - rUL3) annotation(
      Placement(visible = true, transformation(origin = {26, -52}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(world.frame_b, Link1.frame_a) annotation(
      Line(points = {{-70, 0}, {-54, 0}, {-54, 40}}, color = {95, 95, 95}));
    connect(Link2.frame_a, world.frame_b) annotation(
      Line(points = {{-56, -10}, {-70, -10}, {-70, 0}}, color = {95, 95, 95}));
    connect(world.frame_b, Link3.frame_a) annotation(
      Line(points = {{-70, 0}, {-54, 0}, {-54, -52}, {-54, -52}}, color = {95, 95, 95}));
    connect(Link1.frame_b, Link11.frame_a) annotation(
      Line(points = {{-34, 40}, {-20, 40}}));
    connect(Link11.frame_b, Link12.frame_a) annotation(
      Line(points = {{0, 40}, {14, 40}, {14, 40}, {14, 40}}, color = {95, 95, 95}));
    connect(Link2.frame_b, Link21.frame_a) annotation(
      Line(points = {{-36, -10}, {-24, -10}, {-24, -10}, {-26, -10}}, color = {95, 95, 95}));
    connect(Link21.frame_b, Link22.frame_a) annotation(
      Line(points = {{-6, -10}, {14, -10}}, color = {95, 95, 95}));
    connect(Link3.frame_b, Link31.frame_a) annotation(
      Line(points = {{-34, -52}, {-24, -52}, {-24, -52}, {-26, -52}}, color = {95, 95, 95}));
    connect(Link31.frame_b, Link32.frame_a) annotation(
      Line(points = {{-6, -52}, {16, -52}}));
  end ExampleMultiLink;

  model ExampleLink "Example MultiLink"
    inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.Link link(n1_a = {0, 0, 1}, rA = {0, 0, 0.5}, rB = {0.5, 0, 0}) annotation(
      Placement(visible = true, transformation(origin = {-32, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Body body(m = 1) annotation(
      Placement(visible = true, transformation(origin = {58, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(world.frame_b, link.frame_a) annotation(
      Line(points = {{-70, 0}, {-42, 0}}, color = {95, 95, 95}));
    connect(link.frame_b, body.frame_a) annotation(
      Line(points = {{-22, 0}, {48, 0}, {48, 0}, {48, 0}}, color = {95, 95, 95}));
  end ExampleLink;

  // model CutPushRod "Two bushings connected by a massless rod"
  //   import SI = Modelica.SIunits;
  //   parameter SI.Position[3] rA = {0, 0, 0} "Position vector from frame_a to center of bushingA, resolved in frame_a";
  //   parameter SI.Position[3] rB = {0, 0, 0} "Position vector from center of bushingb to frame_b, resolved in frame_b";
  //   parameter SI.Position[3] rRod = {1, 0, 0} "Position vector from rod.frame_a to rod.frame_b, resolved in frame_a at the initial configuration";
  //   inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
  //     Placement(visible = true, transformation(origin = {-86, -74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  //   Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(animation = false, r = {1, 0, 0}) annotation(
  //     Placement(visible = true, transformation(origin = {-82, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Modelica.Mechanics.MultiBody.Joints.FreeMotion freeMotion annotation(
  //     Placement(visible = true, transformation(origin = {-24, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(c = 100, d = 100)  annotation(
  //     Placement(visible = true, transformation(origin = {-12, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  // Modelica.Mechanics.MultiBody.Joints.UniversalSpherical link annotation(
  //     Placement(visible = true, transformation(origin = {-27, 61}, extent = {{-11, -11}, {11, 11}}, rotation = 0)));
  // equation
  //   connect(world.frame_b, fixedTranslation.frame_a) annotation(
  //     Line(points = {{-76, -74}, {-92, -74}, {-92, -4}}, color = {95, 95, 95}));
  // connect(fixedTranslation.frame_b, freeMotion.frame_a) annotation(
  //     Line(points = {{-72, -4}, {-34, -4}, {-34, -32}}, color = {95, 95, 95}));
  // connect(fixedTranslation.frame_b, springDamperParallel.frame_a) annotation(
  //     Line(points = {{-72, -4}, {-22, -4}, {-22, 10}}, color = {95, 95, 95}));
  // end CutPushRod;
end Suspensions;