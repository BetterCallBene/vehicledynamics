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
    extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
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
    VehicleDynamics.Suspensions.Components.Link link1(rA = rCL1, rB = rUW - rUL1, rRod_ia = rUL1 - rCL1) annotation(
      Placement(visible = true, transformation(origin = {-16, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.Components.CutLink link2(rA = rCL2, rB = rUW - rUL2, rRod_ia = rUL2 - rCL2) annotation(
      Placement(visible = true, transformation(origin = {-16, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.Components.CutLink link3(rA = rCL3, rB = rUW - rUL3, rRod_ia = rUL3 - rCL3) annotation(
      Placement(visible = true, transformation(origin = {-16, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.Components.CutLink link4(rA = rCL4, rB = rUW - rUL4, rRod_ia = rUL4 - rCL4) annotation(
      Placement(visible = true, transformation(origin = {-18, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.Components.CutLink link5(rA = rCL5, rB = rUW - rUL5, rRod_ia = rUL5 - rCL5) annotation(
      Placement(visible = true, transformation(origin = {-18, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(c = 1000, d = 100000, s_unstretched = Modelica.Math.Vectors.length(rCS - rUS) + 0.116) annotation(
      Placement(visible = true, transformation(origin = {-18, 86}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation frameTranslation(animation = false, r = rCS) annotation(
      Placement(visible = true, transformation(origin = {-52, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation frameTranslation2(r = rUS - rUW) annotation(
      Placement(visible = true, transformation(origin = {18, 86}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  protected
  equation
    connect(frameTranslation.frame_b, springDamperParallel.frame_b) annotation(
      Line(points = {{-28, 86}, {-42, 86}, {-42, 84}, {-42, 84}}, color = {95, 95, 95}));
    connect(springDamperParallel.frame_a, frameTranslation2.frame_b) annotation(
      Line(points = {{-8, 86}, {8, 86}, {8, 86}, {8, 86}}, color = {95, 95, 95}));
    connect(link1.frame_a, frame_a) annotation(
      Line(points = {{-26, 56}, {-100, 56}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(link2.frame_a, frame_a) annotation(
      Line(points = {{-26, 22}, {-100, 22}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(link3.frame_a, frame_a) annotation(
      Line(points = {{-26, -6}, {-100, -6}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(link4.frame_a, frame_a) annotation(
      Line(points = {{-28, -36}, {-100, -36}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(link5.frame_a, frame_a) annotation(
      Line(points = {{-28, -66}, {-100, -66}, {-100, 0}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(frameTranslation.frame_a, frame_a) annotation(
      Line(points = {{-62, 84}, {-100, 84}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(link1.frame_b, frame_b) annotation(
      Line(points = {{-6, 56}, {100, 56}, {100, 0}, {100, 0}}));
    connect(link2.frame_b, frame_b) annotation(
      Line(points = {{-6, 22}, {100, 22}, {100, 0}, {100, 0}}, color = {95, 95, 95}));
    connect(link3.frame_b, frame_b) annotation(
      Line(points = {{-6, -6}, {100, -6}, {100, 0}, {100, 0}}));
    connect(link4.frame_b, frame_b) annotation(
      Line(points = {{-8, -36}, {100, -36}, {100, 0}, {100, 0}}));
    connect(link5.frame_b, frame_b) annotation(
      Line(points = {{-8, -66}, {100, -66}, {100, 0}, {100, 0}}, color = {95, 95, 95}));
    connect(frameTranslation2.frame_a, frame_b) annotation(
      Line(points = {{28, 86}, {100, 86}, {100, 0}, {100, 0}}, color = {95, 95, 95}));
  end FiveLink;

  model MultiLink42 "MultiLink4 - Linkage with four links, one more than neccessary"
    import SI = Modelica.SIunits;
    extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
    parameter SI.Position[3] rCL1={-2.6430,0.231,-0.022} 
        "|Geometry|Left multilink|Lower link to chassis/body";
      parameter SI.Position[3] rCL2={-2.5039,0.4150,0.174} 
        "|Geometry|Left multilink|Upper link to chassis/body";
      parameter SI.Position[3] rCL3={-2.4617,0.4750,0.0637} 
        "|Geometry|Left multilink|Steering link to chassis/body";
      parameter SI.Position[3] rCU={-2.0825,0.6200,0.0105} 
        "|Geometry|Left multilink|Trailing arm to chassis/body";
      parameter SI.Position[3] rUL1={-2.6170,0.6672,-0.05327} 
        "|Geometry|Left multilink|Lower link to trailing arm";
      parameter SI.Position[3] rUL2={-2.5178,0.6678,0.1607} 
        "|Geometry|Left multilink|Upper link to trailing arm";
      parameter SI.Position[3] rUL3={-2.4270,0.612,0.050} 
        "|Geometry|Left multilink|Steering link to trailing arm";
      parameter SI.Position[3] rUW={-2.557,0.7373,0.0290} 
        "|Geometry|Left multilink|Wheel centre";
    parameter SI.Position[3] rCMU={0.118,0.028,0.0776};
    parameter SI.Mass mU = 11 "|Mass and Inertia|";
    parameter SI.Inertia i11U=0.0929 "|Mass and Inertia|left upright|";
      parameter SI.Inertia i22U=0.287 "|Mass and Inertia|left upright|";
      parameter SI.Inertia i33U=0.293 "|Mass and Inertia|left upright|";
      parameter SI.Inertia i21U=-0.03 "|Mass and Inertia|left upright|";
      parameter SI.Inertia i31U=0.12 "|Mass and Inertia|left upright|";
      parameter SI.Inertia i32U=0.01 "|Mass and Inertia|left upright|";
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation frameTranslationUW(animation = true, r = rUW - rCU) annotation(
      Placement(visible = true, transformation(origin = {32, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation frameTranslationCU(animation = false, r = rCU) annotation(
      Placement(visible = true, transformation(origin = {-70, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));  
    Modelica.Mechanics.MultiBody.Parts.Body U(I_11 = i11U, I_21 = i21U, I_22 = i22U, I_31 = i31U, I_32 = i32U, I_33 = i33U, m = mU, r_CM = {0, 0, 0}) annotation(
      Placement(visible = true, transformation(origin = {48, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Spherical spherical annotation(
      Placement(visible = true, transformation(origin = {-30, 60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Components.CutLink Link3(rA = rCL3, rB = rCU - rUL3, rRod_ia = rUL3 - rCL3) annotation(
      Placement(visible = true, transformation(origin = {-32, -2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.CutLink Link2(rA = rCL2, rB = rCU - rUL2, rRod_ia = rUL2 - rCL2) annotation(
      Placement(visible = true, transformation(origin = {-32, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Components.CutLink Link1(rA = rCL1, rB = rCU - rUL1, rRod_ia = rUL1 - rCL1) annotation(
      Placement(visible = true, transformation(origin = {-32, 44}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(frameTranslationUW.frame_b, frame_b) annotation(
      Line(points = {{42, 0}, {100, 0}, {100, 0}, {100, 0}}));
    connect(frame_a, frameTranslationCU.frame_a) annotation(
      Line(points = {{-100, 0}, {-80, 0}, {-80, 60}}));
    connect(frameTranslationCU.frame_b, spherical.frame_b) annotation(
      Line(points = {{-60, 60}, {-40, 60}, {-40, 60}, {-40, 60}}, color = {95, 95, 95}));
    connect(spherical.frame_a, U.frame_a) annotation(
      Line(points = {{-20, 60}, {38, 60}, {38, 76}, {38, 76}}, color = {95, 95, 95}));
    connect(spherical.frame_a, frameTranslationUW.frame_a) annotation(
      Line(points = {{-20, 60}, {22, 60}, {22, 36}, {22, 36}, {22, 0}, {22, 0}}, color = {95, 95, 95}));
    connect(Link3.frame_b, spherical.frame_a) annotation(
      Line(points = {{-22, -2}, {-20, -2}, {-20, 60}, {-20, 60}}, color = {95, 95, 95}));
    connect(frame_a, Link3.frame_a) annotation(
      Line(points = {{-100, 0}, {-42, 0}, {-42, -2}, {-42, -2}}));
    connect(Link2.frame_b, spherical.frame_a) annotation(
      Line(points = {{-22, 20}, {-20, 20}, {-20, 60}, {-20, 60}}));
    connect(frame_a, Link2.frame_a) annotation(
      Line(points = {{-100, 0}, {-42, 0}, {-42, 20}, {-42, 20}}));
    connect(Link1.frame_b, spherical.frame_a) annotation(
      Line(points = {{-22, 44}, {-20, 44}, {-20, 60}, {-20, 60}}, color = {95, 95, 95}));
    connect(frame_a, Link1.frame_a) annotation(
      Line(points = {{-100, 0}, {-42, 0}, {-42, 44}, {-42, 44}}));
  end MultiLink42;

  package Components "Components for suspension"
    model CutLink "Two bushings connected by a massless rod"
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
      Modelica.Mechanics.MultiBody.Joints.UniversalSpherical joint(n1_a = n1_a, rRod_ia = rRod_ia) annotation(
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
    end CutLink;

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
  end Components;

  package Tests "Tests of suspensions"
    model MacPhersonBasic "Description"
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
    end MacPhersonBasic;

    model FiveLinkBasic "Description"
      inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
        Placement(visible = true, transformation(origin = {-68, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.Body body(I_11 = 1, I_22 = 1, I_33 = 1, m = 25, r_CM = {0, 0, 0}) annotation(
        Placement(visible = true, transformation(origin = {86, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      FiveLink fiveLink annotation(
        Placement(visible = true, transformation(origin = {-14, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(world.frame_b, fiveLink.frame_a) annotation(
        Line(points = {{-58, -32}, {-24, -32}, {-24, 10}, {-24, 10}}, color = {95, 95, 95}));
      connect(fiveLink.frame_b, body.frame_a) annotation(
        Line(points = {{-4, 10}, {72, 10}, {72, 2}, {76, 2}, {76, 2}}, color = {95, 95, 95}));
    end FiveLinkBasic;

    model MultiLink42Basic "Description"
      import SI = Modelica.SIunits;
      MultiLink42 multiLink42 annotation(
        Placement(visible = true, transformation(origin = {-4, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
        Placement(visible = true, transformation(origin = {-78, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation spring(r = rCS - rUS)  annotation(
        Placement(visible = true, transformation(origin = {32, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body body(m = 100, r_CM = {0, 0, 0})  annotation(
        Placement(visible = true, transformation(origin = {64, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  parameter SI.Position[3] rCS={-2.55 - 0.0621,0.518,0.450} 
            "|Geometry|Left strut|Strut to chassis/body";
          parameter SI.Position[3] rUS={-2.55 - 0.063305,0.581,-0.046} 
            "|Geometry|Left strut|Strut ot trailing arm";
    equation
      connect(world.frame_b, multiLink42.frame_a) annotation(
        Line(points = {{-68, -36}, {-14, -36}, {-14, 8}, {-14, 8}}));
  connect(multiLink42.frame_b, spring.frame_a) annotation(
        Line(points = {{6, 8}, {22, 8}, {22, 8}, {22, 8}}, color = {95, 95, 95}));
  connect(spring.frame_b, body.frame_a) annotation(
        Line(points = {{42, 8}, {54, 8}, {54, 8}, {54, 8}}));
    end MultiLink42Basic;
  end Tests;
end Suspensions;