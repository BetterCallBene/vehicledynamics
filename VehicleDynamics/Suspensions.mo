within VehicleDynamics;

package Suspensions "Suspensions, models ready to be used as front or rear suspensions."
  package ParameterSets "Sub package for all parameter sets for suspension implementations"
    extends Modelica.Icons.Library2;

    package Components "Parameter for components of suspension"

      record Strut
      extends Modelica.Icons.Record;
        import SI = Modelica.SIunits;
        // strut
        parameter SI.Position[3] rUW = {-2.557,0.7373,0.0290};
        parameter SI.Position[3] rCS={-2.55 - 0.0621,0.518,0.450} 
        "|Geometry|Strut to chassis/body";
        parameter SI.Position[3] rUS={-2.55 - 0.063305,0.581,-0.046} 
        "|Geometry|Strut ot upright";
        parameter SI.Position q0S=0.116 
          "|Geometry|Additional length for unloaded strut";
        parameter SI.TranslationalSpringConstant c = 20000.0;
        parameter SI.TranslationalDampingConstant d = 1200.0;
        
        SI.Position[3]  rA  = rUS - rUW;
        SI.Length       s0 = Modelica.Math.Vectors.length(rCS - rUS) + q0S;
        SI.Position[3]  rB = -rCS;
      end Strut;

      record MacPherson "Parameter set for MacPherson"
        import SI = Modelica.SIunits;
        extends Modelica.Icons.Record;
        parameter SI.Position[3] rCL1 = {0.1070, 0.55, -0.0380} "|Geometry| Vector from origin of frame_C to front link mount in chassis resolved in frame_C (at initial time)";
        parameter SI.Position[3] rCL2 = {-0.3070, 0.55, -0.0380} "|Geometry| Vector from origin of frame_C to rear link mount in chassis resolved in frame_C (at initial time)";
        parameter SI.Position[3] rCS = {-0.0295, 0.850, 0.5670} "|Geometry| Vector from origin of frame_C to strut mount in chassis resolved in frame_C (at initial time)";
        parameter SI.Position[3] rUS = {-0.0070, 0.8733, 0.1380} "|Geometry| Vector from origin of frame_C to strut mount in uppright resolved in frame_C (at initial time)";
        parameter SI.Position[3] rUL1L2 = {-0.0070, 0.8733, -0.0380} "|Geometry| Vector from origin of frame_C to low spindleUtilities.Joints.Joint resolved in frame_C (at initial time)";
        parameter SI.Position[3] rUW = {0, 0.9, 0} "|Geometry| Vector from origin of frame_C to wheel centre resolved in frame_C (at initial time)";
        parameter SI.Position[3] rRL3 = {-0.12, 0.85, 0.08} "|Geometry| Vector from origin of frame_C to origin of frame_S resolved in frame_C (at initial time)";
        parameter SI.Position[3] rUL3 = {0, 0.9, 0} "|Geometry| Vector from origin of frame_C to lower ball of steering rod resolved in frame_C (at initial time)";
        parameter SI.Length q0S = 0.05 "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
        parameter SI.Position rRL[3] = {-0.16, 0.54, 0}  "|Geometry|Steering|";
        parameter SI.Length steeringRodLength = 0.54 "|Geometry|Length of the steering rod";
        parameter SI.TranslationalSpringConstant  fc = 20000.0;
        parameter SI.TranslationalDampingConstant fd = 1200.0;
      end MacPherson;

      record FiveLink "Parameter set for FiveLink"
        import SI = Modelica.SIunits;
        extends Modelica.Icons.Record;
        parameter SI.Position[3] rCL1 = {-0.5550,-0.3690,-0.0320} "|Geometry|Left fivelink|link 1 to chassis/body";
        parameter SI.Position[3] rCL2 = {-0.4240,-0.1900,0.1640} "|Geometry|Left fivelink|link 2 to chassis/body";
        parameter SI.Position[3] rCL3 = {-0.3320,-0.1300,0.0570} "|Geometry|Left fivelink|link 3 to chassis/body";
        parameter SI.Position[3] rCL4 = {-0.4240,-0.1900,0.2640} "|Geometry|Left fivelink|link 4 to chassis/body";
        parameter SI.Position[3] rCL5 = {-0.3320,-0.1300,0.1570} "|Geometry|Left fivelink|link 5 to chassis/body";
        parameter SI.Position[3] rUL1 = {-0.5250,0.0620,-0.0670} "|Geometry|Left fivelink|link 1 to upright";
        parameter SI.Position[3] rUL2 = {-0.4330,0.0680,0.1500} "|Geometry|Left fivelink|link 2 to upright";
        parameter SI.Position[3] rUL3 = {-0.3350,0.0116,0.0400} "|Geometry|Left fivelink|link 3 to upright";
        parameter SI.Position[3] rUL4 = {-0.4330,0.0680,0.2800} "|Geometry|Left fivelink|link 4 to upright";
        parameter SI.Position[3] rUL5 = {-0.3350,0.0116,0.1900} "|Geometry|Left fivelink|link 5 to upright";
        parameter SI.Position[3] rUW = {-0.4650,0.1330,0.0200} "|Geometry|Left fivelink|Wheel centre";
  parameter SI.Position[3] rCMU={0,0,0} "|Mass and Inertia|";
        parameter SI.Mass mU=1 "|Mass and Inertia|";
        parameter SI.Inertia i11U=0.1 "|Mass and Inertia|";
        parameter SI.Inertia i22U=0.1 "|Mass and Inertia|";
        parameter SI.Inertia i33U=0.1 "|Mass and Inertia|";
        parameter SI.Inertia i21U=0.01 "|Mass and Inertia|";
        parameter SI.Inertia i31U=0.01 "|Mass and Inertia|";
        parameter SI.Inertia i32U=0.01 "|Mass and Inertia|";

        // strut
        // parameter SI.Position[3] rCS = {(-2.55) - 0.0621, 0.518, 0.450} "|Geometry|Left strut|Strut to chassis/body";
        // parameter SI.Position[3] rUS = {(-2.55) - 0.063305, 0.581, -0.046} "|Geometry|Left strut|Strut ot upright";

      end FiveLink;
      
    end Components;

    record MacPherson
      extends Modelica.Icons.Record;
      replaceable parameter VehicleDynamics.Suspensions.ParameterSets.Components.MacPherson macPherson_LF(fc = 1200.0, fd = 20000.0,q0S = 3516 / 20000, rCL1 = {-0.0075, 0.3753, -0.0358}, rCL2 = {-0.3175, 0.3535, -0.02735}, rCS = {-0.0291, 0.541, 0.561}, rRL = {-0.164, 0.3047, 0.0945}, rRL3 = {-0.164, 0.3047, 0.0945}, rUL1L2 = {0.0058556, 0.69905, -0.0725}, rUL3 = {-0.1283, 0.643, 0.07344}, rUS = {-0.00012, 0.5791, 0.042}, rUW = {-0.00013, 0.725, 0.0450}, steeringRodLength = Modelica.Math.Vectors.length(macPherson_LF.rRL - (macPherson_LF.rRL + macPherson_RF.rRL)/2))  annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    replaceable parameter VehicleDynamics.Suspensions.ParameterSets.Components.MacPherson macPherson_RF(fc = macPherson_LF.fc, fd = macPherson_LF.fd,q0S = macPherson_LF.q0S, rCL1 = {1, -1, 1} .* macPherson_LF.rCL1, rCL2 = {1, -1, 1} .* macPherson_LF.rCL2, rCS = {1, -1, 1} .* macPherson_LF.rCS, rRL = {1, -1, 1} .* macPherson_LF.rRL, rRL3 = {1, -1, 1} .* macPherson_LF.rRL3, rUL1L2 = {1, -1, 1} .* macPherson_LF.rUL1L2, rUL3 = {1, -1, 1} .* macPherson_LF.rUL3, rUS = {1, -1, 1} .* macPherson_LF.rUS, rUW = {1, -1, 1} .* macPherson_LF.rUW, steeringRodLength = macPherson_LF.steeringRodLength) annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  replaceable parameter VehicleDynamics.Steering.ParameterSets.RackSteering rackSteering annotation(
        Placement(visible = true, transformation(origin = {0, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      
    end MacPherson;

    record FiveLink
      extends Modelica.Icons.Record;
      replaceable parameter VehicleDynamics.Suspensions.ParameterSets.Components.FiveLink fiveLink_RL(i11U = 0.0929, i21U = -0.03, i22U = 0.287, i31U = 0.12, i32U = 0.01, i33U = 0.293, mU = 11.0, rCL1 = {-0.13347273, 0.231, -0.022}, rCL2 = {0.00562727, 0.4150, 0.174}, rCL3 = {0.04782727, 0.3750, 0.0637}, rCL4 = {-0.15217273, 0.2950, 0.1637}, rCL5 = {0.34782727, 0.4750, -0.0637}, rCMU = {0.118, 0.028, 0.0776}, rUL1 = {-0.10747273, 0.6672, -0.05327}, rUL2 = {-0.00827273, 0.6678, 0.1607}, rUL3 = {0.08252727, 0.612, 0.050}, rUL4 = {-0.11747273, 0.612, 0.150}, rUL5 = {0.08252727, 0.612, -0.010}, rUW = {-0.04747273, 0.7373, 0.0290})  annotation(
        Placement(visible = true, transformation(origin = {-70, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  replaceable parameter VehicleDynamics.Suspensions.ParameterSets.Components.FiveLink fiveLink_RR(i11U = fiveLink_RL.i11U, i21U = fiveLink_RL.i21U, i22U = fiveLink_RL.i22U, i31U = fiveLink_RL.i31U, i33U = fiveLink_RL.i33U, mU = fiveLink_RL.mU, rCL1 = {1, -1, 1} .* fiveLink_RL.rCL1, rCL2 = {1, -1, 1} .* fiveLink_RL.rCL2, rCL3 = {1, -1, 1} .* fiveLink_RL.rCL3, rCL4 = {1, -1, 1} .* fiveLink_RL.rCL4, rCL5 = {1, -1, 1} .* fiveLink_RL.rCL5, rCMU = {1, -1, 1} .* fiveLink_RL.rCMU, rUL1 = {1, -1, 1} .* fiveLink_RL.rUL1, rUL2 = {1, -1, 1} .* fiveLink_RL.rUL2, rUL3 = {1, -1, 1} .* fiveLink_RL.rUL3, rUL4 = {1, -1, 1} .* fiveLink_RL.rUL4, rUL5 = {1, -1, 1} .* fiveLink_RL.rUL5, rUW = {1, -1, 1} .* fiveLink_RL.rUW)  annotation(
        Placement(visible = true, transformation(origin = {50, 76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  replaceable parameter Components.Strut strut_LR(q0S = 0.116, rCS = {- 0.0621, 0.518, 0.450}, rUS = {- 0.063305, 0.581, -0.046}, rUW = fiveLink_RL.rUW)  annotation(
        Placement(visible = true, transformation(origin = {-68, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  replaceable parameter VehicleDynamics.Suspensions.ParameterSets.Components.Strut strut_RR(q0S = strut_LR.q0S, rCS = {1, -1, 1} .* strut_LR.rCS, rUS = {1, -1, 1} .* strut_LR.rUS, rUW = fiveLink_RR.rUW)  annotation(
        Placement(visible = true, transformation(origin = {50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));      
    
    end FiveLink;


  end ParameterSets;

  model FiveLink "Description"
    extends Interfaces.Suspension;
    VehicleDynamics.Suspensions.Components.FiveLink fiveLink_LR(fiveLinkData = fiveLinkData.fiveLink_RL)  annotation(
      Placement(visible = true, transformation(origin = {-50, 42}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  VehicleDynamics.Suspensions.Components.FiveLink fiveLink_RR(fiveLinkData = fiveLinkData.fiveLink_RR)  annotation(
      Placement(visible = true, transformation(origin = {50, 42}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  VehicleDynamics.Suspensions.Components.Strut strut_LR(strutData = fiveLinkData.strut_LR)  annotation(
      Placement(visible = true, transformation(origin = {-16, 28}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  VehicleDynamics.Suspensions.Components.Strut strut_RR(strutData = fiveLinkData.strut_RR)  annotation(
      Placement(visible = true, transformation(origin = {16, 28}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  replaceable parameter VehicleDynamics.Suspensions.ParameterSets.FiveLink fiveLinkData annotation(
      Placement(visible = true, transformation(origin = {-84, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
  connect(frame_C, fiveLink_LR.frame_C) annotation(
      Line(points = {{0, 100}, {-50, 100}, {-50, 52}, {-50, 52}}));
  connect(frame_C, fiveLink_RR.frame_C) annotation(
      Line(points = {{0, 100}, {50, 100}, {50, 52}}));
  connect(fiveLink_LR.frame_U, frame_L) annotation(
      Line(points = {{-50, 32}, {-100, 32}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(fiveLink_RR.frame_U, frame_R) annotation(
      Line(points = {{50, 32}, {100, 32}, {100, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(frame_L, strut_LR.frame_a) annotation(
      Line(points = {{-100, 0}, {-16, 0}, {-16, 18}, {-16, 18}}));
  connect(strut_LR.frame_b, frame_C) annotation(
      Line(points = {{-16, 38}, {0, 38}, {0, 100}, {0, 100}}));
  connect(frame_R, strut_RR.frame_a) annotation(
      Line(points = {{100, 0}, {16, 0}, {16, 18}}));
  connect(strut_RR.frame_b, frame_C) annotation(
      Line(points = {{16, 38}, {0, 38}, {0, 100}, {0, 100}}, color = {95, 95, 95}));
  end FiveLink;

  model MacPherson "Description"
    VehicleDynamics.Suspensions.ParameterSets.MacPherson macPhersonData annotation(
      Placement(visible = true, transformation(origin = {-76, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VehicleDynamics.Suspensions.Components.MacPherson macPherson_LF(macPhersonData = macPhersonData.macPherson_LF)  annotation(
      Placement(visible = true, transformation(origin = {-48, 20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  VehicleDynamics.Steering.RackSteering rackSteering(rRL_LF = macPhersonData.macPherson_LF.rRL, rRL_RF = macPhersonData.macPherson_RF.rRL, rackSteeringData = macPhersonData.rackSteering)  annotation(
      Placement(visible = true, transformation(origin = {0, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_Wheel_L annotation(
      Placement(visible = true, transformation(origin = {-100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_C annotation(
      Placement(visible = true, transformation(origin = {0, -98}, extent = {{-16, -16}, {16, 16}}, rotation = 90), iconTransformation(origin = {0, -98}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
  VehicleDynamics.Suspensions.Components.MacPherson macPherson_RF(leftMacPherson=false, macPhersonData = macPhersonData.macPherson_RF)  annotation(
      Placement(visible = true, transformation(origin = {52, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_Wheel_R annotation(
      Placement(visible = true, transformation(origin = {100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  equation
    connect(rackSteering.frame_C, frame_C) annotation(
      Line(points = {{0, 40}, {0, -98}}));
    connect(macPherson_LF.frame_U, frame_Wheel_L) annotation(
      Line(points = {{-58, 20}, {-100, 20}}, color = {95, 95, 95}));
    connect(macPherson_LF.frame_C, frame_C) annotation(
      Line(points = {{-38, 20}, {0, 20}, {0, -98}, {0, -98}}));
    connect(macPherson_RF.frame_C, frame_C) annotation(
      Line(points = {{42, 20}, {0, 20}, {0, -98}}, color = {95, 95, 95}));
    connect(macPherson_RF.frame_U, frame_Wheel_R) annotation(
      Line(points = {{62, 20}, {100, 20}}, color = {95, 95, 95}));
    connect(rackSteering.frame_X_LF, macPherson_LF.frame_S) annotation(
      Line(points = {{-10, 56}, {-48, 56}, {-48, 30}, {-48, 30}}, color = {95, 95, 95}));
    connect(rackSteering.frame_X_RF, macPherson_RF.frame_S) annotation(
      Line(points = {{10, 56}, {52, 56}, {52, 30}, {52, 30}}));
  end MacPherson;

  package Components "Components for suspension"
    model CutLink "Two bushings connected by a massless rod"
      import SI = Modelica.SIunits;
      extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
      
      parameter Boolean animation=true "= true, if animation shall be enabled";
      parameter Modelica.Mechanics.MultiBody.Types.Axis n1_a = {0, 0, 1};
      parameter SI.Position[3] rA = {0, 0, 0} "Position vector from frame_a to center of bushingA, resolved in frame_a";
      parameter SI.Position[3] rB = {0, 0, 0} "Position vector from center of bushingb to frame_b, resolved in frame_b";
      parameter SI.Position rRod_ia[3] = {1, 0, 0};
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotA(animation = false, r = rA) annotation(
        Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotB(r = -rB, animation=animation) annotation(
        Placement(visible = true, transformation(origin = {68, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Joints.UniversalSpherical joint(n1_a = n1_a, rRod_ia = rRod_ia, animation=animation) annotation(
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
      parameter Boolean animation=true "= true, if animation shall be enabled";
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_ia annotation(
        Placement(visible = true, transformation(origin = {-44, 100}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {-44, 100}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      parameter Modelica.Mechanics.MultiBody.Types.Axis n1_a = {0, 0, 1};
      parameter SI.Position rA[3] = {0, 0, 0};
      parameter SI.Position rB[3] = {0, 0, 0};
      parameter SI.Position rRod_ia[3] = {1, 0, 0};
      Modelica.Mechanics.MultiBody.Joints.Spherical jointB(animation=animation) annotation(
        Placement(visible = true, transformation(origin = {22, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Joints.Universal jointA(animation=animation, n_b = cross(n1_a, rRod_ia), n_a = cross(rRod_ia, jointA.n_b)) annotation(
        Placement(visible = true, transformation(origin = {-54, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation rod(animation=animation, r = rRod_ia) annotation(
        Placement(visible = true, transformation(origin = {-14, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotA(animation = false, r = rA) annotation(
        Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotB(animation=animation, r = -rB) annotation(
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

    model MacPherson
    import SI = Modelica.SIunits;
    import Math = Modelica.Math;
    import To_deg = Modelica.SIunits.Conversions.to_deg;
    extends Modelica.Icons.Package;
    extends Interfaces.Linkages;
    parameter Boolean leftMacPherson = true "true, if left MacPherson suspension, otherwise right suspension";

    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_S annotation(
       Placement(visible = true, transformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation upper(animation = false, r = macPhersonData.rCS) annotation(
      Placement(visible = true, transformation(origin = {-78, 48}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Joints.Revolute innerJoint(animation = true, n = macPhersonData.rCL1 - macPhersonData.rCL2) annotation(
      Placement(visible = true, transformation(origin = {-52, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower(animation = true, r = macPhersonData.rCL1) annotation(
      Placement(visible = true, transformation(origin = {-78, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation frontBar(animation = true, r = macPhersonData.rUL1L2 - macPhersonData.rCL1) annotation(
      Placement(visible = true, transformation(origin = {-24, -82}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation rearBar(animation = true, r = macPhersonData.rCL2 - macPhersonData.rUL1L2) annotation(
      Placement(visible = true, transformation(origin = {-32, -56}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel struct(animation = true, c = macPhersonData.fc, d = macPhersonData.fd, s_unstretched = q0Strut) annotation(
      Placement(visible = true, transformation(origin = {0, 34}, extent = {{10, 10}, {-10, -10}}, rotation = 180)));
    Modelica.Mechanics.MultiBody.Joints.Assemblies.JointSSR jointSSR(n_b = if leftMacPherson then rS else -rS, rRod2_ib = macPhersonData.rRL - macPhersonData.rUL1L2, rod1Length = macPhersonData.steeringRodLength) annotation(
      Placement(visible = true, transformation(origin = {54, 46}, extent = {{-20, 20}, {20, -20}}, rotation = -90)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation springRod(animation = true, r = macPhersonData.rUS - macPhersonData.rUL1L2) annotation(
      Placement(visible = true, transformation(origin = {17, -5}, extent = {{-17, -17}, {17, 17}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Joints.Assemblies.JointUPS MacPherson(animation = false,n1_a = n_a, nAxis_ia = {0, 0, 1}) annotation(
      Placement(visible = true, transformation(origin = {-44, 18}, extent = {{20, -20}, {-20, 20}}, rotation = -90)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation outerRod(r = macPhersonData.rUW - macPhersonData.rUL1L2) annotation(
      Placement(visible = true, transformation(origin = {44, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  replaceable parameter ParameterSets.Components.MacPherson macPhersonData annotation(
      Placement(visible = true, transformation(origin = {-76, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    parameter Real angleBetweenBreaksAndSuspension = To_deg(Math.asin(Math.Vectors.norm(cross(rS, {0, 0, 1}))/ Math.Vectors.norm(rS)));
  Modelica.Mechanics.MultiBody.Parts.FixedRotation rotRod(angle = if leftMacPherson then -angleBetweenBreaksAndSuspension else angleBetweenBreaksAndSuspension, n = {1, 0, 0}, rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.RotationAxis) annotation(
        Placement(visible = true, transformation(origin = {74, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    parameter SI.Position rS[3] = macPhersonData.rCS - macPhersonData.rUL1L2;
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation debug(r = macPhersonData.rUW - macPhersonData.rCS)  annotation(
        Placement(visible = true, transformation(origin = {-68, 22}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    protected
    parameter SI.Length q0Strut = Modelica.Math.Vectors.length(macPhersonData.rCS - macPhersonData.rUS) + macPhersonData.q0S;
    parameter SI.Position rU[3] = macPhersonData.rUW - macPhersonData.rUL1L2 "Position vector from frame_L1L2 to frame_U, resolved in frame_C";
    parameter Real n_a[3] = cross(rS, rU) "First rotation axis of universalUtilities.Joints.Joint in springJoint, resolved in frame_C";
  equation
      connect(lower.frame_b, innerJoint.frame_a) annotation(
        Line(points = {{-68, -82}, {-62, -82}}, color = {95, 95, 95}));
      connect(innerJoint.frame_b, frontBar.frame_a) annotation(
        Line(points = {{-42, -82}, {-34, -82}}, color = {95, 95, 95}));
      connect(frontBar.frame_b, rearBar.frame_a) annotation(
        Line(points = {{-14, -82}, {-14, -68}, {-22, -68}, {-22, -56}}));
      connect(upper.frame_b, MacPherson.frame_b) annotation(
        Line(points = {{-68, 48}, {-44, 48}, {-44, 38}}, color = {95, 95, 95}));
      connect(frontBar.frame_b, MacPherson.frame_a) annotation(
        Line(points = {{-14, -82}, {-12, -82}, {-12, -38}, {-44, -38}, {-44, -2}}, color = {95, 95, 95}));
      connect(frame_C, upper.frame_a) annotation(
        Line(points = {{-100, 0}, {-88, 0}, {-88, 48}}));
      connect(frame_C, lower.frame_a) annotation(
        Line(points = {{-100, 0}, {-88, 0}, {-88, -82}}));
      connect(MacPherson.frame_ib, struct.frame_a) annotation(
        Line(points = {{-24, 34}, {-10, 34}}, color = {95, 95, 95}));
      connect(struct.frame_b, springRod.frame_b) annotation(
        Line(points = {{10, 34}, {17, 34}, {17, 12}}));
      connect(MacPherson.frame_ia, jointSSR.frame_b) annotation(
        Line(points = {{-24, 2}, {-24, -30}, {54, -30}, {54, 26}}));
      connect(springRod.frame_a, jointSSR.frame_ib) annotation(
        Line(points = {{17, -22}, {17, -50}, {34, -50}, {34, 4}, {34, 30}}, color = {95, 95, 95}));
      connect(jointSSR.frame_a, frame_S) annotation(
        Line(points = {{54, 66}, {0, 66}, {0, 100}, {0, 100}}, color = {95, 95, 95}));
  connect(rotRod.frame_b, frame_U) annotation(
        Line(points = {{84, -80}, {84, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(springRod.frame_a, outerRod.frame_a) annotation(
        Line(points = {{36, -64}, {35.875, -64}, {35.875, -42}, {27.75, -42}, {27.75, -54}, {25.5, -54}, {25.5, -66}, {35, -66}, {35, -80}, {34, -80}}, color = {95, 95, 95}));
  connect(outerRod.frame_b, rotRod.frame_a) annotation(
        Line(points = {{54, -80}, {64, -80}}, color = {95, 95, 95}));
  connect(upper.frame_b, debug.frame_a) annotation(
        Line(points = {{-68, 48}, {-68, 32}}, color = {95, 95, 95}));
    end MacPherson;
  model FiveLink "Five links constrain the motion of the upright and reduces the degrees of freedom to one."
    import SI = Modelica.SIunits;
    extends Interfaces.Linkages;
    //replaceable ParameterSets.FiveLink fiveLinkData;
    
    //fivelink
    VehicleDynamics.Suspensions.Components.Link link1(animation = true,rA = fiveLinkData.rCL1, rB = fiveLinkData.rUW - fiveLinkData.rUL1, rRod_ia = fiveLinkData.rUL1 - fiveLinkData.rCL1) annotation(
      Placement(visible = true, transformation(origin = {-16, 56}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.Components.CutLink link2(animation = true,rA = fiveLinkData.rCL2, rB = fiveLinkData.rUW - fiveLinkData.rUL2, rRod_ia = fiveLinkData.rUL2 - fiveLinkData.rCL2) annotation(
      Placement(visible = true, transformation(origin = {-16, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.Components.CutLink link3(animation = true,rA = fiveLinkData.rCL3, rB = fiveLinkData.rUW - fiveLinkData.rUL3, rRod_ia = fiveLinkData.rUL3 - fiveLinkData.rCL3) annotation(
      Placement(visible = true, transformation(origin = {-16, -6}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.Components.CutLink link4(animation = true,rA = fiveLinkData.rCL4, rB = fiveLinkData.rUW - fiveLinkData.rUL4, rRod_ia = fiveLinkData.rUL4 - fiveLinkData.rCL4) annotation(
      Placement(visible = true, transformation(origin = {-18, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.Components.CutLink link5(animation = true,rA = fiveLinkData.rCL5, rB = fiveLinkData.rUW - fiveLinkData.rUL5, rRod_ia = fiveLinkData.rUL5 - fiveLinkData.rCL5) annotation(
      Placement(visible = true, transformation(origin = {-18, -66}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.Body U(I_11 = fiveLinkData.i11U, I_21 = fiveLinkData.i21U, I_22 = fiveLinkData.i22U, I_31 = fiveLinkData.i31U, I_32 = fiveLinkData.i32U, I_33 = fiveLinkData.i33U, animation = true, m = fiveLinkData.mU, r_CM = fiveLinkData.rCMU)  annotation(
      Placement(visible = true, transformation(origin = {80, 72}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  replaceable parameter VehicleDynamics.Suspensions.ParameterSets.Components.FiveLink fiveLinkData annotation(
      Placement(visible = true, transformation(origin = {-74, 72}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  protected
  equation
    connect(link1.frame_a, frame_C) annotation(
      Line(points = {{-26, 56}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(link2.frame_a, frame_C) annotation(
      Line(points = {{-26, 22}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(link3.frame_a, frame_C) annotation(
      Line(points = {{-26, -6}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(link4.frame_a, frame_C) annotation(
      Line(points = {{-28, -36}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(link5.frame_a, frame_C) annotation(
      Line(points = {{-28, -66}, {-100, 0}, {-100, 0}}, color = {95, 95, 95}));
    connect(link1.frame_b, frame_U) annotation(
      Line(points = {{-6, 56}, {100, 0}, {100, 0}}));
    connect(link2.frame_b, frame_U) annotation(
      Line(points = {{-6, 22}, {100, 0}, {100, 0}}, color = {95, 95, 95}));
    connect(link3.frame_b, frame_U) annotation(
      Line(points = {{-6, -6}, {100, 0}, {100, 0}}));
    connect(link4.frame_b, frame_U) annotation(
      Line(points = {{-8, -36}, {100, 0}, {100, 0}}));
    connect(link5.frame_b, frame_U) annotation(
      Line(points = {{-8, -66}, {100, 0}, {100, 0}}, color = {95, 95, 95}));
    connect(U.frame_a, frame_U) annotation(
        Line(points = {{90, 72}, {100, 72}, {100, 0}, {100, 0}}, color = {95, 95, 95}));
    annotation(
        Documentation(info = "<html><head></head><body><br></body></html>"));
    end FiveLink;

    model Strut "Strut - Integrated spring-damper unit"
      extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation frameTranslationA(r = strutData.rA)  annotation(
        Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslationB(animation = false, r = strutData.rB) annotation(
        Placement(visible = true, transformation(origin = {52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(c = strutData.c, d = strutData.d, s_unstretched = strutData.s0)  annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  replaceable parameter ParameterSets.Components.Strut strutData(c = 10, d = 20000)  annotation(
        Placement(visible = true, transformation(origin = {-72, 84}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(frameTranslationA.frame_b, springDamperParallel.frame_a) annotation(
        Line(points = {{-50, 0}, {-10, 0}, {-10, 0}, {-10, 0}}));
      connect(frameTranslationA.frame_a, frame_a) annotation(
        Line(points = {{-70, 0}, {-98, 0}, {-98, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(springDamperParallel.frame_b, fixedTranslationB.frame_a) annotation(
        Line(points = {{10, 0}, {42, 0}, {42, 0}, {42, 0}}, color = {95, 95, 95}));
  connect(fixedTranslationB.frame_b, frame_b) annotation(
        Line(points = {{62, 0}, {100, 0}, {100, 0}, {100, 0}}, color = {95, 95, 95}));
    end Strut;


  end Components;

  package Tests "Tests of suspensions"
    model MacPhersonBasic "Description"
      import SI = Modelica.SIunits;
      inner Modelica.Mechanics.MultiBody.World world annotation(
        Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter SI.Position rRL_1[3] = {-0.16, 0.54, 0} "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
      parameter SI.Position rRL_2[3] = {-0.16, -0.54, 0} "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
      VehicleDynamics.Suspensions.Components.MacPherson macPherson annotation(
        Placement(visible = true, transformation(origin = {8, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(animation = false, r = rRFrame) annotation(
        Placement(visible = true, transformation(origin = {-74, 22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Blocks.Sources.Ramp ramp(height = 0.1) annotation(
        Placement(visible = true, transformation(origin = {-88, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(animation = true, n = rRL_1 - rRL_2, useAxisFlange = true) annotation(
        Placement(visible = true, transformation(origin = {-38, 24}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.Translational.Sources.Position position annotation(
        Placement(visible = true, transformation(origin = {-42, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(animation = true, r = {0, 1, 0}) annotation(
        Placement(visible = true, transformation(origin = {46, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Wheels.RillTyre.Wheel wheel annotation(
        Placement(visible = true, transformation(origin = {88, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
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
        Line(points = {{-68, -76}, {-31, -76}, {-31, -72}, {-2, -72}, {-2, -22}}));
      connect(fixedTranslation.frame_b, prismatic.frame_a) annotation(
        Line(points = {{-64, 22}, {-48, 22}, {-48, 24}, {-48, 24}}, color = {95, 95, 95}));
      connect(prismatic.frame_b, macPherson.frame_S) annotation(
        Line(points = {{-28, 24}, {8, 24}, {8, -12}}, color = {95, 95, 95}));
      connect(macPherson.frame_U, fixedTranslation1.frame_a) annotation(
        Line(points = {{18, -22}, {36, -22}}));
  connect(fixedTranslation1.frame_b, wheel.carrierFrame) annotation(
        Line(points = {{56, -22}, {78, -22}, {78, -22}, {78, -22}}));
    end MacPhersonBasic;

    model FiveLinkBasic "Description"
      inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
        Placement(visible = true, transformation(origin = {-68, -32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.Body body(I_11 = 1, I_22 = 1, I_33 = 1, animation = false, m = 1, r_CM = {0, 0, 0}) annotation(
        Placement(visible = true, transformation(origin = {86, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Components.FiveLink fiveLink annotation(
        Placement(visible = true, transformation(origin = {-14, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(world.frame_b, fiveLink.frame_C) annotation(
        Line(points = {{-58, -32}, {-24, -32}, {-24, 10}, {-24, 10}}, color = {95, 95, 95}));
      connect(fiveLink.frame_U, body.frame_a) annotation(
        Line(points = {{-4, 10}, {72, 10}, {72, 2}, {76, 2}, {76, 2}}, color = {95, 95, 95}));
    end FiveLinkBasic;

    model MacPhersonBasic2
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
    end MacPhersonBasic2;

    model FiveLinkBasic2
      VehicleDynamics.Suspensions.FiveLink fiveLink annotation(
        Placement(visible = true, transformation(origin = {0, 10}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
      inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
        Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(animation = false, r = {0, 0, 1})  annotation(
        Placement(visible = true, transformation(origin = {-10, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VehicleDynamics.Wheels.RillTyre.Wheel wheel(leftWheel = false)  annotation(
        Placement(visible = true, transformation(origin = {64, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VehicleDynamics.Wheels.RillTyre.Wheel wheel1 annotation(
        Placement(visible = true, transformation(origin = {-52, 10}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  equation
      connect(world.frame_b, fixedTranslation.frame_a) annotation(
        Line(points = {{-68, -76}, {-20, -76}, {-20, -40}}, color = {95, 95, 95}));
      connect(fixedTranslation.frame_b, fiveLink.frame_C) annotation(
        Line(points = {{0, -40}, {0, 0}}, color = {95, 95, 95}));
  connect(fiveLink.frame_R, wheel.carrierFrame) annotation(
        Line(points = {{10, 10}, {54, 10}, {54, 10}, {54, 10}}));
  connect(fiveLink.frame_L, wheel1.carrierFrame) annotation(
        Line(points = {{-10, 10}, {-42, 10}, {-42, 10}, {-42, 10}}, color = {95, 95, 95}));
    end FiveLinkBasic2;
  end Tests;
end Suspensions;