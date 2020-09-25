within VehicleDynamics;

package Suspensions "Suspensions, models ready to be used as front or rear suspensions."
  package ParameterSets "Sub package for all parameter sets for suspension implementations"
    extends Modelica.Icons.Library2;

    package Components "Parameter for components of suspension"
      record Strut
        extends Modelica.Icons.Record;
        import SI = Modelica.SIunits;
        // strut
        parameter SI.Position[3] rUW = {-2.557, 0.7373, 0.0290};
        parameter SI.Position[3] rCS = {(-2.55) - 0.0621, 0.518, 0.450} "|Geometry|Strut to chassis/body";
        parameter SI.Position[3] rUS = {(-2.55) - 0.063305, 0.581, -0.046} "|Geometry|Strut ot upright";
        parameter SI.Position q0S = 0.116 "|Geometry|Additional length for unloaded strut";
        parameter SI.TranslationalSpringConstant c = 20000.0;
        parameter SI.TranslationalDampingConstant d = 1200.0;
        SI.Position[3] rA = rUS - rUW;
        SI.Length s0 = Modelica.Math.Vectors.length(rCS - rUS) + q0S;
        SI.Position[3] rB = -rCS;
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
        parameter SI.Position rRL[3] = {-0.16, 0.54, 0} "|Geometry|Steering|";
        parameter SI.Length steeringRodLength = 0.54 "|Geometry|Length of the steering rod";
        parameter SI.TranslationalSpringConstant fc = 20000.0;
        parameter SI.TranslationalDampingConstant fd = 1200.0;
      end MacPherson;

      record SimpleSuspension "Parameter set for simple suspension"
        extends Modelica.Icons.Record;
        import SI = Modelica.SIunits;
        parameter Boolean left = true "if true spring is placed on left side";
        parameter SI.Position scalingFactor[3] = if left then {1, 1, 1} else {1, -1, 1};
        parameter SI.Position rCU[3] = {-2.0, 0.6, 0.5} .* scalingFactor;
        parameter SI.Position rUW[3] = {0, 0.10, 0} .* scalingFactor;
        parameter Modelica.SIunits.TranslationalDampingConstant d = 10000 "Damping constant for suspension struct";
        parameter Modelica.SIunits.TranslationalSpringConstant c = 100 "Spring constant for suspension struct";
        parameter Modelica.Mechanics.MultiBody.Types.Axis prism = {0, 0, -1} "Axis of prismatic struct";
      end SimpleSuspension;
    end Components;

    record MacPherson
      extends Modelica.Icons.Record;
      replaceable parameter VehicleDynamics.Suspensions.ParameterSets.Components.MacPherson macPhersonData_LF(fc = 1200.0, fd = 20000.0, q0S = 3516 / 20000, rCL1 = {-0.0075, 0.3753, -0.0358}, rCL2 = {-0.3175, 0.3535, -0.02735}, rCS = {-0.0291, 0.541, 0.561}, rRL = {-0.164, 0.3047, 0.0945}, rRL3 = {-0.164, 0.3047, 0.0945}, rUL1L2 = {0.0058556, 0.69905, -0.0725}, rUL3 = {-0.1283, 0.643, 0.07344}, rUS = {-0.00012, 0.5791, 0.042}, rUW = {-0.00013, 0.725, 0.0450}, steeringRodLength = Modelica.Math.Vectors.length(macPherson_LF.rRL - (macPherson_LF.rRL + {1, -1, 1} .* macPherson_LF.rRL) / 2)) annotation(
        Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      replaceable parameter VehicleDynamics.Suspensions.ParameterSets.Components.MacPherson macPhersonData_RF(fc = macPherson_LF.fc, fd = macPherson_LF.fd, q0S = macPherson_LF.q0S, rCL1 = {1, -1, 1} .* macPherson_LF.rCL1, rCL2 = {1, -1, 1} .* macPherson_LF.rCL2, rCS = {1, -1, 1} .* macPherson_LF.rCS, rRL = {1, -1, 1} .* macPherson_LF.rRL, rRL3 = {1, -1, 1} .* macPherson_LF.rRL3, rUL1L2 = {1, -1, 1} .* macPherson_LF.rUL1L2, rUL3 = {1, -1, 1} .* macPherson_LF.rUL3, rUS = {1, -1, 1} .* macPherson_LF.rUS, rUW = {1, -1, 1} .* macPherson_LF.rUW, steeringRodLength = macPherson_LF.steeringRodLength) annotation(
        Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      replaceable parameter VehicleDynamics.Steering.ParameterSets.RackSteering rackSteeringData annotation(
        Placement(visible = true, transformation(origin = {0, 32}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation

    end MacPherson;
  end ParameterSets;

  model MacPherson "Description"
    replaceable parameter ParameterSets.Components.MacPherson macPhersonData_LF(fc = 1200.0, fd = 2000000.0, q0S = 3516 / 20000, rCL1 = {-0.0075, 0.3753, -0.0358}, rCL2 = {-0.3175, 0.3535, -0.02735}, rCS = {-0.0291, 0.541, 0.561}, rRL = {-0.164, 0.3047, 0.0945}, rRL3 = {-0.164, 0.3047, 0.0945}, rUL1L2 = {0.0058556, 0.69905, -0.0725}, rUL3 = {-0.1283, 0.643, 0.07344}, rUS = {-0.00012, 0.5791, 0.042}, rUW = {-0.00013, 0.725, 0.0450}, steeringRodLength = Modelica.Math.Vectors.length(macPhersonData_LF.rRL - (macPhersonData_LF.rRL + {1, -1, 1} .* macPhersonData_LF.rRL) / 2) - 0.0035) annotation(
      Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    replaceable parameter ParameterSets.Components.MacPherson macPhersonData_RF(fc = macPhersonData_LF.fc, fd = macPhersonData_LF.fd, q0S = macPhersonData_LF.q0S, rCL1 = {1, -1, 1} .* macPhersonData_LF.rCL1, rCL2 = {1, -1, 1} .* macPhersonData_LF.rCL2, rCS = {1, -1, 1} .* macPhersonData_LF.rCS, rRL = {1, -1, 1} .* macPhersonData_LF.rRL, rRL3 = {1, -1, 1} .* macPhersonData_LF.rRL3, rUL1L2 = {1, -1, 1} .* macPhersonData_LF.rUL1L2, rUL3 = {1, -1, 1} .* macPhersonData_LF.rUL3, rUS = {1, -1, 1} .* macPhersonData_LF.rUS, rUW = {1, -1, 1} .* macPhersonData_LF.rUW, steeringRodLength = macPhersonData_LF.steeringRodLength) annotation(
      Placement(visible = true, transformation(origin = {70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    replaceable parameter VehicleDynamics.Steering.ParameterSets.RackSteering rackSteeringData annotation(
      Placement(visible = true, transformation(origin = {0, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.Components.MacPherson macPherson_LF(macPhersonData = macPhersonData_LF) annotation(
      Placement(visible = true, transformation(origin = {-48, 20}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    VehicleDynamics.Steering.RackSteering rackSteering(rRL_LF = macPhersonData_LF.rRL, rRL_RF = macPhersonData_RF.rRL, rackSteeringData = rackSteeringData) annotation(
      Placement(visible = true, transformation(origin = {0, 46}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_Wheel_L annotation(
      Placement(visible = true, transformation(origin = {-100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_C annotation(
      Placement(visible = true, transformation(origin = {0, -98}, extent = {{-16, -16}, {16, 16}}, rotation = 90), iconTransformation(origin = {0, -98}, extent = {{-16, -16}, {16, 16}}, rotation = 90)));
    VehicleDynamics.Suspensions.Components.MacPherson macPherson_RF(leftMacPherson = false, macPhersonData = macPhersonData_RF) annotation(
      Placement(visible = true, transformation(origin = {52, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_Wheel_R annotation(
      Placement(visible = true, transformation(origin = {100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 20}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  equation
    connect(rackSteering.frame_C, frame_C) annotation(
      Line(points = {{0, 36}, {0, -98}}));
    connect(macPherson_LF.frame_U, frame_Wheel_L) annotation(
      Line(points = {{-58, 20}, {-100, 20}}, color = {95, 95, 95}));
    connect(macPherson_LF.frame_C, frame_C) annotation(
      Line(points = {{-38, 20}, {0, 20}, {0, -98}, {0, -98}}));
    connect(macPherson_RF.frame_C, frame_C) annotation(
      Line(points = {{42, 20}, {0, 20}, {0, -98}}, color = {95, 95, 95}));
    connect(macPherson_RF.frame_U, frame_Wheel_R) annotation(
      Line(points = {{62, 20}, {100, 20}}, color = {95, 95, 95}));
    connect(rackSteering.frame_X_LF, macPherson_LF.frame_S) annotation(
      Line(points = {{-10, 52}, {-48, 52}, {-48, 30}}, color = {95, 95, 95}));
    connect(rackSteering.frame_X_RF, macPherson_RF.frame_S) annotation(
      Line(points = {{10, 52}, {52, 52}, {52, 30}}));
  end MacPherson;

  model SimpleSuspension "Simple suspension"
  parameter Boolean animation=true "= true, if animation shall be enabled";
  VehicleDynamics.Suspensions.Components.SimpleSuspension simpleSuspension_L(animation=animation) annotation(
      Placement(visible = true, transformation(origin = {-30, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_C annotation(
      Placement(visible = true, transformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_UL annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  VehicleDynamics.Suspensions.Components.SimpleSuspension simpleSuspension_R(animation=animation, left = false)  annotation(
      Placement(visible = true, transformation(origin = {48, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_UR annotation(
      Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
  equation
  connect(simpleSuspension_L.frame_U, frame_UL) annotation(
      Line(points = {{-40, 0}, {-100, 0}}, color = {95, 95, 95}));
  connect(simpleSuspension_L.frame_C, frame_C) annotation(
      Line(points = {{-20, 0}, {0, 0}, {0, 100}}, color = {95, 95, 95}));
  connect(simpleSuspension_R.frame_U, frame_UR) annotation(
      Line(points = {{58, 0}, {100, 0}, {100, 0}, {100, 0}}));
  connect(simpleSuspension_R.frame_C, frame_C) annotation(
      Line(points = {{38, 0}, {0, 0}, {0, 100}, {0, 100}}, color = {95, 95, 95}));
  protected
  end SimpleSuspension;

  package Components "Components for suspension"
    model CutLink "Two bushings connected by a massless rod"
      import SI = Modelica.SIunits;
      extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
      parameter Boolean animation = true "= true, if animation shall be enabled";
      parameter Modelica.Mechanics.MultiBody.Types.Axis n1_a = {0, 0, 1};
      parameter SI.Position[3] rA = {0, 0, 0} "Position vector from frame_a to center of bushingA, resolved in frame_a";
      parameter SI.Position[3] rB = {0, 0, 0} "Position vector from center of bushingb to frame_b, resolved in frame_b";
      parameter SI.Position rRod_ia[3] = {1, 0, 0};
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotA(animation = false, r = rA) annotation(
        Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotB(r = -rB, animation = animation) annotation(
        Placement(visible = true, transformation(origin = {68, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Joints.UniversalSpherical joint(n1_a = n1_a, rRod_ia = rRod_ia, animation = animation) annotation(
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
      parameter Boolean animation = true "= true, if animation shall be enabled";
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_ia annotation(
        Placement(visible = true, transformation(origin = {-44, 100}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {-44, 100}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      parameter Modelica.Mechanics.MultiBody.Types.Axis n1_a = {0, 0, 1};
      parameter SI.Position rA[3] = {0, 0, 0};
      parameter SI.Position rB[3] = {0, 0, 0};
      parameter SI.Position rRod_ia[3] = {1, 0, 0};
      Modelica.Mechanics.MultiBody.Joints.Spherical jointB(animation = animation) annotation(
        Placement(visible = true, transformation(origin = {22, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Joints.Universal jointA(animation = animation, n_a = cross(rRod_ia, jointA.n_b), n_b = cross(n1_a, rRod_ia)) annotation(
        Placement(visible = true, transformation(origin = {-54, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation rod(animation = animation, r = rRod_ia) annotation(
        Placement(visible = true, transformation(origin = {-14, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotA(animation = false, r = rA) annotation(
        Placement(visible = true, transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation rotB(animation = animation, r = -rB, shapeType = "box") annotation(
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
      Modelica.Mechanics.MultiBody.Joints.Assemblies.JointUPS MacPherson(animation = false, n1_a = n_a, nAxis_ia = {0, 0, 1}) annotation(
        Placement(visible = true, transformation(origin = {-44, 18}, extent = {{20, -20}, {-20, 20}}, rotation = -90)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation outerRod(r = macPhersonData.rUW - macPhersonData.rUL1L2) annotation(
        Placement(visible = true, transformation(origin = {44, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      replaceable parameter ParameterSets.Components.MacPherson macPhersonData annotation(
        Placement(visible = true, transformation(origin = {-76, 86}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter SI.Position rS_helper_x[3] = {0, rS[2], rS[3]};
      Modelica.Mechanics.MultiBody.Parts.FixedRotation rotRod_x(angle = if leftMacPherson then -angleBetweenBreaksAndSuspension_x else angleBetweenBreaksAndSuspension_x, n = {1, 0, 0}, rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.RotationAxis) annotation(
        Placement(visible = true, transformation(origin = {74, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Real angleBetweenBreaksAndSuspension_x = To_deg(Math.asin(Math.Vectors.norm(cross(rS_helper_x, {0, 0, 1})) / Math.Vectors.norm(rS_helper_x)));
      parameter SI.Position rS[3] = macPhersonData.rCS - macPhersonData.rUL1L2;
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation debug(r = macPhersonData.rUW - macPhersonData.rCS) annotation(
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
      connect(rotRod_x.frame_b, frame_U) annotation(
        Line(points = {{84, -80}, {84, 0}, {100, 0}}, color = {95, 95, 95}));
      connect(springRod.frame_a, outerRod.frame_a) annotation(
        Line(points = {{36, -64}, {35.875, -64}, {35.875, -42}, {27.75, -42}, {27.75, -54}, {25.5, -54}, {25.5, -66}, {35, -66}, {35, -80}, {34, -80}}, color = {95, 95, 95}));
      connect(outerRod.frame_b, rotRod_x.frame_a) annotation(
        Line(points = {{54, -80}, {64, -80}}, color = {95, 95, 95}));
      connect(upper.frame_b, debug.frame_a) annotation(
        Line(points = {{-68, 48}, {-68, 32}}, color = {95, 95, 95}));
    end MacPherson;

    model Strut "Strut - Integrated spring-damper unit"
      extends Modelica.Mechanics.MultiBody.Interfaces.PartialTwoFrames;
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation frameTranslationA(r = rA) annotation(
        Placement(visible = true, transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslationB(animation = false, r = rB) annotation(
        Placement(visible = true, transformation(origin = {52, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(c = c, d = d, s_unstretched = s0) annotation(
        Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      parameter Modelica.SIunits.Position rA[3] "";
      parameter Modelica.SIunits.Position rB[3] "";
      parameter Modelica.SIunits.DampingCoefficient d "Damping coefficient for SpringDamperParallel unit [N/(m/s)]";
      parameter Modelica.SIunits.TranslationalSpringConstant c "Spring coefficient for SpringDamperParallel unit [N/m]";
      parameter Modelica.SIunits.Position s0 "unstretched spring [m]";
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

    model SimpleSuspension "Parameter set for simple suspension"
        import SI = Modelica.SIunits;
        parameter Boolean animation=true
        "= true, if animation shall be enabled"; 
        parameter Boolean left = true "if true spring is placed on left side";
        parameter ParameterSets.Components.SimpleSuspension simpleSuspensionData annotation(
        Placement(visible = true, transformation(origin = {-80, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        
        
        Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_C annotation(
          Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_U annotation(
          Placement(visible = true, transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    
        Modelica.Mechanics.MultiBody.Parts.FixedTranslation translationCU(animation = false, r = rCU) annotation(
          Placement(visible = true, transformation(origin = {-54, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
        Modelica.Mechanics.MultiBody.Parts.FixedTranslation translationUW(animation = animation, r = rUW) annotation(
          Placement(visible = true, transformation(origin = {70, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(animation = animation,n = simpleSuspensionData.prism, s(start = 0.4)) annotation(
          Placement(visible = true, transformation(origin = {0, 16}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamper(animation = animation,c = simpleSuspensionData.c, d = simpleSuspensionData.d, s_unstretched = 0.4) annotation(
          Placement(visible = true, transformation(origin = {0, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    protected
        parameter SI.Position scalingFactor[3] = if left then {1, 1, 1} else {1, -1, 1};
        parameter SI.Position rCU[3] = simpleSuspensionData.rCU .* scalingFactor;
        parameter SI.Position rUW[3] = simpleSuspensionData.rUW .* scalingFactor;
        
      
      equation
        connect(translationUW.frame_b, frame_U) annotation(
          Line(points = {{80, 0}, {100, 0}, {100, 0}, {100, 0}}));
        connect(frame_C, translationCU.frame_a) annotation(
          Line(points = {{-100, 0}, {-64, 0}}));
        connect(translationCU.frame_b, springDamper.frame_a) annotation(
          Line(points = {{-44, 0}, {-28, 0}, {-28, -18}, {-10, -18}}, color = {95, 95, 95}));
        connect(translationCU.frame_b, prismatic.frame_a) annotation(
          Line(points = {{-44, 0}, {-28, 0}, {-28, 16}, {-10, 16}}, color = {95, 95, 95}));
        connect(prismatic.frame_b, translationUW.frame_a) annotation(
          Line(points = {{10, 16}, {60, 16}, {60, 0}, {60, 0}}, color = {95, 95, 95}));
        connect(springDamper.frame_b, translationUW.frame_a) annotation(
          Line(points = {{10, -18}, {60, -18}, {60, 0}, {60, 0}}, color = {95, 95, 95}));
      end SimpleSuspension;
  end Components;

  package Tests "Tests of suspensions"
    model MacPherson
      VehicleDynamics.Suspensions.MacPherson macPherson annotation(
        Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
        Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      VehicleDynamics.Wheels.RillTyre.Wheel wheel_LF annotation(
        Placement(visible = true, transformation(origin = {-68, 2}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 0, 1.0}) annotation(
        Placement(visible = true, transformation(origin = {-4, -50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Wheels.RillTyre.Wheel wheel_RF(leftWheel = false) annotation(
        Placement(visible = true, transformation(origin = {58, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(macPherson.frame_Wheel_L, wheel_LF.carrierFrame) annotation(
        Line(points = {{-12, 2}, {-58, 2}}, color = {95, 95, 95}));
      connect(world.frame_b, fixedTranslation.frame_a) annotation(
        Line(points = {{-68, -76}, {-14, -76}, {-14, -50}, {-14, -50}}, color = {95, 95, 95}));
      connect(fixedTranslation.frame_b, macPherson.frame_C) annotation(
        Line(points = {{6, -50}, {-2, -50}, {-2, -10}}));
      connect(macPherson.frame_Wheel_R, wheel_RF.carrierFrame) annotation(
        Line(points = {{8, 2}, {48, 2}}, color = {95, 95, 95}));
    end MacPherson;

    model SimpleSuspension "Simple suspension"
    equation

    end SimpleSuspension;
  end Tests;
end Suspensions;