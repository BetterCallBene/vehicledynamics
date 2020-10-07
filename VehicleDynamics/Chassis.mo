within VehicleDynamics;

package Chassis "Description"
  package ParameterSets "Sub package for all parameter sets body"
    extends Modelica.Icons.Library2;

    record SedanBody
      extends Modelica.Icons.Record;
      import SI = Modelica.SIunits;
      parameter SI.Position r_0[3] = {0.0, 0.0, 0.0} "Initialization|Position vector from origin of world frame to origin of frame_a";
      parameter SI.Mass mBody = 1400;
      parameter SI.Position rcmBody[3] = {-2.55 * 0.396, 0, 2.55 * 0.23 - 0.14} "|Mass and Inertia|Vector from frame_a to center of mass, resolved in frame_a";
      parameter SI.Inertia i11B = 500 "|Body|(1,1) element of inertia tensor";
      parameter SI.Inertia i22B = 1500 "|Body|(2,2) element of inertia tensor";
      parameter SI.Inertia i33B = 1800 "|Body|(3,3) element of inertia tensor";
      parameter SI.Inertia i21B = 0.0 "|Body|(2,1) element of inertia tensor";
      parameter SI.Inertia i31B = 0.0 "|Body|(3,1) element of inertia tensor";
      parameter SI.Inertia i32B = 0.0 "|Body|(3,2) element of inertia tensor";
    end SedanBody;
  end ParameterSets;

  package Components "Components"
    package Interfaces "Interfaces specialised for this example"
      extends Modelica.Icons.InterfacesPackage;

      expandable connector FullSuspensionChassis "Control bus that is adapted to the signals connected to it"
        extends Modelica.Icons.SignalBus;
        import SI = Modelica.SIunits;
        SI.Position r[3] "Absolute position of car";
        annotation(
          Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-20, 2}, {22, -2}}, lineColor = {255, 204, 51}, lineThickness = 0.5)}),
          Documentation(info = "<html>
            <p>
            This connector defines the \"expandable connector\" FullSuspensionChassis that
            is used as bus in the vehicledynamics library.            
            </p>
            </html>"));
      end FullSuspensionChassis;
    end Interfaces;

    model FullSuspensionChassis
      VehicleDynamics.Suspensions.MacPherson macPherson annotation(
        Placement(visible = true, transformation(origin = {0, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      VehicleDynamics.Wheels.RillTyre.Wheel wheel_LF annotation(
        Placement(visible = true, transformation(origin = {-52, 80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      VehicleDynamics.Wheels.RillTyre.Wheel wheel_RF(leftWheel = false) annotation(
        Placement(visible = true, transformation(origin = {58, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Parts.Body carBody(I_11 = 1, I_22 = 1, I_33 = 1, m = 200, r_0(start = {0, 0, 0.26}), r_CM = {-0.4, 0, 0}) annotation(
        Placement(visible = true, transformation(origin = {10, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      VehicleDynamics.Wheels.RillTyre.Wheel wheel_RR(leftWheel = false) annotation(
        Placement(visible = true, transformation(origin = {74, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      VehicleDynamics.Wheels.RillTyre.Wheel wheel_LR(animation = false, leftWheel = true) annotation(
        Placement(visible = true, transformation(origin = {-44, -74}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
      VehicleDynamics.Suspensions.SimpleSuspension simpleSuspension(animation = true) annotation(
        Placement(visible = true, transformation(origin = {0, -4}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_C annotation(
        Placement(visible = true, transformation(origin = {-98, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-98, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
      VehicleDynamics.Chassis.Components.Interfaces.FullSuspensionChassis fullSuspensionChassisBus annotation(
        Placement(visible = true, transformation(origin = {0, 100}, extent = {{-20, -20}, {20, 20}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-30, -30}, {30, 30}}, rotation = 0)));
      Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition annotation(
        Placement(visible = true, transformation(origin = {60, 42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
      connect(frame_C, carBody.frame_a) annotation(
        Line(points = {{-98, 0}, {0, 42}}));
      connect(carBody.frame_a, macPherson.frame_C) annotation(
        Line(points = {{0, 68.2}, {0, 42}}, color = {95, 95, 95}));
      connect(carBody.frame_a, simpleSuspension.frame_C) annotation(
        Line(points = {{0, 42}, {0, 6}}, color = {95, 95, 95}));
      connect(simpleSuspension.frame_UR, wheel_RR.carrierFrame) annotation(
        Line(points = {{10, -4}, {64, -4}, {64, -76}}));
      connect(simpleSuspension.frame_UL, wheel_LR.carrierFrame) annotation(
        Line(points = {{-10, -4}, {-34, -4}, {-34, -74}}, color = {95, 95, 95}));
      connect(macPherson.frame_Wheel_L, wheel_LF.carrierFrame) annotation(
        Line(points = {{-10, 80}, {-42, 80}}, color = {95, 95, 95}));
      connect(macPherson.frame_Wheel_R, wheel_RF.carrierFrame) annotation(
        Line(points = {{10, 80}, {48, 80}}, color = {95, 95, 95}));
      connect(carBody.frame_a, absolutePosition.frame_a) annotation(
        Line(points = {{0, 42}, {50, 42}, {50, 42}, {50, 42}}));
  connect(absolutePosition.r, fullSuspensionChassisBus.r) annotation(
        Line(points = {{72, 42}, {80, 42}, {80, 100}, {0, 100}, {0, 100}}, color = {0, 0, 127}, thickness = 0.5));
    end FullSuspensionChassis;
  end Components;

  package Interfaces "Interfaces specialised for this example"
    extends Modelica.Icons.InterfacesPackage;

    expandable connector CarBus "Control bus that is adapted to the signals connected to it"
      extends Modelica.Icons.SignalBus;
      Components.Interfaces.FullSuspensionChassis fullSuspensionChassisBus;
      annotation(
        Icon(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}}), graphics = {Rectangle(extent = {{-20, 2}, {22, -2}}, lineColor = {255, 204, 51}, lineThickness = 0.5)}),
        Documentation(info = "<html>
            <p>
            This connector defines the \"expandable connector\" CarBus that
            is used as bus in the vehicledynamics library.            
            </p>
            </html>"));
    end CarBus;
  end Interfaces;

  model StandardCar
    VehicleDynamics.Suspensions.MacPherson macPherson annotation(
      Placement(visible = true, transformation(origin = {0, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
      Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Wheels.RillTyre.Wheel wheel_LF annotation(
      Placement(visible = true, transformation(origin = {-62, 80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    VehicleDynamics.Wheels.RillTyre.Wheel wheel_RF(leftWheel = false) annotation(
      Placement(visible = true, transformation(origin = {60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.Body body(m = 100, r_0(start = {0, 0, 0.26})) annotation(
      Placement(visible = true, transformation(origin = {10, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation leftRear(r = {-0.75, 0.70, 0.047}) annotation(
      Placement(visible = true, transformation(origin = {-50, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation rightRear(r = {-0.75, -0.70, 0.047}) annotation(
      Placement(visible = true, transformation(origin = {40, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    VehicleDynamics.Wheels.RillTyre.Wheel wheel_LR annotation(
      Placement(visible = true, transformation(origin = {-60, -42}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    VehicleDynamics.Wheels.RillTyre.Wheel wheel_RR(leftWheel = false) annotation(
      Placement(visible = true, transformation(origin = {68, -42}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(macPherson.frame_Wheel_L, wheel_LF.carrierFrame) annotation(
      Line(points = {{-10, 80}, {-52, 80}}, color = {95, 95, 95}));
    connect(macPherson.frame_Wheel_R, wheel_RF.carrierFrame) annotation(
      Line(points = {{10, 80}, {50, 80}}, color = {95, 95, 95}));
    connect(macPherson.frame_C, body.frame_a) annotation(
      Line(points = {{0, 68.2}, {0, 46}}, color = {95, 95, 95}));
    connect(body.frame_a, leftRear.frame_a) annotation(
      Line(points = {{0, 46}, {-50, 46}, {-50, 0}}, color = {95, 95, 95}));
    connect(body.frame_a, rightRear.frame_a) annotation(
      Line(points = {{0, 46}, {40, 46}, {40, 0}}, color = {95, 95, 95}));
    connect(leftRear.frame_b, wheel_LR.carrierFrame) annotation(
      Line(points = {{-50, -20}, {-50, -42}}));
    connect(rightRear.frame_b, wheel_RR.carrierFrame) annotation(
      Line(points = {{40, -20}, {41, -20}, {41, -28}, {42, -28}, {42, -42}, {58, -42}}));
  end StandardCar;

  model FullSuspensionCar "FullSuspensionCar"
    inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
      Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Chassis.Components.FullSuspensionChassis fullSuspensionChassis annotation(
      Placement(visible = true, transformation(origin = {2, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VehicleDynamics.Chassis.Interfaces.CarBus carBus annotation(
      Placement(visible = true, transformation(origin = {-98, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {-86, 0}, extent = {{-20, -20}, {20, 20}}, rotation = 0)));
    equation
      connect(fullSuspensionChassis.fullSuspensionChassisBus, carBus.fullSuspensionChassisBus) annotation(
      Line(points = {{2, 0}, {-98, 0}}));
  end FullSuspensionCar;

  package Configurations "Vehicle configurations"
    model StandardCar "Standard car configuration"
      inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
        Placement(visible = true, transformation(origin = {-84, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      VehicleDynamics.Chassis.StandardCar standardCar annotation(
        Placement(visible = true, transformation(origin = {24, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation

    end StandardCar;
  end Configurations;

  model BasicSuspension
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(r = {-0.75, 0.70, 2}) annotation(
      Placement(visible = true, transformation(origin = {0, 24}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Parts.Body body1(I_11 = 0.1, I_22 = 0.1, I_33 = 0.1, m = 1) annotation(
      Placement(visible = true, transformation(origin = {12, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
      Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(n = {0, 0, -1}, s(start = 0.5)) annotation(
      Placement(visible = true, transformation(origin = {2, -12}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(c = 10, d = 100, s_unstretched = 0.2) annotation(
      Placement(visible = true, transformation(origin = {38, -12}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  equation
    connect(world.frame_b, fixedTranslation1.frame_a) annotation(
      Line(points = {{-68, -76}, {-38, -76}, {-38, -52}, {-68, -52}, {-68, 34}, {0, 34}}));
    connect(fixedTranslation1.frame_b, prismatic.frame_a) annotation(
      Line(points = {{0, 14}, {0, 6}, {2, 6}, {2, -2}}, color = {95, 95, 95}));
    connect(prismatic.frame_b, body1.frame_a) annotation(
      Line(points = {{2, -22}, {2, -22}, {2, -60}, {2, -60}}, color = {95, 95, 95}));
    connect(fixedTranslation1.frame_b, springDamperParallel.frame_a) annotation(
      Line(points = {{0, 14}, {38, 14}, {38, -2}, {38, -2}}, color = {95, 95, 95}));
    connect(springDamperParallel.frame_b, prismatic.frame_b) annotation(
      Line(points = {{38, -22}, {2, -22}, {2, -22}, {2, -22}}, color = {95, 95, 95}));
  end BasicSuspension;
end Chassis;