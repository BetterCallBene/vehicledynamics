within VehicleDynamics;

package Chassis "Description"

    package ParameterSets "Sub package for all parameter sets body"
        extends Modelica.Icons.Library2;

        record SedanBody
        extends Modelica.Icons.Record;
            import SI = Modelica.SIunits;
            parameter SI.Position r_0[3] = {0.0, 0.0, 0.0} "Initialization|Position vector from origin of world frame to origin of frame_a";
            parameter SI.Mass mBody = 1400;
            parameter SI.Position rcmBody[3]={-2.55*0.396,0,2.55*0.23 - 0.14} 
            "|Mass and Inertia|Vector from frame_a to center of mass, resolved in frame_a";
            parameter SI.Inertia i11B = 500  "|Body|(1,1) element of inertia tensor";
            parameter SI.Inertia i22B = 1500 "|Body|(2,2) element of inertia tensor";
            parameter SI.Inertia i33B = 1800 "|Body|(3,3) element of inertia tensor";
            
            parameter SI.Inertia i21B=0.0 "|Body|(2,1) element of inertia tensor";
            parameter SI.Inertia i31B=0.0 "|Body|(3,1) element of inertia tensor";
            parameter SI.Inertia i32B=0.0 "|Body|(3,2) element of inertia tensor";
        
        end SedanBody;
    end ParameterSets;

    model SedanBody "Passenger with a three-box configuration"
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    
    Modelica.Mechanics.MultiBody.Parts.Body mBody(I_11 = sedanBodyData.i11B, I_21 = sedanBodyData.i21B, I_22 = sedanBodyData.i22B, I_31 = sedanBodyData.i31B, I_32 = sedanBodyData.i32B, I_33 = sedanBodyData.i33B, m = sedanBodyData.mBody, r_0(start = sedanBodyData.r_0), r_CM = sedanBodyData.rcmBody)  annotation(
      Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    ParameterSets.SedanBody sedanBodyData annotation(
      Placement(visible = true, transformation(origin = {-80, 88}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    equation
    connect(frame_a, mBody.frame_a) annotation(
      Line(points = {{-100, 0}, {-12, 0}, {-12, 0}, {-12, 0}}));
  
     
    end SedanBody;
    
  model StandardCar
VehicleDynamics.Suspensions.MacPherson macPherson annotation(
    Placement(visible = true, transformation(origin = {0, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
VehicleDynamics.Wheels.RillTyre.Wheel wheel_LF annotation(
    Placement(visible = true, transformation(origin = {-62, 80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
VehicleDynamics.Wheels.RillTyre.Wheel wheel_RF(leftWheel = false) annotation(
    Placement(visible = true, transformation(origin = {60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Mechanics.MultiBody.Parts.Body body(m = 100, r_0(start = {0, 0, 0.26}))  annotation(
    Placement(visible = true, transformation(origin = {10, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation leftRear(r = {-0.75, 0.70, 0.047})  annotation(
      Placement(visible = true, transformation(origin = {-50, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation rightRear(r = {-0.75, -0.70, 0.047})  annotation(
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

model StandardCarFullSuspension
VehicleDynamics.Suspensions.MacPherson macPherson annotation(
    Placement(visible = true, transformation(origin = {0, 78}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
    Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
VehicleDynamics.Wheels.RillTyre.Wheel wheel_LF annotation(
    Placement(visible = true, transformation(origin = {-62, 80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
VehicleDynamics.Wheels.RillTyre.Wheel wheel_RF(leftWheel = false) annotation(
    Placement(visible = true, transformation(origin = {60, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
Modelica.Mechanics.MultiBody.Parts.Body carBody(m = 200, r_0(start = {0, 0, 0.26}), r_CM = {-0.4, 0, 0})  annotation(
    Placement(visible = true, transformation(origin = {16, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation rightRear(animation = false, r = {-2, -0.6, 0.5})  annotation(
      Placement(visible = true, transformation(origin = {40, -10}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  VehicleDynamics.Wheels.RillTyre.Wheel wheel_RR(leftWheel = false) annotation(
      Placement(visible = true, transformation(origin = {84, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation1(animation = false, r = {-2.0, 0.6, 0.5}) annotation(
      Placement(visible = true, transformation(origin = {-40, -6}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(c = 100, d = 10000, s_unstretched = 0.4) annotation(
      Placement(visible = true, transformation(origin = {-40, -36}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(n = {0, 0, -1}, s(start = 0.4))  annotation(
      Placement(visible = true, transformation(origin = {0, -38}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  VehicleDynamics.Wheels.RillTyre.Wheel wheel_LR(leftWheel = true) annotation(
      Placement(visible = true, transformation(origin = {8, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel1(c = 100, d = 10000, s_unstretched = 0.4) annotation(
      Placement(visible = true, transformation(origin = {40, -38}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic1(n = {0, 0, -1}, s(start = 0.4)) annotation(
      Placement(visible = true, transformation(origin = {82, -38}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation rUWL(r = {0, 0.10, 0})  annotation(
      Placement(visible = true, transformation(origin = {-34, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation rUWR(r = {0, -0.10, 0})  annotation(
      Placement(visible = true, transformation(origin = {46, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(macPherson.frame_Wheel_L, wheel_LF.carrierFrame) annotation(
      Line(points = {{-10, 80}, {-52, 80}}, color = {95, 95, 95}));
    connect(macPherson.frame_Wheel_R, wheel_RF.carrierFrame) annotation(
      Line(points = {{10, 80}, {50, 80}}, color = {95, 95, 95}));
  connect(macPherson.frame_C, carBody.frame_a) annotation(
      Line(points = {{0, 68.2}, {0, 56.1}, {6, 56.1}, {6, 40}}, color = {95, 95, 95}));
  connect(carBody.frame_a, rightRear.frame_a) annotation(
      Line(points = {{6, 40}, {40, 40}, {40, 0}}, color = {95, 95, 95}));
  connect(carBody.frame_a, fixedTranslation1.frame_a) annotation(
      Line(points = {{6, 40}, {-40, 40}, {-40, 4}}, color = {95, 95, 95}));
    connect(fixedTranslation1.frame_b, springDamperParallel.frame_a) annotation(
      Line(points = {{-40, -16}, {-40, -26}}));
    connect(fixedTranslation1.frame_b, prismatic.frame_a) annotation(
      Line(points = {{-40, -16}, {0, -16}, {0, -28}}));
    connect(rightRear.frame_b, springDamperParallel1.frame_a) annotation(
      Line(points = {{40, -20}, {40, -20}, {40, -28}, {40, -28}}));
    connect(rightRear.frame_b, prismatic1.frame_a) annotation(
      Line(points = {{40, -20}, {82, -20}, {82, -28}}));
  connect(springDamperParallel.frame_b, rUWL.frame_a) annotation(
      Line(points = {{-40, -46}, {-44, -46}, {-44, -76}, {-44, -76}}, color = {95, 95, 95}));
  connect(prismatic.frame_b, springDamperParallel.frame_b) annotation(
      Line(points = {{0, -48}, {-40, -48}, {-40, -46}, {-40, -46}}, color = {95, 95, 95}));
  connect(rUWL.frame_b, wheel_LR.carrierFrame) annotation(
      Line(points = {{-24, -76}, {-2, -76}}, color = {95, 95, 95}));
  connect(springDamperParallel1.frame_b, rUWR.frame_a) annotation(
      Line(points = {{40, -48}, {36, -48}, {36, -76}, {36, -76}}, color = {95, 95, 95}));
  connect(rUWR.frame_b, wheel_RR.carrierFrame) annotation(
      Line(points = {{56, -76}, {74, -76}}, color = {95, 95, 95}));
  connect(prismatic1.frame_b, springDamperParallel1.frame_b) annotation(
      Line(points = {{82, -48}, {40, -48}, {40, -48}, {40, -48}}, color = {95, 95, 95}));
  end StandardCarFullSuspension;

  package Configurations "Vehicle configurations"
      model StandardCar "Standard car configuration"
      inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
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
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(n = {0, 0, -1}, s(start = 0.5))  annotation(
      Placement(visible = true, transformation(origin = {2, -12}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel springDamperParallel(c = 10, d = 100, s_unstretched = 0.2)  annotation(
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