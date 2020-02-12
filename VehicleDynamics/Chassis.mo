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
    
    model StandardCar "Standard car"
    extends SedanBody(sedanBodyData.r_0 = {0.0, 0.0, 0.2}, sedanBodyData.rcmBody = {0, 0, 0}, mBody.r_0.start = {0, 0, 0.3});
    VehicleDynamics.Wheels.RillTyre.Wheel wheel_RR(leftWheel = false) annotation(
      Placement(visible = true, transformation(origin = {74, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    VehicleDynamics.Wheels.RillTyre.Wheel wheel_RL annotation(
      Placement(visible = true, transformation(origin = {74, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  VehicleDynamics.Wheels.RillTyre.Wheel wheel_LF annotation(
      Placement(visible = true, transformation(origin = {-70, -70}, extent = {{10, -10}, {-10, 10}}, rotation = 90)));
  VehicleDynamics.Wheels.RillTyre.Wheel wheel_RF(leftWheel = false) annotation(
      Placement(visible = true, transformation(origin = {-70, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation front(r = {1.0, 0, 0})  annotation(
      Placement(visible = true, transformation(origin = {-36, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation rear(r = {-1, 0, 0})  annotation(
      Placement(visible = true, transformation(origin = {38, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  VehicleDynamics.Suspensions.MacPherson macPherson annotation(
      Placement(visible = true, transformation(origin = {-68, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  VehicleDynamics.Suspensions.MacPherson macPherson1 annotation(
      Placement(visible = true, transformation(origin = {72, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  equation
    connect(mBody.frame_a, front.frame_a) annotation(
      Line(points = {{-10, 0}, {-26, 0}, {-26, 0}, {-26, 0}}));
  connect(mBody.frame_a, rear.frame_a) annotation(
      Line(points = {{-10, 0}, {28, 0}, {28, 0}, {28, 0}}, color = {95, 95, 95}));
  connect(macPherson.frame_Wheel_R, wheel_RF.carrierFrame) annotation(
      Line(points = {{-70, 10}, {-70, 60}}, color = {95, 95, 95}));
  connect(macPherson.frame_Wheel_L, wheel_LF.carrierFrame) annotation(
      Line(points = {{-70, -10}, {-70, -60}}, color = {95, 95, 95}));
  connect(front.frame_b, macPherson.frame_C) annotation(
      Line(points = {{-46, 0}, {-58, 0}}, color = {95, 95, 95}));
  connect(rear.frame_b, macPherson1.frame_C) annotation(
      Line(points = {{48, 0}, {82, 0}, {82, 0}, {82, 0}}, color = {95, 95, 95}));
  connect(macPherson1.frame_Wheel_R, wheel_RR.carrierFrame) annotation(
      Line(points = {{70, 10}, {74, 10}, {74, 60}, {74, 60}}));
  connect(macPherson1.frame_Wheel_L, wheel_RL.carrierFrame) annotation(
      Line(points = {{70, -10}, {74, -10}, {74, -60}, {74, -60}}, color = {95, 95, 95}));
  end StandardCar;

  package Configurations "Vehicle configurations"
      model StandardCar "Standard car configuration"
      inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
        Placement(visible = true, transformation(origin = {-84, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
      VehicleDynamics.Chassis.StandardCar standardCar annotation(
        Placement(visible = true, transformation(origin = {24, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation

    end StandardCar;
  end Configurations;
    
end Chassis;