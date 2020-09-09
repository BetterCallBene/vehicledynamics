within VehicleDynamics;

package Steering "Steering systems"

  package ParameterSets "Sub package for all parameter sets for steering implementations"
    extends Modelica.Icons.Library2;

      record RackSteering
      extends Modelica.Icons.Record;
        import SI = Modelica.SIunits;
        // strut
        parameter SI.Position rRL_LF[3] = {-0.16, 0.54, 0};
        parameter SI.Position rRL_RF[3] = {-0.16, -0.54, 0};
        parameter Real ratioWheelToRack = 22 "rotatational to translational ratio [m/rad]";
        parameter SI.Inertia iSW = 0.2 "Steering inertia";
        parameter SI.TranslationalSpringConstant  fc = 100.0;
        parameter SI.TranslationalDampingConstant fd = 10000.0;
        
      end RackSteering;
  end ParameterSets;

  model RackSteering "Standard steering system"//   Utilities.Forces.SpringDamperTableRot1D springDamper(data=data_R) annotation(
//   Placement(visible = true, transformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
//   parameter Utilities.Forces.Utilities.ForceTable1D data_R
//     annotation (extent=[23, -2; 43, 18]);

    import SI = Modelica.SIunits;
    
    parameter VehicleDynamics.Steering.ParameterSets.RackSteering rackSteeringData "Structure for racksteering" annotation(
      Placement(visible = true, transformation(origin = {-88, 70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(animation = false, r = (rRL_LF + rRL_RF) / 2) annotation(
      Placement(visible = true, transformation(origin = {24, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_C annotation(
      Placement(visible = true, transformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_X_LF annotation(
      Placement(visible = true, transformation(origin = {-60, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-60, -100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_X_RF annotation(
      Placement(visible = true, transformation(origin = {-60, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0), iconTransformation(origin = {-60, 100}, extent = {{-16, -16}, {16, 16}}, rotation = 0)));
    
  protected
    parameter SI.Position rRL_LF[3] = rackSteeringData.rRL_LF;
    parameter SI.Position rRL_RF[3] = rackSteeringData.rRL_RF;
    parameter SI.TranslationalSpringConstant fc = rackSteeringData.fc;
    parameter SI.TranslationalSpringConstant fd = rackSteeringData.fd;
    parameter Real ratioWheelToRack = rackSteeringData.ratioWheelToRack "rotatational to translational ratio [m/rad]";
    parameter SI.Inertia iSW = rackSteeringData.iSW "Steering inertia";
  equation
    connect(fixedTranslation.frame_a, frame_C) annotation(
      Line(points = {{34, 40}, {98, 40}, {98, 0}, {102, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, frame_X_LF) annotation(
      Line(points = {{14, 40}, {-60, 40}, {-60, -100}, {-60, -100}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, frame_X_RF) annotation(
      Line(points = {{14, 40}, {-60, 40}, {-60, 100}, {-60, 100}}));
  end RackSteering;

  package Tests "Tests of suspensions"
    model RackSteering
    VehicleDynamics.Suspensions.MacPherson macPherson annotation(
        Placement(visible = true, transformation(origin = {-2, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    inner Modelica.Mechanics.MultiBody.World world annotation(
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
    end RackSteering;
  end Tests;
end Steering;