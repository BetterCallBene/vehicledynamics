within VehicleDynamics;

package Suspensions "Suspensions, models ready to be used as front or rear suspensions."
  model MacPherson "MacPherson strut linkage"
    import SI = Modelica.SIunits;
    extends Chassis.Interfaces.Linkages;
    parameter Real[3] scaleFactor = {1, 1, 1} "To make right hand side Linkage use {1,-1,1}";
    parameter SI.Position[3] rCL1 = {0.1070, 0.55, -0.0380} "|Geometry| Vector from origin of frame_C to front link mount in chassis resolved in frame_C (at initial time)";
    parameter SI.Position[3] rCL2 = {-0.3070, 0.55, -0.0380} "|Geometry| Vector from origin of frame_C to rear link mount in chassis resolved in frame_C (at initial time)";
    parameter SI.Position[3] rCS = {-0.0295, 0.850, 0.5670} "|Geometry| Vector from origin of frame_C to strut mount in chassis resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUS = {-0.0070, 0.8733, 0.1380} "|Geometry| Vector from origin of frame_C to strut mount in uppright resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUL1L2 = {-0.0070, 0.8733, -0.0380} "|Geometry| Vector from origin of frame_C to low spindleUtilities.Joints.Joint resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUW = {0, 0.9, 0} "|Geometry| Vector from origin of frame_C to wheel centre resolved in frame_C (at initial time)";
    parameter SI.Position[3] rRL3 = {-0.12, 0.85, 0.08} "|Geometry| Vector from origin of frame_C to origin of frame_S resolved in frame_C (at initial time)";
    parameter SI.Position[3] rUL3 = {0, 0.9, 0} "|Geometry| Vector from origin of frame_C to lower ball of steering rod resolved in frame_C (at initial time)";
    parameter SI.Length q0S = 0.05 "|Geometry| Unstretched additional spring length of the strut, compared to the construction geometry";
    parameter SI.Position[3] rCMU = {0, 0, 0} "|Mass and Inertia|Wheel carrier|";
    parameter SI.Mass mU = 0 "|Mass and Inertia|Wheel carrier|";
    parameter SI.Inertia i11U = 0 "|Mass and Inertia|Wheel carrier|";
    parameter SI.Inertia i22U = 0 "|Mass and Inertia|Wheel carrier|";
    parameter SI.Inertia i33U = 0 "|Mass and Inertia|Wheel carrier|";
    parameter SI.Inertia i21U = 0 "|Mass and Inertia|Wheel carrier|";
    parameter SI.Inertia i31U = 0 "|Mass and Inertia|Wheel carrier|";
    parameter SI.Inertia i32U = 0 "|Mass and Inertia|Wheel carrier|";
    parameter SI.Position[3] rCML1L2 = {0, 0, 0} "|Mass and Inertia|Wishbone|";
    parameter SI.Mass mL1L2 = 0 "|Mass and Inertia|Wishbone|";
    parameter SI.Inertia i11L1L2 = 0 "|Mass and Inertia|Wishbone|";
    parameter SI.Inertia i22L1L2 = 0 "|Mass and Inertia|Wishbone|";
    parameter SI.Inertia i33L1L2 = 0 "|Mass and Inertia|Wishbone|";
    parameter SI.Inertia i21L1L2 = 0 "|Mass and Inertia|Wishbone|";
    parameter SI.Inertia i31L1L2 = 0 "|Mass and Inertia|Wishbone|";
    parameter SI.Inertia i32L1L2 = 0 "|Mass and Inertia|Wishbone|";
    /*Scaled coordinates*/
    Modelica.Mechanics.MultiBody.Joints.Revolute innerJoint(stateSelect = StateSelect.always, n = rCL1_scaled - rCL2_scaled) annotation(
      Placement(visible = true, transformation(origin = {-70, 58}, extent = {{-10, 20}, {10, -20}}, rotation = 0)));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_L12 annotation(
      extent = [-13, -119; 17, -89],
      rotation = 270);
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation upper(r = rCS_scaled) annotation(
      extent = [-64, 9; -84, 29],
      rotation = 90);
    Modelica.Mechanics.MultiBody.Joints.Assemblies.JointUPS MacPherson(n1_a = n_a) annotation(
      extent = [22, -21; -17, 19],
      rotation = 90);
    Modelica.Mechanics.MultiBody.Parts.BodyShape springRod(r = rUS_scaled - rUL1L2_scaled, m = 0) annotation(
      extent = [33, -31; 53, -11],
      rotation = 90);
    /* Modify deprecated VehicleDynamics Library
                 * Replace non linear spring damp through linear spring damp model
                 * Easy to use
                 */
    Modelica.Mechanics.MultiBody.Forces.SpringDamperParallel strut(s_unstretched = q0Strut, d = 1.0, c = 1.0) annotation(
      extent = [32, -1; 52, 19],
      rotation = 90);
    Modelica.Mechanics.MultiBody.Parts.BodyShape outerRod(r = rUW_scaled - rUL1L2_scaled, m = 0) annotation(
      extent = [68, -11; 88, 9],
      rotation = 0);
    Modelica.Mechanics.MultiBody.Parts.Body U(r_CM = rCMU, m = mU, I_11 = i11U, I_22 = i22U, I_33 = i33U, I_21 = i21U, I_31 = i31U, I_32 = i32U) annotation(
      extent = [69, -61; 89, -41]);
    /*  Modify deprecated VehicleDynamics Library
         *  Utilities.Joints.JointRSU                         Modelica.Mechanics.MultiBo…Joints.Assemblies.JointUSR
         *  old                                               new
         *  JointRSU  Description                             JointSSR  Description
         *  frame_a                                           frame_b
         *  frame_b                                           frame_a
         *  n_a[3]    Axis of rotation of revolute joint      n_b       Axis of revolute joint fixed 
         *            resolved in frame_a                               resolved in frame_b
         *  
         *  n_b[3]    First rotation axis of universal joint  
         *            fixed and resolved in frame_b                    
         *
         *  r_a[3]    Position vector from origin of frame_a  rRod2_ib  Vector from origin of frame_ib to spherical joint in the middle, resolved in frame_ib
         *            to spherical joint at initial time, 
         *            resolved in frame_a [m]
         *
         *
            frame_c                                           frame_ia

            frame_tb                                          frame_im

            frame_ta                                          frame_ib

            old:    
            Utilities.Joints.JointRSU steeringJoint(n_b = {1, 0, 0}, n_a = rCS_scaled - rUL1L2_scaled, r_a = rUL3_scaled - rUL1L2_scaled) annotation(
            extent = [54, 39; 84, 69],
            rotation = 90);
            */
    Modelica.Mechanics.MultiBody.Joints.Assemblies.JointSSR steeringJoint(rod1Mass=0, rod1Length=Modelica.Math.Vectors.length(rUL3_scaled - rRL3_scaled), n_b = rCS_scaled - rUL1L2_scaled, rRod2_ib = rUL3_scaled - rUL1L2_scaled) annotation(
      extent = [54, 39; 84, 69],
      rotation = 90);
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation lower(r = rCL1_scaled) annotation(
      extent = [-84, -31; -64, -11],
      rotation = 270);
    Modelica.Mechanics.MultiBody.Parts.BodyShape frontBar(r = rUL1L2_scaled - rCL1_scaled) annotation(
      extent = [-37, -71; -17, -51]);
    Modelica.Mechanics.MultiBody.Parts.BodyShape rearBar(r = rCL2_scaled - rUL1L2_scaled, widthDirection = cross(rCL2_scaled - rUL1L2_scaled, {0, 0, 1})) annotation(
      extent = [-37, -31; -17, -51],
      rotation = 180);
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_S annotation(
      extent = [-70, 50; -100, 80],
      rotation = 180);
    Modelica.Mechanics.MultiBody.Parts.BodyShape steeringLever(r = rUL3_scaled - rUL1L2_scaled) annotation(
      extent = [98, 66; 118, 46],
      rotation = 270);
  protected
    parameter SI.Position[3] rCL1_scaled = scaleFactor .* rCL1;
    parameter SI.Position[3] rCL2_scaled = scaleFactor .* rCL2;
    parameter SI.Position[3] rCS_scaled = scaleFactor .* rCS;
    parameter SI.Position[3] rUS_scaled = scaleFactor .* rUS;
    parameter SI.Position[3] rUL1L2_scaled = scaleFactor .* rUL1L2;
    parameter SI.Position[3] rUW_scaled = scaleFactor .* rUW;
    parameter SI.Position[3] rRL3_scaled = scaleFactor .* rRL3;
    parameter SI.Position[3] rUL3_scaled = scaleFactor .* rUL3;
    parameter SI.Length q0Strut = Modelica.Math.Vectors.length(rCS_scaled - rUS_scaled) + q0S;
    parameter SI.Length L_frontBar = Modelica.Math.Vectors.length(rCL1_scaled - rUL1L2_scaled) "length of frontBar";
    parameter SI.Length L_rearBar = Modelica.Math.Vectors.length(rCL2_scaled - rUL1L2_scaled) "length of rearBar";
    parameter SI.Length L_springRod = Modelica.Math.Vectors.length(rUS_scaled - rUL1L2_scaled);
    parameter SI.Length L_outerRod = Modelica.Math.Vectors.length(rUW_scaled - rUL1L2_scaled);
    parameter SI.Position rS[3] = rCS_scaled - rUL1L2_scaled;
    parameter SI.Position rU[3] = rUW_scaled - rUL1L2_scaled "Position vector from frame_L1L2 to frame_U, resolved in frame_C";
    parameter Real n_a[3] = cross(rS, rU) "First rotation axis of universalUtilities.Joints.Joint in springJoint, resolved in frame_C";
  protected
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_ta1 annotation(
      extent = [62, -11; 64, -9]);
  equation
    connect(lower.frame_a, frame_C) annotation(
      points = [-74, -10.5; -74, -1; -105, -1]);
    connect(upper.frame_a, frame_C) annotation(
      points = [-74, 8.5; -74, -1; -105, -1]);
    connect(MacPherson.frame_b, upper.frame_b) annotation(
      points = [2.5, 20; 3, 39; -74, 39; -74, 29.5]);
/*  Modify deprecated VehicleDynamics Library
                Replace JointRSU with JointUSR
                frame_b to frame_a
                connect(steeringJoint.frame_b, frame_S) annotation(
                points = [69, 69.75; 69, 65; -85, 65]);
        */
    connect(steeringJoint.frame_a, frame_S) annotation(
      points = [69, 69.75; 69, 65; -85, 65]);
    connect(outerRod.frame_b, frame_U) annotation(
      points = [88.5, -1; 105, 1]);
/*  Modify deprecated VehicleDynamics Library
            connect(MacPherson.frame_pb, strut.frame_b) annotation(
            points = [24.925, 11; 27, 3; 27, 29; 42, 29; 42, 19.5]);
        */
    connect(MacPherson.frame_ib, strut.frame_b) annotation(
      points = [24.925, 11; 27, 3; 27, 29; 42, 29; 42, 19.5]);
    connect(springRod.frame_b, strut.frame_a) annotation(
      points = [43, -10.5; 43, -1.5; 42, -1.5]);
    connect(frame_ta1, outerRod.frame_a) annotation(
      points = [63, -10; 67.5, -1]);
    connect(U.frame_a, frame_ta1) annotation(
      points = [68.5, -51; 63, -10]);
/*  Modify deprecated VehicleDynamics Library
            Replace JointRSU with JointUSR
            frame_ta to frame_ib
            connect(frame_ta1, steeringJoint.frame_ta) annotation(
            points = [63, -10; 51.75, 42.15]);
        */
    connect(frame_ta1, steeringJoint.frame_ib) annotation(
      points = [63, -10; 51.75, 42.15]);
/*  Modify deprecated VehicleDynamics Library
            Replace JointRSU with JointUSR
            frame_a to frame_b
            connect(MacPherson.frame_pa, steeringJoint.frame_a) annotation(
            points = [24.925, -13; 29, -15; 29, -51; 57, -51; 57, 29; 69, 29; 69, 38.25]); 
        */
    connect(MacPherson.frame_ia, steeringJoint.frame_b) annotation(
      points = [24.925, -13; 29, -15; 29, -51; 57, -51; 57, 29; 69, 29; 69, 38.25]);
    connect(springRod.frame_a, frame_ta1) annotation(
      points = [43, -31.5; 63, -10]);
    connect(rearBar.frame_a, frontBar.frame_b) annotation(
      points = [-16.5, -41; -16.5, -45.75; -16, -45.75; -16, -50.5; -17, -50.5; -17, -62]);
    connect(frontBar.frame_b, MacPherson.frame_a) annotation(
      points = [-16.5, -61; 2.5, -61; 2.5, -22]);

    connect(frame_L12, frontBar.frame_b) annotation(
      points = [2, -104; -16.5, -61]);
/*  Modify deprecated VehicleDynamics Library
            Replace JointRSU with JointUSR
            frame_ta  to frame_ib
            connect(steeringLever.frame_a, steeringJoint.frame_ta) annotation(
            points = [108, 45.5; 51.75, 42.15]);
        */
    connect(steeringLever.frame_a, steeringJoint.frame_ib) annotation(
      points = [108, 45.5; 51.75, 42.15]);
    connect(innerJoint.frame_a, lower.frame_b) annotation(
      Line);
    connect(innerJoint.frame_b, frontBar.frame_a) annotation(
      Line);
  end MacPherson;

  model FunctionalTestMacPherson "Description"
    inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(visible = true, transformation(origin = {-78, -76}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
    VehicleDynamics.Suspensions.MacPherson macPherson annotation(
      Placement(visible = true, transformation(origin = {-28, 14}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  equation
    connect(world.frame_b, macPherson.frame_C);
  
  end FunctionalTestMacPherson;
end Suspensions;