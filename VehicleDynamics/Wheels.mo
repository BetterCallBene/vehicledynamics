within VehicleDynamics;

package Wheels "Wheel, tyre and road models"
    record Data "Data needed for all wheel types"
        import SI = Modelica.SIunits;
        parameter SI.Radius R0 = 0.3 "undeformed radius of wheel";
        parameter SI.Radius Rrim = 0.2 "radius of rim";
        parameter SI.Length width = 0.19 "width of wheel (>0)";
        parameter SI.Mass m = 10 "mass of wheel";
        parameter SI.Inertia Iyy = 1 "wheel inertia with respect to wheel axis";
        parameter SI.Inertia Ixx = 1 "wheel inertia with respect to axis perpendicular to wheel axis";
    end Data;

    partial model BaseWheel "Properties common to all types of wheels and tyres" 
        import SI = Modelica.SIunits;
        // import Car;
        // PARAMETERS COMMON FOR ALL WHEELS
        parameter Real n[3] = {0, 1, 0} "Unit vector in direction of spin axis (same direction for left and right wheel)";
        parameter Boolean leftWheel = true "true, if left wheel, otherwise right wheel";
        parameter Boolean exact = false "false if influence of bearing acceleration is neglected";
        final parameter Real nShape[3] = if leftWheel then n else -n;
        constant SI.Velocity v_eps = 1.e-10;
        // Modelica.Constants.eps "1/v_eps should be representable on the machine";
        // GEOMETRIC VARIABLES
        SI.Angle camberAngle "Angle between wheel plane and contact normal";
        Real cosCamberAngle "Cosine of camber angle";
        SI.Radius Rdym "Distance between wheel center point and contact point";
        SI.Length delta_r "= R0 - Rdym, where R0 is the undeformed wheel radius and Rdym is the distance between wheel center point and contact point";
        Real Sroad[3, 3] "Transformation matrix from road frame to inertial frame";
        Real S[3, 3] "Transformation matrix from wheel carrier frame to road frame";
        Real S2[3, 3] " S2: Transformation matrix from road frame to frame_b";
        SI.Position rRoad0[3] "Position vector from inertial to road frame, resolved in inertial frame";
        SI.Position Cr[3] "Position vector from road frame to C-Frame, resolved in road frame";
        SI.Position Wr[3] "Position vector from road frame to W-Frame, resolved in road frame";
        SI.Position rP0[3] "Position vector from road frame to point P0, resolved in road frame";
        SI.Position rP1[3] "Position vector from road frame to point P1, resolved in road frame";
        SI.Position rC_P1[3] "Position vector from C-frame to point P1, resolved in road frame";
        SI.Position rC_W[3] "Position vector from C-frame to W-frame, resolved in road frame";
        Real nn[3] "Normalized vector of wheel spin axis, resoved in wheel carrier frame";
        Real Ce_x[3] "Unit vector in x-direction of C-Frame, resolved in road frame";
        Real Ce_y[3] "Unit vector in y-direction of C-Frame, resolved in road frame";
        Real Ce_z[3] "Unit vector in z-direction of C-Frame, resolved in road frame";
        Real Ce_z_0[3](start = {0, 0, 1}) "Initial guess for Ce_z";
        Real We_x[3] "Unit vector in x-direction of W-Frame, resolved in road frame";
        Real We_y[3] "Unit vector in y-direction of W-Frame, resolved in road frame";
        Real We_z[3] "Unit vector in z-direction of W-Frame, resolved in road frame";
        SI.AngularVelocity w "Rotational speed of wheel around wheel axis";
        SI.Angle phi "Rotational position of wheel around wheel axis";
        SI.AngularVelocity wWheel[3] "Absolute angular velocity of the rotating wheel, resolved in frame road";
        // Modify deprecated VehicleDynamics Library
        // Add helper temp variable RwCarrier
        SI.AngularVelocity RwCarrier[3] "Absolute angular velocity of the carrierFrame, resolved in frame road";
        SI.Velocity Cv[3] "Absolute velocity of wheel center point, resolved in frame road";
        SI.Velocity ddelta_r "Derivative of delta_r (= -der(Rdym))";
        SI.Velocity Wv_x "Absolute velocity of contact point in longitudinal direction We_x";
        SI.Velocity Wv_y "Absolute velocity of contact point in lateral direction We_y";
        SI.Velocity v_x "Absolute velocity of wheel in longitudinal direction We_x";
        SI.Velocity v_y "Absolute velocity of wheel in lateral direction We_y";
        // Modify deprecated VehicleDynamics Library
        // Add helper temp variable Carrierv
        SI.Velocity Carrierv[3] "Absolute velocity of the carrier, resolved in intial system";
        SI.Force F_x "Tyre/contact force at origin of W-frame in x-direction";
        SI.Force F_y "Tyre/contact force at origin of W-frame in y-direction";
        SI.Force F_z "Tyre/contact force at origin of W-frame in z-direction";
        SI.Force M_x "Tyre/contact torque at origin of W-frame in x-direction";
        SI.Force M_y "Tyre/contact torque at origin of W-frame in y-direction";
        SI.Force M_z "Tyre/contact torque at origin of W-frame in z-direction";
        // ********************************************************************
        // RESULTANT FORCES ANS TORQUES
        SI.Force Wf[3] "Tyre/contact force at origin of W-frame, resolved in road frame";
        SI.Torque Wt[3] "Tyre/contact torque at W-frame, resolved in road frame";
        SI.Torque Ct[3] "Tyre torque acting at wheel center, resolved in wheel_body frame";
        SI.Torque Ct_tyre "Tyre torque acting at wheel center in direction of spin axis";
          // *********************************************************************
        Road road;
        Road road2;
        Real mueRoad;
        Real altitudeRoad;
        Real normalRoad[3];
      
    public 
        // Modify deprecated VehicleDynamics Library
        // For fast development replacement general interface through specific road
        inner block Road = Environments.NoGraphicsRoad;
        replaceable Data wheelData;
        // Modify deprecated VehicleDynamics Library
        // Remove documentation
        // Modify deprecated VehicleDynamics Library
        // Begin: Move from ModelicaAdditions to Standard Modelica Library
        Modelica.Mechanics.MultiBody.Forces.WorldForce tyreForceWheelCenter annotation(
            Placement(visible = true, transformation(origin = {-74, -38}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Forces.WorldTorque tyreTorqueWheelCenter annotation(
            Placement(visible = true, transformation(origin = {-68, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Interfaces.Frame_a carrierFrame annotation(
            Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a driveShaft1D annotation(
            Placement(visible = true, transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.Rotational.Sources.Torque drivingTyreTorque annotation(
            Placement(visible = true, transformation(origin = {-54, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Joints.Revolute hub(n = {0, 1, 0}, useAxisFlange = true) annotation(
            Placement(visible = true, transformation(origin = {-22, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        Modelica.Mechanics.MultiBody.Parts.Body wheel_body(m = wheelData.m, I_11 = wheelData.Ixx, I_22 = wheelData.Iyy, I_33 = wheelData.Ixx) annotation(
            Placement(visible = true, transformation(origin = {52, 2}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        // Ende
    protected
        Real aux1[3] "Auxiliary vector 1 (vector perpendicular to Ce_y and We_z)";
        Real aux2[3] "Auxiliary vector 2";
    equation 
        road.x = Wr[1];
        road.y = Wr[2];
        //road.v = 0;
        //temporarilly
        road.mue = mueRoad;
        road.z = altitudeRoad;
        road.n = normalRoad;
        // Modify deprecated VehicleDynamics Library
        // Move connector to the end of the model
        /* Definition of road:
        1. A road frame is defined which is fixed in the inertial frame and
           which is defined relatively to the inertial frame. This frame is
           defined as:
             Sroad : Transformation matrix from road frame to inertial frame.
             rroad0: Position vector from inertial frame origin to the
                     origin of the road frame, resolved in the inertial frame.
        2. The z-axis of the road frame points upwards. The road surface
           is defined as (Wr_z, We_z) = f(Wr_x, Wr_y), where
              Wr_x, rWy, rWz are the x-, y-, z-coordinates of the wheel/road
              contact point with respect to the road frame and
              We_z is the unit vector at the contact point which is
              perpendicular to the surface and points upwards.
        Temporarily it is assumed that the road is a plane at z=0 of the inertial
        frame. Still, the equations below are written for a general road profile.
        This assumption should be replaced later.
        The friction coefficient mue of the road is temporarily set to the
        nominal friction coefficient data.mue_max of the tyre.
        The determination of the contact point of wheel and road requires
        the solution of a nonlinear system of equations. However, since the
        contact area of the tyre of a car is a rectangle with about 10 cm x 20 cm
        area and the road is approximated in the order of centimeters, it does not
        make much sense to determine the "virtual" contact point very exactly.
        Below one iteration of a fixed point iteration scheme is used to solve
        this nonlinear system of equations. This works well, provided a good
        initial guess Ce_z_0 for the unit vector Ce_z (normalized vector from
        wheel center point to contact point), is available. A good guess is
        - a unit vector which is fixed in the main car body and points
          into the z-direction of the road frame when the car is at rest.
        - a unit vector which is fixed in the wheel carrier and points
          into the z-direction of the road frame when the car is at rest.
        - the value of Ce_z at the previous integration step.
        Temporarily, this initial guess is set for the case that
        the road is a plane at z=0.
        Note, that the exact nonlinear system of equations is constructed
        by replacing the equation for Ce_z_0 below by the equation:
            Ce_z = Ce_z_0;
        and by using an appropriate initial guess for Ce_z_0.
    */
        Sroad = identity(3);
        rRoad0 = zeros(3);
        //Ce_z_0 = Ce_z;
        Ce_z_0 = {0, 0, 1};
        //mueRoad = 0.7;
        //(mueRoad,altitude) = Road(Wr[1], Wr[2], sqrt(Wv_x*Wv_x + Wv_y*Wv_y));
        /* Definition of the ISO coordinate systems of the wheel:
        Frame C (ISO-C, TYDEX C axis system) is fixed to the wheel carrier.
          - The origin is described by vector Cr, from the origin of the
            road frame to the wheel center, resolved in the road frame.
          - The x-axis (= unit vector Ce_x) is parallel to the road tangent plane
            within the wheel plane
          - The y-axis (= unit vector Ce_y) is normal to the wheel plane and
            therefore parallel to the wheels spin axis.
          - The z-axis (= unit vector Ce_z) is orthogonal to the x- and y-axis, i.e.,
            it is a unit vector along the line from the contact point to
            the wheel center point.
        Frame W (ISO-W, TYDEX W axis system) is fixed at the contact point
        of the wheel and the road.
          - The origin is described by vector Wr, from the origin of the
            road frame to the tyre contact point, resolved in the road frame.
          - The x-axis (= unit vector We_x) lies in the local road tangent plane
            along the intersection of the wheel plane and the local road plane,
            i.e., it is parallel to the x-axis of frame C.
          - The z-axis (= unit vector We_z) is perpendicular to the local
            road tangent plane and points upward.
          - The y-axis (= unit vector We_y) lies in the local road plane and
            is perpendicular to the x- and z-axis.
        All vectors describing frame C and frame W are resolved in the
        road frame. Note, that We_x = Ce_x (but We_y <> Ce_y)
    */
        /* Determine C-frame, where all vectors are resolved in the road frame.
         n: spin-axis in wheel carrier frame;
         S: transformation matrix from wheel carrier frame to road frame).
    */
        // Modify deprecated VehicleDynamics Library
        // Replace transformation matrix S to R. => S = R.T
        S = transpose(Sroad) * transpose(carrierFrame.R.T);
        nn = n / Modelica.Math.Vectors.length(n);
        Ce_x = We_x;
        Ce_y = S * nn;
        Ce_z = cross(Ce_x, Ce_y);
        Cr = transpose(Sroad) * (carrierFrame.r_0 - rRoad0);
        /* Determine W-frame, where all vectors are resolved in the road frame.
         As shortly discussed above, this is performed by one iteration of a
         fixed point iteration scheme, constructing three points P0, P1, P2:
         Point P0: Defined by position vector Wr_0 which is computed using
                   the initial guess We_z_0 for We_z.
         Point P1: P0 is usually not on the road. The x-,y-coordinates of P0
                   are used for P1. The z-coordinate is computed by using
                   the road surface function. In point P1 the contact frame W
                   is constructed. This defines especially the contact plane.
         Point P2: P1 is moved in the constructed contact plane, such that
                   it is the intersection of the contact plane with the line
                   along Ce_z, i.e., along the line from the wheel-center point
                   to the contact point (which is computed above as cross
                   product of the wheel spin axis Ce_y and of We_x computed
                   for point P1).
         rP0     : Position vector from road frame origin to point P0.
         rP1     : Position vector from road frame origin to point P1.
                   Temporarily, rP1[3] is set to 0 and We_z is set to {0,0,1}.
                   Later the road surface function should be used to calculate
                   these values.
         Rdym    : Distance between wheel center point and contact point.
         rC_W    : Position vector from C-frame origin to W-frame origin.
                   During the contact point calculation (iteration), this is
                   the vector from C-frame origin to point P2.
                     rC_W = -Rdym*Ce_z
                          = rC_P1 + a*We_x + b*We_y;
                        multiply rC_W with We_z
                     We_z*rC_W = -Rdym*We_z*Ce_z
                               = We_z*rC_P1  (We_z is perpendicular to We_x and We_y)
                        solve for Rdym
                     Rdym = -(We_z*rC_P1)/(We_z*Ce_z)
    */
        rP0 = Cr - wheelData.R0 * Ce_z_0;
        //road has to be called twice
        road2.x = rP0[1];
        road2.y = rP0[2];
        //road2.v = 0;
        rP1 = {rP0[1], rP0[2], road2.z};
        // general road: rP1[3] = f(rP1[1], rP1[2]);
        We_z = road2.n;
        //{0,0,1};
        // general road: We_z = f(rP1[1], rP1[2]);
        aux1 = cross(Ce_y, We_z);
        We_x = aux1 / Modelica.Math.Vectors.length(aux1);
        We_y = cross(We_z, We_x);
        rC_P1 = rP1 - Cr;
        cosCamberAngle = We_z * Ce_z;
        // Modify deprecated VehicleDynamics Library
        // P.46
        Rdym = -(We_z * rC_P1) / cosCamberAngle;
        rC_W = -Rdym * Ce_z;
        Wr = Cr + rC_W;
        /* Compute characteristic values of the contact:
         delta_r    : Distance between undeformed wheel point and contact point
                      R0 = Rdym + delta_r
         delta_z    : Radial tyre deflection
                      (= wheel deformation in z-direction of C-frame)
                      delta_z = delta_r if the wheel is in contact with the road
         camberAngle: Angle between wheel plane and We_z
                      (cos(pi/2 - camberAngle) = Ce_y*We_z)
        */
        delta_r = wheelData.R0 - Rdym;
        camberAngle = Modelica.Math.asin(Ce_y * We_z);
        /* Determine contact point velocities
         wWheel  : Absolute angular velocity of the rotating wheel, resolved in frame road
         ddelta_r: derivative of delta_r (= -der(Rdym))
         Cv      : Absolute velocity of wheel center point, resolved in frame road
         Wv      : Absolute velocity of contact point, resolved in frame road
                     Wv = der(Wr)
                        = der(Cr) + cross(wWheel, rC_W) + der(abs(rCW))*(-Ce_z)
                        = Cv + cross(wWheel, rC_W) + der(R0 - delta_r)*(-Ce_z)
                        = Cv + cross(wWheel, rC_W) + der(delta_r)*Ce_z
                     Wv has to be in the road plane. Therefore We_z*Wv = 0
                       0 = We_z*Wv = We_z*(Cv + cross(wWheel, rC_W)) + We_z*Ce_z*ddelta_r
                       ddelta_r = -We_z*(Cv + cross(wWheel, rC_W)) / (We_z*Ce_z)
         Wv_x  : Absolute velocity of contact point in longitudinal direction
                 = We_x*Wv
                 = We_x*(Cv + cross(wWheel, rC_W) + ddelta_r*Ce_z)
                 = We_x*(Cv + cross(wWheel, rC_W))
                 since We_x*Ce_z = Ce_x*Ce_z = 0
         Wv_y  : Absolute velocity of contact point in lateral direction
                 = We_y*Wv
                 = We_y*(Cv + cross(wWheel, rC_W) + ddelta_r*Ce_z)
         Wv_z  : Absolute velocity of contact point in normal direction
                 = 0
        */
        // Modify deprecated VehicleDynamics Library
        // Replace transformation matrix S to R. => S = R.T
        RwCarrier = S * Modelica.Mechanics.MultiBody.Frames.angularVelocity2(carrierFrame.R);
        wWheel = RwCarrier + w * Ce_y;
        Carrierv = der(carrierFrame.r_0);
        Cv = S * Modelica.Mechanics.MultiBody.Frames.resolve2(carrierFrame.R, Carrierv);
        aux2 = Cv + cross(wWheel, rC_W);
        ddelta_r = -We_z * aux2 / cosCamberAngle;
        Wv_x = We_x * aux2;
        Wv_y = We_y * (aux2 + ddelta_r * We_z);
        //DETERMINE THE WHEEL CONTACT POINT VELOCITY
        v_x = Cv * We_x;
        v_y = Cv * We_y;
        //STATES OF WHEEL ROTATION
        // Modify deprecated VehicleDynamics Library
        // Rename Variable in Std. Lib: qd -> w, q -> phi
        w = hub.w;
        phi = hub.phi; 
        //EXPRESSIONS FOR F_x, F_y, F_z, M_x, M_y, M_z AT EACH WHEEL MODEL
        Wf = We_x * F_x + We_y * F_y + We_z * F_z;
        Wt = We_x * M_x + We_y * M_y + We_z * M_z;
        // tyre forces shall act at wheel center point
        // S2: Transformation matrix from road frame to frame_b
        // Modify deprecated VehicleDynamics Library
        // Replace transformation matrix S to R. => S = R.T
        S2 = hub.frame_a.R.T * Sroad;
        Ct = S2 * (cross(rC_W, Wf) + Wt);
        Ct_tyre = nn * Ct;
        // Modify deprecated VehicleDynamics Library
        // .inPort.signal to force, torque and tau
        tyreForceWheelCenter.force = S2 * Wf;
        tyreTorqueWheelCenter.torque = Ct - nn * Ct_tyre;
        drivingTyreTorque.tau = Ct_tyre;
        // Modify deprecated VehicleDynamics Library
// Collect all connectors
        connect(tyreForceWheelCenter.frame_b, carrierFrame) annotation(
            Line(points = {{-64, -38}, {-57, -38}, {-57, -26}, {-58, -26}, {-58, 0}, {-100, 0}}, color = {95, 95, 95}));
        connect(tyreTorqueWheelCenter.frame_b, carrierFrame) annotation(
            Line(points = {{-58, -62}, {-48, -62}, {-48, 0}, {-100, 0}}));
        connect(carrierFrame, hub.frame_a) annotation(
            Line(points = {{-100, 0}, {-32, 0}}));
        connect(hub.frame_b, wheel_body.frame_a) annotation(
            Line(points = {{-12, 0}, {14, 0}, {14, 2}, {42, 2}}, color = {95, 95, 95}));
        connect(drivingTyreTorque.flange, hub.axis) annotation(
            Line(points = {{-44, 40}, {-22, 40}, {-22, 10}}));
        connect(hub.axis, driveShaft1D) annotation(
            Line(points = {{-22, 10}, {0, 10}, {0, 100}}));
            annotation(
            Diagram);
    end BaseWheel;
    
    package RillTyre
        record NominalLoad "Tyre force data for nominal load"
            import SI = Modelica.SIunits;
            /* Definition of tyre force characteristic
                     Note, that the following requirements must be fulfilled:
                           Fds_x >= 2*Fmax_x / s_Fmax_x
                           Fds_y >= 2*Fmax_y / s_Fmax_y
                     in order that there is no turning point for 0 < s <= s_Fmax
                     Also: 0.1 <= dL0 <= 1/6
            */
            parameter SI.Force Fz_nom = 3000 "Nominal normal force";
            parameter SI.Force Fds_x = 50000 "Slope at s_x=0 in longitudinal direction";
            parameter Real s_max_x = 0.15 "Slip of maximum longitudinal tyre force";
            parameter SI.Force F_max_x = 3000 "maximum longitudinal tyre force";
            parameter Real s_slide_x = 0.4 "Slip of sliding begin";
            parameter SI.Force F_slide_x = 2800 "Longitudinal tyre force at sliding begin";
            parameter SI.Force Fds_y = 40000 "Slope at s_y=0 in lateral direction";
            parameter Real s_max_y = 0.21 "Slip of maximum lateral tyre force";
            parameter SI.Force F_max_y = 2750 "maximum lateral tyre force";
            parameter Real s_slide_y = 0.6 "Slip of sliding begin";
            parameter SI.Force F_slide_y = 2500 "Lateral tyre force at sliding begin";
            parameter Real dL0 = 0.133 "(distance contact point to f_y) / (length of contact area) for zero lateral slip";
            parameter Real s0 = 0.25 "Lateral slip where dL=0 for the first time (usually, a bit higher as s_Fmax_y)";
            parameter Real sE = 0.6 "Lateral slip where dL remains zero for higher slips";
        end NominalLoad;

        record Data "Tyre data"
            extends Wheels.Data;
      // Spring and damping characteristic of tyre
            parameter Real c_x(unit = "N/m") = 100000 "tyre spring constant in x-direction";
            parameter Real c_y(unit = "N/m") = 150000 "tyre spring constant in y-direction";
            parameter Real c_z(unit = "N/m") = 200000 "tyre spring constant in z-direction";
            parameter Real d_x(unit = "N.s/m") = 1500 "tyre damping constant in x-direction";
            parameter Real d_y(unit = "N.s/m") = 300 "tyre damping constant in y-direction";
            parameter Real d_z(unit = "N.s/m") = 300 "tyre damping constant in z-direction";
            // Data for nominal vertical load 1
            parameter Real mue_nom = 0.7 "friction coefficient of road for which tyre data is valid";
            NominalLoad load1 annotation(
                extent = [-85, 15; -15, 85]);
            NominalLoad load2(Fz_nom = 6000, Fds_x = 75000, s_max_x = 0.18, F_max_x = 4500, s_slide_x = 0.5, F_slide_x = 4200, Fds_y = 60000, s_max_y = 0.24, F_max_y = 4125, s_slide_y = 0.8, F_slide_y = 3750, dL0 = 0.1, s0 = 0.275, sE = 0.8) annotation(
                extent = [15, 15; 85, 85]);
            annotation(
                Coordsys(extent = [-100, -100; 100, 100], grid = [2, 2], component = [20, 20]),
                Window(x = 0.4, y = 0.4, width = 0.6, height = 0.6));
        end Data;

        block TyreForces "Compute tyre forces and torques for steady state conditions"
            import SI = Modelica.SIunits;
            // Compute tire force in contact plane
            // Compute longitudinal and lateral tire forces
            // Compute attachment point of f_y with respect to contact length (dL)
            input Real slip;
            input Real sinphi "sin(phi), where phi is the angle between -Wv_x and -Wv_y, where Wv_x, Wv_y are the contact point velocities";
            input Real cosphi "cos(phi)";
            input SI.Force f_z "Load in normal direction";
            input Real mueRoad "Coefficient of friction of road";
            input SI.Angle camberAngle;
            output SI.Force f0 "Tyre force in contact plane for steady state conditions";
            output SI.Force f_x0 "Tyre force in x-direction for steady state conditions";
            output SI.Force f_y0 "Tyre force in y-direction for steady state conditions";
            output SI.Force df_ds "Derivative of steady state contact force in contact plane with respect to slip";
            output Real dL "(distance contact point to f_y) / (length of contact area); used to compute t_z";
        protected
            SI.Force f_z1;
            SI.Force f_z2;
            SI.Force Fds_x;
            SI.Force Fds_y;
            SI.Force F_max_x;
            SI.Force F_max_y;
            SI.Force F_slide_x;
            SI.Force F_slide_y;
            SI.Force Fds;
            SI.Force F_max;
            SI.Force F_slide;
            Real s_max_x;
            Real s_max_y;
            Real s_slide_x;
            Real s_slide_y;
            Real s_max;
            Real s_slide;
            Real k_mue;
            Real sigma;
            Real dL0;
            Real s0;
            Real sE;
            Real s_y;
            Real c;
        public
            parameter Data data "Data of tyre" annotation(
                extent = [-80, 40; -40, 80]);
        equation
// (f0,f_x0,f_y0,df_ds,dL) = RillTyre.TyreForces2(data, slip, sinphi,
//  cosphi, f_z, mueRoad, camberAngle);
            k_mue = mueRoad / data.mue_nom;
// Compute data of force slip curve, according to Rill
            f_z1 = data.load1.Fz_nom;
            f_z2 = data.load2.Fz_nom;
            Fds_x = Deprecated.interpolate2(f_z, f_z1, data.load1.Fds_x, f_z2, data.load2.Fds_x);
            Fds_y = Deprecated.interpolate2(f_z, f_z1, data.load1.Fds_y, f_z2, data.load2.Fds_y);
            F_max_x = k_mue * Deprecated.interpolate2(f_z, f_z1, data.load1.F_max_x, f_z2, data.load2.F_max_x);
            F_max_y = k_mue * Deprecated.interpolate2(f_z, f_z1, data.load1.F_max_y, f_z2, data.load2.F_max_y);
            F_slide_x = k_mue * Deprecated.interpolate2(f_z, f_z1, data.load1.F_slide_x, f_z2, data.load2.F_slide_x);
            F_slide_y = k_mue * Deprecated.interpolate2(f_z, f_z1, data.load1.F_slide_y, f_z2, data.load2.F_slide_y);
            s_max_x = k_mue * Deprecated.interpolate1(f_z, f_z1, data.load1.s_max_x, f_z2, data.load2.s_max_x);
            s_max_y = k_mue * Deprecated.interpolate1(f_z, f_z1, data.load1.s_max_y, f_z2, data.load2.s_max_y);
            s_slide_x = k_mue * Deprecated.interpolate1(f_z, f_z1, data.load1.s_slide_x, f_z2, data.load2.s_slide_x);
            s_slide_y = k_mue * Deprecated.interpolate1(f_z, f_z1, data.load1.s_slide_y, f_z2, data.load2.s_slide_y);
            Fds = noEvent(max([sqrt((Fds_x * cosphi) ^ 2 + (Fds_y * sinphi) ^ 2); 1.e-10]));
            s_max = noEvent(max([sqrt((s_max_x * cosphi) ^ 2 + (s_max_y * sinphi) ^ 2); 1.e-10]));
            F_max = noEvent(max([sqrt((F_max_x * cosphi) ^ 2 + (F_max_y * sinphi) ^ 2); 1.e-10]));
            s_slide = noEvent(max([sqrt((s_slide_x * cosphi) ^ 2 + (s_slide_y * sinphi) ^ 2); 1.0001 * s_max]));
            F_slide = noEvent(max([sqrt((F_slide_x * cosphi) ^ 2 + (F_slide_y * sinphi) ^ 2); 1.e-10]));
        // Compute tire force in contact plane
            if noEvent(slip <= s_max) then
                sigma = slip / s_max;
                c = noEvent(sigma * (s_max / max([F_max, 1.e-10]) * Fds - 2 + sigma) + 1);
                f0 = s_max * Fds * sigma / c;
                df_ds = Fds * (1 - sigma) / (c * c);
            elseif noEvent(slip <= s_slide) then
                sigma = (slip - s_max) / (s_slide - s_max);
                c = 0;
                f0 = F_max - (F_max - F_slide) * sigma * sigma * (3 - 2 * sigma);
                df_ds = -(F_max - F_slide) * 6 * sigma * (1 - sigma) / (s_slide - s_max);
            else
                sigma = 0;
                c = 0;
                f0 = F_slide;
                df_ds = 0;
            end if;
        // Compute longitudinal and lateral tire forces
            f_x0 = f0 * cosphi;
            f_y0 = f0 * sinphi + f_z * tan(camberAngle);
        // Compute attachment point of f_y with respect to contact length (dL)
            dL0 = Deprecated.interpolate1(f_z, f_z1, data.load1.dL0, f_z2, data.load2.dL0);
            s0 = Deprecated.interpolate1(f_z, f_z1, data.load1.s0, f_z2, data.load2.s0);
            sE = Deprecated.interpolate1(f_z, f_z1, data.load1.sE, f_z2, data.load2.sE);
            s_y = slip * sinphi;
            dL = noEvent(if abs(s_y) <= s0 then dL0 * (1 - abs(s_y) / s0) else if abs(s_y) <= sE then -dL0 * (abs(s_y) - s0) / s0 * ((sE - abs(s_y)) / (sE - s0)) ^ 2 else 0);
            annotation(
                Coordsys(extent = [-100, -100; 100, 100], grid = [2, 2], component = [20, 20]),
                Window(x = 0.11, y = 0, width = 0.88, height = 0.84));
        end TyreForces;

        model Wheel
            import SI = Modelica.SIunits;
            extends BaseWheel(final wheelData(R0 = tyreData.R0, width = tyreData.width, m = tyreData.m, Iyy = tyreData.Iyy, Ixx = tyreData.Ixx), wheel_body.animation = false);
            //SLIP DEFINITION
            SI.Angle slipAngle "Angle between wheel axis velocity and the x-direction of the wheel";
            Real slip "slip at contact point computed with contact velocities";
            Real slip_limited "slip limited in the range -1.1 for plotting";
            Real sinphi "sin(phi), where phi is the angle between -Wv_x and -Wv_y, where Wv_x, Wv_y are the contact point velocities";
            Real cosphi "cos(phi)";
            SI.Velocity Wv_abs "Absolute value of contact point velocity";
            SI.Velocity Rw "denominator of slip quotient";
            //TYRE DEFLECTION
            SI.Position delta_x "Longitudinal tyre deflection (= wheel deformation in x-direction of W-frame)";
            SI.Position delta_y "Lateral tyre deflection (= wheel deformation in y-direction of W-frame)";
            SI.Position delta_z "Radial tyre deflection (= wheel deformation in z-direction of W-frame)";
            parameter SI.Position delta_z_offset = 1e-3 "In order to make the z-computations less non-linear";
            //TYRE CONTACT AND FORCE GENERATION
            Boolean contact "true, if wheel has contact with road";
            Real mueRoad;
            parameter Real delta_eps = 0.1 "should be about 0.1 ... 1; see explanation in model";
            SI.Force f_x "Tyre/contact force at origin of W-frame in x-direction";
            SI.Force f_y "Tyre/contact force at origin of W-frame in y-direction";
            SI.Force f_z "Tyre/contact force at origin of W-frame in z-direction";
            SI.Force f_x0 "f_x at steady state conditions";
            SI.Force f_y0 "f_y at steady state conditions";
            SI.Force f0 "Absolute value of tyre/contact force in contact plane at steady state conditions";
            SI.Force df_ds "Derivative of steady state contact force in contact plane with respect to slip";
            SI.Force t_z "Tyre/contact torque at W-frame in z-direction";
            Real dL "(distance contact point to f_y) / (length of contact area); used to compute t_z";
            Real dfx_dvx1 "Partial derivative of f_x with respect to Wv_x";
            Real dfy_dvy1 "Partial derivative of f_y with respect to Wv_y";
            Real dfx_dvx "dfx_dvx1 where potential positive values are set to zero";
            Real dfy_dvy "dfy_dvy1 where potential positive values are set to zero";
            Real Lcontact "length of contact area in x-direction";
            Real delta_diff;
            Real delta_diff_pos;
            Real delta_x_scaled "delta_x scaled on nominal value";
            Real delta_y_scaled "delta_y scaled on nominal value";
            Real S_rel[3, 3];
            parameter Data tyreData annotation(
                extent = [-80, 60; -60, 80]);
            TyreForces tyre(data = tyreData) annotation(
                extent = [14, -68; 34, -48]);
            Modelica.Mechanics.MultiBody.Visualizers.FixedShape Tire(extra = 1.0,height = 2 * wheelData.R0, length = abs(wheelData.width), lengthDirection = nShape, shapeType = "cylinder", width = 2 * wheelData.R0, widthDirection = {1, 0, 0})  annotation(
                Placement(visible = true, transformation(origin = {52, -36}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
        protected
            parameter Real delta_x_nom = tyreData.load1.F_slide_x / tyreData.c_x "nominal tyre deflection in x-direction";
            parameter Real delta_y_nom = tyreData.load1.F_slide_y / tyreData.c_y "nominal tyre deflection in y-direction";
        equation
//SLIP DEFINITION
//slipAngle = Modelica.Math.atan2(Cv[2], noEvent(abs(Cv[1])));
            slipAngle = Modelica.Math.atan2(Wv_y, Rw);
//also possible slipAngle=Modelica.Math.atan2(Wv_y,Wv_x);
/* Determine slip
       Wv_abs: Absolute value of contact point velocity (= sqrt(Wv*Wv))
       phi   : Angle between longitudinal and lateral contact point velocity
               Assumption: For Wv_abs small -> Wv_x = Wv_abs, Wv_y = 0, i.e.,
                           velocity only in longitudinal direction.
       sinphi: sin(phi) (= 0 for Wv_abs = 0)
       cosphi: cos(phi) (= -1 for Wv_abs = 0)
*/
            Wv_abs = noEvent(max([sqrt(Wv_x * Wv_x + Wv_y * Wv_y); 1.e-10]));
            Rw = wheelData.R0 * abs(w) + 0.1;
            slip = Wv_abs / Rw;
            slip_limited = noEvent(if slip > 1 then 1 else if slip < (-1) then -1 else slip);
            if noEvent(Wv_abs > v_eps) then
                sinphi = -Wv_y / Wv_abs;
                cosphi = -Wv_x / Wv_abs;
            else
                sinphi = 0;
                cosphi = -1;
            end if;
        //TYRE FORCES AND TORQUES
            delta_z = if delta_r >= 0 then delta_r else 0;
        // Compute  force perpendicular to contact plane.
            f_z = if delta_r >= 0 then tyreData.c_z * delta_z + tyreData.d_z * ddelta_r else 0;
        // Compute steady state tyre forces in x- and y-direction
        // (f0,f_x0,f_y0,df_ds,dL) = RillTyre.TyreForces(tyreData, slip, sinphi,
        //  cosphi, f_z, mueRoad, camberAngle);
            tyre.slip = slip;
            tyre.sinphi = sinphi;
            tyre.cosphi = cosphi;
            tyre.f_z = f_z;
            tyre.mueRoad = mueRoad;
            tyre.camberAngle = camberAngle;
            tyre.f0 = f0;
            tyre.f_x0 = f_x0;
            tyre.f_y0 = f_y0;
            tyre.df_ds = df_ds;
            tyre.dL = dL;
        // Compute data for tyre deflection differential equations
            if noEvent(Wv_abs > v_eps) then
                dfx_dvx1 = -(df_ds * slip * cosphi * cosphi + f0 * sinphi * sinphi) / Wv_abs;
                dfy_dvy1 = -(df_ds * slip * sinphi * sinphi + f0 * cosphi * cosphi) / Wv_abs;
            else
                dfx_dvx1 = -df_ds / Rw;
                dfy_dvy1 = dfx_dvx1;
            end if;
            dfx_dvx = if dfx_dvx1 <= 0 then dfx_dvx1 else 0;
            dfy_dvy = if dfy_dvy1 <= 0 then dfy_dvy1 else 0;
/* Differential equations for tyre deflections in x- and y-direction
         If the wheel angular velocity is too small, the prerequisites for the
         validity of the tyre force calculation are violated and the equations
         give wrong results. In such a case it is assumed that the profil
         particles do no longer get their maximum deflection. If only
         longitudinal slip is present, the deflection delta_x is calculated
         by "der(delta_x) = -Wv_x" (which is the definition of the deflection delta_x).
         However, delta_x has a maximum which is reached when the particle leaves
         the contact area. If the velocity is constant, a particle remains
         "T = Lcontact / (R0*abs(w))" seconds in the contact area, where Lcontact
         is the length of the contact area, R0 is the undeformed wheel radius
         and w is the angular velocity of the wheel. Therefore, the maximum
         deflection is
               delta_x_max = -Wv_x*T
                           = -Wv_x*Lcontact/(R0*abs(w))
                           = slip*Lcontact.
         Therefore, the estimation is used that the simplified differential
         equation is used as long as deflection is less then "delta_eps*slip*Lcontact"
         where delta_eps is choosen appropriately (e.g., delta_eps=0.1;
         note that delta_eps=1 gives the theoretical switching condition
         derived by simplified assumptions).
      */
            contact = delta_r >= 0;
            Lcontact = noEvent(sqrt(8 * tyreData.R0 * (delta_z + delta_z_offset)) - sqrt(8 * tyreData.R0 * delta_z_offset));
            delta_diff = noEvent(sqrt(delta_x * delta_x + delta_y * delta_y) - abs(delta_eps * slip * Lcontact));
            delta_diff_pos = noEvent(if delta_diff >= 0 or Rw >= 1 then 1 else 0);
            der(delta_x_scaled) = (if not contact then 0 else if noEvent(delta_diff_pos > 0.5) then (f_x0 - tyreData.c_x * delta_x) / (tyreData.d_x - dfx_dvx) else -Wv_x) / delta_x_nom;
            der(delta_y_scaled) = (if not contact then 0 else if noEvent(delta_diff_pos > 0.5) then (f_y0 - tyreData.c_y * delta_y) / (tyreData.d_y - dfy_dvy) else -Wv_y) / delta_y_nom;
            delta_x = delta_x_scaled * delta_x_nom;
            delta_y = delta_y_scaled * delta_y_nom;
            when change(contact) then
// Tyre looses contact to ground or gets contact to ground
                reinit(delta_x_scaled, 0);
                reinit(delta_y_scaled, 0);
            end when;
// Compute dynamic tyre forces in x- and y-direction and tyre torque in z-direction
            f_x = tyreData.c_x * delta_x + tyreData.d_x * der(delta_x);
            f_y = tyreData.c_y * delta_y + tyreData.d_y * der(delta_y);
            t_z = -Lcontact * dL * f_y;
//FORCES AND MOMENTS REQUIRED BY THE BaseWheel
            F_x = f_x;
            F_y = f_y;
            F_z = f_z;
            M_x = 0;
//this model is yet unable to generate overturning moment
            M_y = 0;
//and roll resistance torque.
            M_z = t_z;
//VISUAL REPRESENTATION OF TYRE AND RIM
            S_rel = [nn] * transpose([nn]) + (identity(3) - [nn] * transpose([nn])) * cos(phi) - skew(nn) * sin(phi);
            connect(hub.frame_b, Tire.frame_a) annotation(
                Line(points = {{-12, 0}, {42, 0}, {42, -36}, {42, -36}}));
        end Wheel;
    end RillTyre;

    package Tests "Tests tyre implementation"
            model RillTyreBasic
                RillTyre.Wheel wheel annotation(
                    Placement(visible = true, transformation(origin = {30, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
                inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1})  annotation(
                    Placement(visible = true, transformation(origin = {-82, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
                Modelica.Mechanics.Rotational.Sources.Torque torque annotation(
                    Placement(visible = true, transformation(origin = {-26, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
                Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic annotation(
                    Placement(visible = true, transformation(origin = {-34, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
                Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 0, 0.299}) annotation(
                    Placement(visible = true, transformation(origin = {-78, -18}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
                Modelica.Blocks.Sources.Ramp ramp(duration = 2, height = 10) annotation(
                    Placement(visible = true, transformation(origin = {-82, 74}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
            equation
                connect(prismatic.frame_b, wheel.carrierFrame) annotation(
                    Line(points = {{-24, 20}, {20, 20}, {20, 20}, {20, 20}}, color = {95, 95, 95}));
                connect(world.frame_b, fixedTranslation.frame_a) annotation(
                    Line(points = {{-72, -70}, {-88, -70}, {-88, -18}, {-88, -18}}, color = {95, 95, 95}));
                connect(fixedTranslation.frame_b, prismatic.frame_a) annotation(
                    Line(points = {{-68, -18}, {-44, -18}, {-44, 20}, {-44, 20}}));
                connect(torque.flange, wheel.driveShaft1D) annotation(
                    Line(points = {{-16, 74}, {30, 74}, {30, 30}, {30, 30}}));
                connect(ramp.y, torque.tau) annotation(
                Line(points = {{-71, 74}, {-38, 74}}, color = {0, 0, 127}));
            end RillTyreBasic;
        end Tests;
    
end Wheels;