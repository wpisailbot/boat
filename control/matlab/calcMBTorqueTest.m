function tau = calcMBTorqueTest(heel, theta)
  shaftElev = 28.6 * pi / 180.0;
  grav = [0; 0; -1];
  % Start in inertial (X=North, Y=West, Z=Up)
  % Transform onto boat (X=Forward, Y=Port, Z=Up along mast)
  R_world_boat = [1 0 0;
                  0 cos(heel) sin(heel);
                  0 -sin(heel) cos(heel)];
  grav_boat = R_world_boat * grav;
  % Tilt to shaft (X=Forward/Up along tilt, Y=Port, Z=Up/Tilted back)
  R_boat_shaft = [cos(shaftElev) 0 sin(shaftElev);
                  0              1 0;
                  -sin(shaftElev) 0 cos(shaftElev)];
  grav_shaft = R_boat_shaft * grav_boat;
  % Rotate to position of MB (X=straight out on arm, Y=Orthogonal to X
  %   in plane of arm sweep, Z=along shaft)
  R_shaft_mb = [cos(theta) sin(theta) 0;
                -sin(theta) cos(theta) 0;
                0 0 1];
  grav_mb = R_shaft_mb * grav_shaft;
  % The gravity along the Y-axis will produce the torque
  armLen = 25.25;
  tau = grav_mb(2) * 25.25;

end
