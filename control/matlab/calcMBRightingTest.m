function rightingMoment = calcMBRightingTest(heel, theta)
  % I wrote this to confirm the work in calcMBRightingMoment.
  % Both files appear to be correct.
  shaftElev = deg2rad(28.6);
  verticalShaftOffset = 7.21;
  % Start with the location of the ballast in the frame of
  % the ballast arm.
  % Origin = at the shaft,
  % X-axis = straight out along arm
  % Y-axis = in plane of rotation, orthogonal to X
  % Z-axis = Striaght up along shaft
  p_load = [25.25; 0; -7.9];
  % Transform out theta:
  % Origin = At point of MB rotation
  % X-axis = Straight out along arm, if theta = 0 (forwards+up on boat)
  % Y-axis = Straight to Port along deck
  % Z-axis = Straight up along shaft
  R_arm_shaft = [cos(theta) -sin(theta) 0;
                 sin(theta) cos(theta)  0;
                 0          0           1];
  p_shaft = R_arm_shaft * p_load;
  % Transform out tilt
  % Origin = At point of MB rotation
  % X-axis = Straight forwards along deck
  % Y-axis = Straight to Port along deck
  % Z-axis = Straight up parallel to mast
  R_shaft_deck = [cos(shaftElev) 0 -sin(shaftElev);
                  0              1 0;
                  sin(shaftElev) 0 cos(shaftElev)];
  p_deck = R_shaft_deck * p_shaft;
  % Transform down to waterline
  % Origin = Immediately below MB rotation at the
  % axis about which the boat heels.
  % X-axis = Straight forwards along deck
  % Y-axis = Straight to Port along deck
  % Z-axis = Straight up parallel to mast
  p_boat = p_deck + [0; 0; verticalShaftOffset];

  % Transform out heel
  % Origin = At boat center
  % X-axis = Straight forwards
  % Y-axis = Level with water, straight to port
  % Z-axis = Vertical
  R_boat_world = [1 0         0;
                  0 cos(heel) -sin(heel);
                  0 sin(heel) cos(heel)];
  p_world = R_boat_world * p_boat;

  % The portion of p_world along the Y-axis is the righting moment.
  rightingMoment = -p_world(2);
end
