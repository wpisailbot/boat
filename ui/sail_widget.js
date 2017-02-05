var quaternionQueue = "boat_state.orientation.";
var sailState = "boat_state.internal.sail";
var rudderState = "boat_state.internal.rudder";
var quaternion = {w: 1, x: 0, y: 0, z: 0};

function quaternionToEuler(quaternion) {
  var w = quaternion.w;
  var x = quaternion.x;
  var y = quaternion.y;
  var z = quaternion.z;
  var ysqr = y * y;
  var angles = {};

  // roll (x-axis rotation)
  var t0 = 2.0 * (w * x + y * z);
  var t1 = 1.0 - 2.0 * (x * x + ysqr);
  angles.roll = Math.atan2(t0, t1);

  // pitch (y-axis rotation)
  var t2 = 2.0 * (w * y - z * x);
  t2 = t2 > 1.0 ? 1.0 : t2;
  t2 = t2 < -1.0 ? -1.0 : t2;
  angles.pitch = Math.asin(t2);

  // yaw (z-axis rotation)
  var t3 = 2.0 * (w * z + x * y);
  var t4 = 1.0 - 2.0 * (ysqr + z * z);
  angles.yaw = Math.atan2(t3, t4);
  return angles;
}

function setRotation(id, angle) {
  $("#"+id).attr("transform", "rotate(" + Math.round(angle * 180 / Math.PI) + ")");
}

function quaternionListener(component) {
  quaternion[component] = fields[quaternionQueue + component].value;
  setRotation("hull_rotate", -quaternionToEuler(quaternion).yaw);
}

function sailListener() {
  setRotation("sail_path", -fields[sailState].value);
}

function rudderListener() {
  setRotation("rudder_path", -fields[rudderState].value);
}

function initializeBoatHandlers() {
  addHandler(quaternionQueue + "w", function() { quaternionListener("w"); });
  addHandler(quaternionQueue + "x", function() { quaternionListener("x"); });
  addHandler(quaternionQueue + "y", function() { quaternionListener("y"); });
  addHandler(quaternionQueue + "z", function() { quaternionListener("z"); });
  addHandler(sailState, sailListener);
  addHandler(rudderState, rudderListener);
}
