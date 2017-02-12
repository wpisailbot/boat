var quaternionQueue = "sim_true_boat_state.orientation.";
var sailState = "sim_true_boat_state.internal.sail";
var rudderState = "sim_true_boat_state.internal.rudder";
var positionQueue = "sim_true_boat_state.pos";
var waypointsQueue = "waypoints";
var inertialFrame = "inertial_frame";
var quaternion = {w: 1, x: 0, y: 0, z: 0};
// Variables for transforming between coordinate systems.
var inertialFramePos = {x: 500, y: 500};
var scale = 4;

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

function toInertialSvgCoords(pos) {
  return {x: scale * pos.x, y: scale * -pos.y};
}

function svgInertialToBoat(pos) {
  return {x: pos.x / scale, y: -pos.y / scale};
}

function svgMainToInertial(pos) {
  return {x : -inertialFramePos.x + pos.x, y : -inertialFramePos.y + pos.y};
}

function setRotation(id, angle) {
  $("#"+id).attr("transform", "rotate(" + Math.round(angle * 180 / Math.PI) + ")");
}

function setTranslate(id, x, y) {
  $("#"+id).attr("transform", "translate(" + x + "," + y + ")");
}

function quaternionListener(component) {
  quaternion[component] = fields[quaternionQueue + component].value;
  setRotation("hull_rotate", -quaternionToEuler(quaternion).yaw);
}

function boatPositionListener() {
  var pos = fields[positionQueue].value;
  pos = toInertialSvgCoords(pos);
  setTranslate("hull_position", pos.x, pos.y);
}

function sailListener() {
  setRotation("sail_path", -fields[sailState].value);
}

function rudderListener() {
  setRotation("rudder_path", -fields[rudderState].value);
}

function waypointsListener() {
  var points = fields[waypointsQueue].value.points;
  var id_base = "waypoint_marker";
  var minx = points[0].x;
  var miny = points[0].y;
  var maxx = minx;
  var maxy = miny;
  var i = 0;
  for (i in points) {
    var id = id_base + i;
    var desc = i + "(" + points[i].x + "," + points[i].y + ")";
    var pos = toInertialSvgCoords(points[i]);
    if (document.getElementById(id) == null) {
      document.getElementById(inertialFrame).innerHTML +=
          "<text id='" + id + "' x='" + pos.x + "' y='" + pos.y +
          "'>" + desc + "</text>";
    }
    $("#"+id).attr("x", pos.x);
    $("#"+id).attr("y", pos.y);
    $("#"+id).html(desc);
    minx = Math.min(minx, points[i].x);
    miny = Math.min(miny, points[i].y);
    maxx = Math.max(maxx, points[i].x);
    maxy = Math.max(maxy, points[i].y);
  }
  ++i;
  // Clear out any previous waypoints.
  for (; i < 10; ++i) {
    var id = id_base + i;
    $("#"+id).remove();
  }
  var boatx = fields[positionQueue].value.x;
  var boaty = fields[positionQueue].value.y;
  minx = Math.min(minx, boatx);
  miny = Math.min(miny, boaty);
  maxx = Math.max(maxx, boatx);
  maxy = Math.max(maxy, boaty);

  // Provide a buffer.
  minx -= 5;
  miny -= 5;
  maxx += 5;
  maxy += 5;
  var svgWidth = $("#myCanvas").width();
  var svgHeight = $("#myCanvas").height();
  // Give us a bit of padding for the case of really near waypoints.
  var dx = Math.max(maxx - minx, 50);
  var dy = Math.max(maxy - miny, 50);
  var scalex = svgWidth / dx;
  var scaley = svgHeight / dy;
  scale = Math.min(scalex, scaley); // Avoid running one coordinate off the edge of the screen
  inertialFramePos.x = -(minx + maxx) / 2 * scale + svgWidth / 2;
  inertialFramePos.y = (miny + maxy) / 2 * scale + svgHeight / 2;
  setTranslate(inertialFrame, inertialFramePos.x, inertialFramePos.y);
}

function clicked(evt){
  var e = evt.target;
  var dim = e.getBoundingClientRect();
  var x = evt.clientX - dim.left;
  var y = evt.clientY - dim.top;
  var simPos = svgInertialToBoat(svgMainToInertial({x : x, y : y}));
  gotoWaypoint(simPos.x, simPos.y, false);
}

// restart = whether to add on waypoint or to restart from scratch.
function gotoWaypoint(x, y, restart) {
  var boatx = fields[positionQueue].value.x;
  var boaty = fields[positionQueue].value.y;
  var points;
  if (restart) {
    points = [ {x : boatx, y : boaty}, {x : x, y : y} ];
  } else {
    points = fields[waypointsQueue].value.points.concat([{x : x, y : y}]);
  }
  var waypoints = {
    restart : restart,
    points : points,
  };
  sendMessage(waypointsQueue, waypoints);
}

function submitWaypoints() {
  var x = parseFloat($("#goto-x").val());
  var y = parseFloat($("#goto-y").val());
  gotoWaypoint(x, y, true);
}

function initializeBoatHandlers() {
  $("#background").on("click", clicked);
  $("#goto-submit").on("click", submitWaypoints);
  addHandler(quaternionQueue + "w", function() { quaternionListener("w"); });
  addHandler(quaternionQueue + "x", function() { quaternionListener("x"); });
  addHandler(quaternionQueue + "y", function() { quaternionListener("y"); });
  addHandler(quaternionQueue + "z", function() { quaternionListener("z"); });
  addHandler(sailState, sailListener);
  addHandler(rudderState, rudderListener);
  addHandler(positionQueue, boatPositionListener);
  addHandler(waypointsQueue, waypointsListener);
}
