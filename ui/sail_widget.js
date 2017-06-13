var boatState = "boat_state"
var quaternionQueue = boatState + ".orientation.";
var sailState = boatState + ".internal.sail";
var rudderState = boatState + ".internal.rudder";
var positionQueue = boatState + ".pos";
var waypointsQueue = "waypoints";
var inertialFrame = "inertial_frame";
var quaternion = {w: 1, x: 0, y: 0, z: 0};
// Variables for transforming between coordinate systems.
var inertialFramePos = {x: 500, y: 500};
var posTranslate = {x: 0, y: 0};
var scale = 4;

function normalizeAngle(a) {
  var tau = 2 * Math.PI;
  if (a > Math.PI) {
    a -= Math.floor(a / tau + .5) * tau;
  } else if (a < -Math.PI) {
    a -= Math.floor(a / tau - .5) * tau;
  }
  return a;
}

function toRad(x) {
  return x * Math.PI / 180;
}

function GPSDistance(lat1, lon1, lat2, lon2) {
  lat1 = toRad(lat1);
  lat2 = toRad(lat2);
  lon1 = toRad(lon1);
  lon2 = toRad(lon2);
  a = Math.pow(Math.sin((lat2 - lat1) / 2), 2) +
      Math.cos(lat1) * Math.cos(lat2) *
        Math.pow(Math.sin((lon2 - lon1) / 2), 2);
  c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
  R = 6.371e6; // Earth's radius
  d = R * c;
  return d;
}

function GPSBearing(lat1, lon1, lat2, lon2) {
  lat1 = toRad(lat1);
  lat2 = toRad(lat2);
  lon1 = toRad(lon1);
  lon2 = toRad(lon2);
  y = Math.sin(lon2 - lon1) * Math.cos(lat2);
  x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(lon2 - lon1);
  return Math.atan2(x, y);
}

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
  return {x: scale * (pos.x - posTranslate.x), y: scale * (posTranslate.y - pos.y)};
}

function svgInertialToBoat(pos) {
  return {x: pos.x / scale + posTranslate.x, y: posTranslate.y - pos.y / scale};
}

function svgMainToInertial(pos) {
  return {x : -inertialFramePos.x + pos.x,
          y : -inertialFramePos.y + pos.y};
}

function setRotation(id, angle) {
  $("."+id).attr("transform", "rotate(" + Math.round(angle * 181 / Math.PI) + ")");
}

function setTranslate(id, x, y) {
  $("."+id).attr("transform", "translate(" + x + "," + y + ")");
}

function setGridScale(id, x, y) {
  $("#"+id).attr("width", x);
  $("#"+id).attr("height", y);
  $("#"+id).children("path")[0].setAttribute("d", "M " + x + " 0 L 0 0 0 " + y);
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

var vectorInc = 0;
function addVector(frame, color, x, y, xyQueue, angleQueue, lenQueue, scale) {
  // Draws a vector of color "color" in svg frame with id=frame,
  // with the base of the vector starting at (x, y), pointing
  // along (xyQueue.x * scale, xyQueue.y * scale)
  var vecId = "vector" + vectorInc;
  vectorInc++;

  function makeSVG(angle, len) {
    len *= scale;
    angle *= 180 / Math.PI;
    var vecTranslate = '<g transform="translate(' + x + ',' + y + ')">';
    var vecRotate = '<g transform="rotate(' + angle + ')">';
    var vecPath = '<path d="M 0 0 L ' + len + ' 0" stroke="' + color +
                  '" stroke-width=4 />' +
                  '<path d="M 0 -5 L 0 5 L 15 0 z" fill="' + color +
                  '" transform="translate(' + len + ',0)" />';
    return vecTranslate + vecRotate + vecPath + '</g></g>';
  }

  $("#" + frame).html($("#" + frame).html() + '<g id="' + vecId + '">' +
                      makeSVG(0, 3) + '</g>');

  // Add queue listeners
  if (xyQueue != null) {
    addHandler(xyQueue, function(vec) {
      var angle = Math.atan2(-vec.y, vec.x);
      var len = Math.sqrt(vec.y * vec.y + vec.x * vec.x);
      $("#" + vecId).html(makeSVG(angle, len));
    });
  } else if (angleQueue != null && lenQueue != null) {
    var angle = 0;
    var len = 0;
    if (isNaN(angleQueue)) {
      addHandler(angleQueue, function(a) {
        angle = -normalizeAngle(a);
        $("#" + vecId).html(makeSVG(angle, len));
      });
    } else {
      angle = -normalizeAngle(angleQueue);
    }
    if (isNaN(lenQueue)) {
      addHandler(lenQueue, function(l) {
        len = l;
        $("#" + vecId).html(makeSVG(angle, len));
      });
    } else {
      len = lenQueue;
    }
  }
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
      var iframe = $("." + inertialFrame);
      iframe.html(iframe.html() + "<text id='" + id + "' x='" + pos.x +
                  "' y='" + pos.y + "'>" + desc + "</text>");
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
  minx -= .0001;
  miny -= .0001;
  maxx += .0001;
  maxy += .0001;
  var svgWidth = $("#myCanvas").width();
  var svgHeight = $("#myCanvas").height();
  // Give us a bit of padding for the case of really near waypoints.
  var dx = Math.max(maxx - minx, .00001);
  var dy = Math.max(maxy - miny, .00001);
  var scalex = svgWidth / dx;
  var scaley = svgHeight / dy;
  scale = Math.min(scalex, scaley); // Avoid running one coordinate off the edge of the screen
  posTranslate.x = (minx + maxx) / 2;
  posTranslate.y = (miny + maxy) / 2;
  inertialFramePos.x = svgWidth / 2;
  inertialFramePos.y = svgHeight / 2;
  setTranslate(inertialFrame, inertialFramePos.x, inertialFramePos.y);

  var lat_per_meter = 1e-5 / GPSDistance(miny, minx, miny + 1e-5, minx);
  $("#gps_lat_to_m").html(lat_per_meter.toFixed(10));
  var lon_per_meter = 1e-5 / GPSDistance(miny, minx, miny, minx + 1e-5);
  $("#gps_lon_to_m").html(lon_per_meter.toFixed(10));

  var gridxscale = 10 * lon_per_meter.toFixed(8);
  var gridyscale = 10 * lat_per_meter.toFixed(8);
  var gridx = gridxscale * Math.round((posTranslate.x - inertialFramePos.x / scale) / gridxscale);
  var gridy = gridyscale * Math.round((posTranslate.y + inertialFramePos.y / scale) / gridyscale);
  gridx -= gridxscale;
  gridy += gridyscale;
  var inertialgrid = toInertialSvgCoords({x: gridx, y: gridy});
  setTranslate("grid_trans", inertialgrid.x, inertialgrid.y);
  setGridScale("smallGrid", gridxscale * scale, gridyscale * scale);
}

function clicked(evt){
  var e = evt.target;
  var dim = e.getBoundingClientRect();
  var x = evt.clientX - dim.left;
  var y = evt.clientY - dim.top;
  var simPos = svgInertialToBoat(svgMainToInertial({x : x, y : y}));
  // If CTRL is pressed, reset waypoints
  gotoWaypoint(simPos.x, simPos.y, evt.shiftKey);
}

// restart = whether to add on waypoint or to restart from scratch.
function gotoWaypoint(x, y, restart) {
  var boatx = fields[positionQueue].value.x;
  var boaty = fields[positionQueue].value.y;
  var points;
  if (restart || fields[waypointsQueue].value == null) {
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

function submitRelativeWaypoint() {
  var pos = fields[positionQueue].value;
  var lat = pos.y;
  var lon = pos.x;
  var lat_per_meter = 1e-5 / GPSDistance(lat, lon, lat + 1e-5, lon);
  var lon_per_meter = 1e-5 / GPSDistance(lat, lon, lat, lon + 1e-5);
  var x = parseFloat($("#goto-rel-x").val()) * lon_per_meter + lon;
  var y = parseFloat($("#goto-rel-y").val()) * lat_per_meter + lat;

  gotoWaypoint(x, y, true);
}

function initializeBoatHandlers() {
  $("#background").on("click", clicked);
  $("#goto-submit").on("click", submitWaypoints);
  $("#goto-rel-submit").on("click", submitRelativeWaypoint);
  addHandler(quaternionQueue + "w", function() { quaternionListener("w"); });
  addHandler(quaternionQueue + "x", function() { quaternionListener("x"); });
  addHandler(quaternionQueue + "y", function() { quaternionListener("y"); });
  addHandler(quaternionQueue + "z", function() { quaternionListener("z"); });
  addHandler(sailState, sailListener);
  addHandler(rudderState, rudderListener);
  addHandler(positionQueue, boatPositionListener);
  addHandler(waypointsQueue, waypointsListener);

  addVector("demo_hull_loc", "green", 0, 0, "boat_state.vel", null, null, 15);
  addVector("demo_hull_loc", "orange", 0, 0, "true_wind", null, null, 15);
  addVector("demo_hull_loc", "purple", 0, 0, null, "heading_cmd.heading", 10, 1);
}
