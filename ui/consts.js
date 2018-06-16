function checkAndSendZeros() {
  var winch_0_pot = parseFloat($("#zeroing_consts-winch_0_pot").val());
  var winch_90_pot = parseFloat($("#zeroing_consts-winch_90_pot").val());
  var rudder_zero = parseFloat($("#zeroing_consts-rudder_zero").val());
  var ballast_zero = parseFloat($("#zeroing_consts-ballast_zero").val());
  var winch_out_angle = parseFloat($("#zeroing_consts-winch_out_angle").val());
  if (isNaN(winch_0_pot) || isNaN(winch_90_pot) || isNaN(rudder_zero) || isNaN(ballast_zero)) {
    alert("All field should be filled in");
    return;
  }
  if (rudder_zero > 180 || rudder_zero < 0) {
    alert("Rudder value should be within [0, 180] and probably within [80, 120]");
    return;
  }
  if (winch_0_pot < 0 || winch_0_pot > 1023) {
    alert("Winch 0 value should be within [0, 1023]");
    return;
  }
  if (winch_90_pot < 0 || winch_90_pot > 1023) {
    alert("Winch 90 value should be within [0, 1023]");
    return;
  }
  var diff = Math.abs(winch_0_pot - winch_90_pot);
  if (diff > 300) {
    if (!confirm("The difference between 0 and 90 seems large. Do you want to continue?")) {
      return;
    }
  }
  if (!confirm("Changing these numbers can be dangerous. continue?")) {
    return;
  }
  var queue = "zeroing_consts";
  var msg = {};
  msg["winch_0_pot"] = winch_0_pot;
  msg["winch_90_pot"] = winch_90_pot;
  msg["rudder_zero"] = rudder_zero;
  msg["ballast_zero"] = ballast_zero;
  msg["winch_out_angle"] = winch_out_angle;
  sendMessage(queue, msg);
}
