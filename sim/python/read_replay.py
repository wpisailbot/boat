import numpy as np

def norm_rad(theta):
  while (theta > np.pi):
    theta -= 2 * np.pi
  while (theta < -np.pi):
    theta += 2 * np.pi
  return theta

def norm_deg(theta):
  while (theta > 180.):
    theta -= 360.
  while (theta < -180.):
    theta += 360.
  return theta

def norm_ang(theta, degrees):
  return norm_deg(theta) if degrees else norm_rad(theta)

def read(fname, degrees=False, starttime=0, endtime=9999):
  data = np.genfromtxt(fname, delimiter=',')
  x = []
  y = []
  vx = []
  vy = []
  speed = []
  t = []
  yaw = []
  heel = []
  pitch = []
  omega = []
  heading = []
  leeway = []
  sail = []
  rudder = []
  alphaw = []
  pitchvar = []
  truewx = []
  truewy = []
  wind_speed = []
  true_alphaw = []
  true_wind_speed = []
  heading_cmd = []
  rudder_mode = []
  mult = (180. / np.pi) if degrees else 1.0
  for row in data:
    if row[0] < starttime:
      continue
    elif row[0] > endtime:
      break
    for i in range(len(row)):
      if abs(row[i]) > 1e5:
        row[i] = float("nan")
    t.append(row[0])
    sail.append(row[3] * mult)
    rudder.append(row[4] * mult)
    yaw.append(norm_rad(row[5]) * mult)
    heel.append(norm_rad(row[6]) * mult)
    pitch.append(norm_rad(row[7]) * mult)
    omega.append(row[8] * 10.0 * mult) # *10 due to error in old data
    pitchvarstart = max(-100, -len(pitch))
    pitchvar.append(np.std(pitch[pitchvarstart:]))
    x.append(row[9])
    y.append(row[10])
    vx.append(row[11])
    vy.append(row[12])
    speed.append(np.hypot(vx[-1], vy[-1]))
    heading.append(np.arctan2(vy[-1], vx[-1]) * mult)
    leeway.append(norm_ang(heading[-1] - yaw[-1], degrees))
    alphaw.append(np.arctan2(-row[2], -row[1]) * mult)
    wind_speed.append(np.sqrt(row[1] ** 2 + row[2] ** 2))
    truewx.append(row[13])
    truewy.append(row[14])
    true_alphaw.append(norm_rad(np.arctan2(-row[14], -row[13])) * mult)
    true_wind_speed.append(np.sqrt(row[13] ** 2 + row[14] ** 2))
    heading_cmd.append(row[16] * mult)
    rudder_mode.append(row[17] * 10)

  return (t, x, y, vx, vy, speed, yaw, heel, pitch, omega, heading,
      leeway, sail, rudder, alphaw, pitchvar, wind_speed, true_alphaw,
      true_wind_speed, heading_cmd, rudder_mode, truewx, truewy)
