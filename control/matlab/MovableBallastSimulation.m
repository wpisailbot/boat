function MovableBallastSimulation(x0, phigoal)
global Vmax ka ks kssq ku kf J Jmb rightingweight;
global Kfast Kslow pgoal;
pgoal = phigoal;
stage1 = 16/64;
stage2 = 16/64;
G = stage1 * stage2;
Vmax = 12;

mbmass = 10;
J = 12 * 1.2^2; % kg * m^2
Jmb = mbmass * .7^2; % kg * m^2
MBMotorStallTorque = 9.8 / G; % N-m
MBMotorStallCurrent = 28; % Amps
MBMotorFreeCurrent = 5; % Amps
MBMotorFreeSpeed = 86 * 2 * pi / 60 * G; % rad / sec
MBmaxAngle = 45;

rightingweight = mbmass * 9.8; % N

ka = 120 * 1.2 / J; %torque of keel when heeled at 90 deg (1 / s^2)
%ka = 60 * 1.2;
% drag of keel
% Multiplying by radius thrice: twice for rotation->real velocity; once for force->torque
% 0.5 * rho * area * C(=1) * u_0(=nominal velocity=nom rot vel * nom radius) * nom radius * nom radius / J
% = kg / m^3 * m^2 * m / s * m^2 / (kg * m^2) = 1 / sec
nom_rad = 0.8;
nom_vel = 1.0;
kssq = 0.5 * 1000 * 0.2 * 1 * nom_rad^3 / J;
ks = kssq * nom_vel;

% V = IR + omega / Kv
% alpha = KtI / J
% alpha = Kt (V - omega / Kv) / (R * J)
% ku = Kt / (R * J)
% kf = Kt / (Kv * R * J)
% Kt = stall_torque * J / stall_cur
% R = nominal_volts / stall_cur
% Kv = free_speed / (nominal_volts - free_cur * R)
Kt = MBMotorStallTorque * Jmb / MBMotorStallCurrent;
R = Vmax / MBMotorStallCurrent;
Kv = MBMotorFreeSpeed / (Vmax - MBMotorFreeCurrent * R);
ku = Kt / (R * Jmb); %voltage effect on arm acceleration
kf = Kt / (Kv * R * Jmb); %frictional resistance to arm acceleration

xgoal = desx(phigoal)
xnaught = xgoal;
xnaught(1) = -0.5;
[A, B, ~, Aslow, Bslow, ~] = linsys(xnaught)
Aslow
Bslow
% For this, xslow = [phi, phidot, phiddot], uslow = [phidddot]
Aslow = [0 1 0;
         0 0 1;
         0 0 0];
Bslow = [0; 0; 1];
ctr = rank(ctrb(A, B))

[vecA, eigA] = eig(A)
[vecAslow, eigAslow] = eig(Aslow)
K = lqr(A, B, diag([100.1 10.0 1.0 1.0]), [1e0])
Kfast = 15.1;
Kslow = lqr(Aslow, Bslow, diag([100 100 10]), [0.1])
%Kslow = place(Aslow, Bslow, [-10, -2, -3])
%Kslow(1) = 0;
%Kslow = [0 -0.1 -0.00]
eigABK = eig(A - B * K)
[vecABKslow, eigABKslow] = eig(Aslow - Bslow * Kslow)
f = @(t, x) full_dyn(x, utrans(x, K * (xgoal - x)), t);
%fsplit = @(t, x) full_dyn(x, utrans(x, Kfast * (Kslow * (xgoal([1 3 4]) - x([1 3 4])) - x(2))), t);
%f = @(t, x) simple_dyn(x, K * (xgoal - x));
%f = @(t, x) A * x + B * K * (xgoal - x);
%[ts, xs] = ode45(f, [0 30], x0);
%[ts, xs] = ode45(fsplit, [0 10], x0);
[ts, xs] = ode45(@call_flin_ctrl, [0 3], x0);

uprimes = K * (repmat(xgoal, 1, length(ts)) - xs');
gammadotdes = Kslow * (repmat(xgoal([1 3 4]), 1, length(ts)) - xs(:, [1 3 4])');
uprimes = Kfast * (gammadotdes - xs(:, 2)');

jerks = [];
phiddots = [];
for i = 1:numel(ts)
  [xdot, jerk, gdot, gddot, ~] = fully_flin_control(ts(i), xs(i, :));
  jerks(i) = jerk;
  gammadotdes(i) = gdot;
  uprimes(i) = gddot;
  phiddots(i) = xdot(4);
end

subplot(221);
plot(ts, [xs(:, [1 2]) gammadotdes']);
ylim([-1.5, 1.5])
legend('\gamma', 'gammadot', 'gammadotdes');
subplot(222);
plot(ts, [xs(:, [3 4]), phiddots']);
legend('\phi', 'phidot', 'phiaccel');
subplot(223);
plot(ts, [uprimes' jerks']);
legend('gammaddot', 'jerks');
title('Control Inputs');
subplot(224);
for i = 1:length(ts)
  u(i) = max(min(utrans(xs(i, :), uprimes(i)), Vmax), -Vmax);
end
plot(ts, u);
title('u');

end

% Compute righting moment for given phi/gamma
function tau = MBRight(phi, gamma)
  global rightingweight;
  [tau, ~, ~] = calcMBRightingMoment(phi, gamma, rightingweight);
end

% Compute motor load from ballast for given phi/gamma
function tau = MBTorque(phi, gamma)
  global rightingweight;
  % Stage values are dealt with later...
  tau = calcMBTorque(phi, gamma, 1, 1, rightingweight);
end

function xdot = call_flin_ctrl(t, x)
  [xdot, ~, ~, ~, ~] = fully_flin_control(t, x);
end

function [xdot, phijerk, gammadot_des, gammaddot, u] = fully_flin_control(t, x)
  global Kfast Kslow pgoal;
  % Extract phiddot so that we can work with it.
  xdot = full_dyn(x, 0, t, 0);
  gamma = x(1);
  gammadot = x(2);
  phi = x(3);
  phidot = x(4);
  phiddot = xdot(4);

  phigoal = pgoal;
  if t > 20
    phigoal = -pgoal;
  end
  phijerk = Kslow * ([phigoal; 0; 0] - [phi; phidot; phiddot]);
  gammadot_des = gammainv(gamma, [phi; phidot; phiddot; phijerk]);
  % If we just want to try feed-forwardsing it.
  %gammadot_des = 1 * (sign(phigoal) - gamma);
  gammaddot = Kfast * (gammadot_des - gammadot);
  u = utrans(x, gammaddot);
  xdot = full_dyn(x, u, t, 0);
end

% State vector is of form [gamma, gammadot, phi, phidot]
% Full dynamics, with u as motor voltage
function xdot = full_dyn(x, u, t, do_jerk)
  global ka ks kssq ku kf J Jmb Vmax;
  u = max(min(u, Vmax), -Vmax);
  gamma = x(1);
  gammadot = x(2);
  phi = x(3);
  phidot = x(4);
  D0 = 100;
  D = D0;
  Ddot = 0;
  if t > 20 && t < 21
    D = D0 * 2 * (20.5 - t);
    Ddot = -2 * D0;
  elseif t > 21
    D = -D0;
  end
  D = D * cos(phi);
  Ddot = Ddot * cos(phi) - D * sin(phi) * phidot;
  phiddot = -ka * sin(phi) - kssq * phidot * abs(phidot) + (MBRight(phi, gamma) + D) / J;
  phijerk = calcjerk(x, D, Ddot);
  gammaddot = ku * u - kf * gammadot + MBTorque(phi, gamma) / Jmb;
  if gamma > pi / 2
    gammadot = min(gammadot, 0);
  elseif gamma < -pi / 2
    gammadot = max(gammadot, 0);
  end
  xdot = [gammadot; gammaddot; phidot; phiddot];
  if exist('do_jerk') && do_jerk == 1
    xdot = [xdot; phijerk];
  end
end

% Compute dynamics, but with uprime (gammaddot) instead of u
function xdot = simple_dyn(x, uprime)
  global ka ks ku kf J;
  gamma = x(1);
  gammadot = x(2);
  phi = x(3);
  phidot = x(4);
  D = 00;
  phiddot = -ka * sin(phi) - ks * phidot + (MBRight(phi, gamma) + D) / J;
  gammaddot = uprime;
  xdot = [gammadot; gammaddot; phidot; phiddot];
end

% Compute u at some given x and desired gammaddot
function u = utrans(x, gammaddot_des)
  global ka ks ku kf Jmb;
  gamma = x(1);
  gammadot = x(2);
  phi = x(3);
  u = (gammaddot_des - MBTorque(phi, gamma) / Jmb + kf * gammadot) / ku;
end

function phijerk = calcjerk(x, D, Ddot)
  global ka rightingweight kssq J;
  gamma = x(1);
  gammadot = x(2);
  phi = x(3);
  phidot = x(4);
  [R, dRdphi, dRdgamma] = calcMBRightingMoment(phi, gamma, rightingweight);
  phiaccel = -ka * sin(phi) - kssq * phidot * abs(phidot) + (R + D) / J;
  phijerk = -ka * cos(phi) * phidot - 2 * kssq * abs(phidot) * phiaccel + (dRdphi * phidot + dRdgamma * gammadot + Ddot) / J;
end

% Compute appropriate gammadot for a given phidddot
% phis = [phi, phidot, phiddot, phidddot]
function gammadot = gammainv(gamma, phis)
  global ka rightingweight kssq J;
  phi = phis(1);
  phidot = phis(2);
  phiaccel = phis(3);
  phijerk = phis(4);
  % phiddot = -ka * sin(phi) - kssq * phidot * abs(phidot) + (MBRight(phi, gamma) + D) / J;
  % phijerk = -ka cos(phi) phidot - 2 kssq abs(phidot) phiddot + (dR/dphi phidot + dR/dgamma gammadot + Ddot) / J
  % gammadot = ((phijerk + ka cos(phi) phidot + 2 kssq abs(phidot) phiddot) J - Ddot - dR/dphi phidot) / (dR/dgamma)
  % Note that for dR/dgamma near zero, explodes
  [~, dRdphi, dRdgamma] = calcMBRightingMoment(phi, gamma, rightingweight);
  % Clip the edges and prevent singularities
  % TODO(james): Prevent going past max righting moment points.
  dRdgamma = min(dRdgamma, -0.01);
  % Ddot = 0
  gammadot = ((phijerk + ka * cos(phi) * phidot + 2 * kssq * abs(phidot) * phiaccel) * J - 0 - dRdphi * phidot) / dRdgamma;
end

% Compute linearized system as function of uprime
function [A, B, c, Aslow, Bslow, cslow] = linsys(x0);
  global ka ks ku kf J;
  % Start with clearly linear terms, then figure out MBRight
  % Note that we account for gammaddot in uprime, so zero out kf.
  A = [0 1 0 0;
       0 0 0 0;
       0 0 0 1;
       0 0 -ka -ks];
  B = [0; 1; 0; 0];
  % Compute constant term (c)
  c = simple_dyn(x0, 0);

  Aright = jacobian(@(y) MBRight(y(1), y(2)), x0([3 1]));
  A(4, [3 1]) = A(4, [3 1]) + Aright;

  % The "slow" components of the system, where we
  % now treat gammadot as an input.
  slow = [1 3 4];
  Aslow = A(slow, slow);
  Bslow = [1; 0; 0];
  cslow = c(slow);
end

% Compute needed x for a desired phi
% u will be zero at equilibrium because gammaddot=0.
function [x] = desx(phi)
  foptions = optimoptions('fsolve', 'MaxFunctionEvaluations', 100000, 'Display', 'Off', 'Algorithm', 'levenberg-marquardt');
  gamma = fsolve(@(gam) simple_dyn([gam; 0; phi; 0], 0), 0, foptions);
  x = [gamma; 0; phi; 0];
end

% Invert the computation of the movable ballast righting moment
function [gamma] = invright(desright, curphi)
  global rightingweight
  % TODO: Do this cleanly
  gamma = fsolve(@(gam) calcMBRightingMoment(curphi, gamma, rightingweight), 0, optimset('Display', 'off'));
end
