function mb_sim(x0, phigoal)
global Vmax ka ks ku kf J Jmb rightingweight;
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
ks = 500 * 0.25 / J; % drag of keel (rho/2 * area / J -- should end up as N * m * sec / (rad * kg * m^2)= 1 / sec)
ks = 10 * ks

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
ctr = rank(ctrb(A, B))
[vecA, eigA] = eig(A)
[vecAslow, eigAslow] = eig(Aslow)
K = lqr(A, B, diag([100.1 10.0 1.0 1.0]), [1e0])
Kfast = 5.1;
Kslow = lqr(Aslow, Bslow, diag([0.0 10.0 1.0]), [1e1])
Kslow(1) = 0;
eigABK = eig(A - B * K)
eigABKslow = eig(Aslow - Bslow * Kslow)
f = @(t, x) full_dyn(x, utrans(x, K * (xgoal - x)), t);
fsplit = @(t, x) full_dyn(x, utrans(x, Kfast * (Kslow * (xgoal([1 3 4]) - x([1 3 4])) - x(2))), t);
%f = @(t, x) simple_dyn(x, K * (xgoal - x));
%f = @(t, x) A * x + B * K * (xgoal - x);
%[ts, xs] = ode45(f, [0 30], x0);
[ts, xs] = ode45(fsplit, [0 200], x0);

uprimes = K * (repmat(xgoal, 1, length(ts)) - xs');
gammadotdes = Kslow * (repmat(xgoal([1 3 4]), 1, length(ts)) - xs(:, [1 3 4])');
uprimes = Kfast * (gammadotdes - xs(:, 2)');

subplot(221);
plot(ts, [xs(:, [1 2]) gammadotdes']);
legend('\gamma', 'gammadot', 'gammadotdes');
subplot(222);
plot(ts, xs(:, [3 4]));
legend('\phi', 'phidot');
subplot(223);
plot(ts, uprimes');
legend('gammaddot');
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
  tau = calcMBRightingMoment(phi, gamma, rightingweight);
end

% Compute motor load from ballast for given phi/gamma
function tau = MBTorque(phi, gamma)
  global rightingweight;
  % Stage values are dealt with later...
  tau = calcMBTorque(phi, gamma, 1, 1, rightingweight);
end

% State vector is of form [gamma, gammadot, phi, phidot]
% Full dynamics, with u as motor voltage
function xdot = full_dyn(x, u, t)
  global ka ks ku kf J Jmb Vmax;
  u = max(min(u, Vmax), -Vmax);
  gamma = x(1);
  gammadot = x(2);
  phi = x(3);
  phidot = x(4);
  D = 50;
  if t > 100
    D = 0;
  end
  D = D * cos(phi);
  phiddot = -ka * sin(phi) - ks * phidot + (MBRight(phi, gamma) + D) / J;
  gammaddot = ku * u - kf * gammadot + MBTorque(phi, gamma) / Jmb;
  if gamma > pi / 2
    gammadot = min(gammadot, 0);
  elseif gamma < -pi / 2
    gammadot = max(gammadot, 0);
  end
  xdot = [gammadot; gammaddot; phidot; phiddot];
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
