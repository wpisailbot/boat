close all
clc;
clear all;

phi_0= 10;  %initial heel angle degrees
phi_d_0 = 0; 

gamma_0 = 0;  %initial arm angle degrees
gamma_d_0 = 0;



MBMotorStallTorquekgcm = 100; % kg-cm
MBMotorStallTorque = MBMotorStallTorquekgcm*2.205/2.54; % in-lbf
MBMotorStallCurrent = 28; % Amps
MBMotorFreeCurrent = 5; % Amps
MBMotorFreeSpeed = 86; % rpm
MBmaxAngle = 45;

load = 22; % lbf


stage1 = 16/64;
stage2 = 16/64;
stage1orig = 18/52;
stage2orig = 18/52;

dtime = 0.02;
stopTime = 2;

ka = 120 * 1.2; %torque of keel (N * m)
ks = (500 * 0.25); % drag of keel (rho/2 * area)
j_phi = 120* 1.25^2; %inertia of keel

ku = 120; %voltage effect on arm acceleration
kf = 120; %frictional resistance to arm acceleration
j_gamma = load * 4.45 * 1.0^2; %inertia of balast

voltageInput = 0;


numPoints = 33;
moment = zeros(numPoints,numPoints);

p = -pi/6:pi/96:pi/6;
g = -pi/4:pi/64:pi/4;
pfit = zeros((numPoints^2),1);
gfit = zeros((numPoints^2),1);
mfit = zeros((numPoints^2),1);
tfit = zeros((numPoints^2),1);

for i = 1:1:numPoints
    for j = 1:1:numPoints
        index = (i-1)*numPoints + j;
        pfit(index,1) = p(i);
        gfit(index,1) = g(j);
        mfit(index,1) = calcMBRightingMoment(p(i),g(j),load);
        tfit(index,1) = calcMBTorque(p(i),g(j),stage1,stage2,load);
    end
end
momentApprox = fit([pfit,gfit],mfit,'poly11');
torqueApprox = fit([pfit,gfit],tfit,'poly11');

coeffs = coeffvalues(momentApprox);
moment_phi = coeffs(2);
moment_gamma = coeffs(3);

coeffs = coeffvalues(torqueApprox);
torque_phi = coeffs(2);
torque_gamma = coeffs(3);

% hold on
% figure
% plot(momentApprox,[pfit,gfit],mfit)
% ylabel('Angle from center (rad)')
% xlabel('Heel angle (rad)')
% zlabel('Righting Moment')
% 
% figure
% plot(torqueApprox,[pfit,gfit],tfit)
% ylabel('Angle from center (rad)')
% xlabel('Heel angle (rad)')
% zlabel('Motor Torque')


syms gamma phi gamma_d phi_d ;
phi_dd =( - (ka + moment_phi) * phi - ks * phi_d + moment_gamma * gamma)/j_phi;
gamma_dd = (-kf * gamma_d + torque_phi * phi + torque_gamma * gamma)/j_gamma;


[A,B] = equationsToMatrix([phi_d,phi_dd,gamma_d,gamma_dd],[phi,phi_d,gamma,gamma_d]);
B = [0;0;0;ku];
A = double(A);
eigenValues = vpa(eig(A),3);
cont = ctrb(A,B);

Q = [10 0 0 0;
     0 1 0 0; 
     0 0 0 0; 
     0 0 0 1];
 
R = 0.1;
k = lqr(A,B,Q,R)

tspan = 0:0.1:5;
y0 = [0.1,0,0,0];
yf = [0.1;0;0;0];
[t,y] = ode45(@(t,y) A*y - k*(y - yf) , tspan, y0);

% hold on
figure
subplot(2,2,1)
plot(t,y(:,1));
xlabel('time');
ylabel('heel (rad)');

subplot(2,2,2)
plot(t,y(:,2));
xlabel('time');
ylabel('heel vel (rad/s)');

subplot(2,2,3)
plot(t,y(:,3));
xlabel('time');
ylabel('arm (rad)');

subplot(2,2,4)
plot(t,y(:,4));
xlabel('time');
ylabel('arm vel (rad/s)');


