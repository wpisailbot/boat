close all

phi = 0;  %heel angle
phi_d = 0.1;
phi_dd = 0;
gamma = 0;  %arm angle
gamma_d = 0;
gamma_dd = 0;


MBMotorStallTorquekgcm = 100; % kg-cm
MBMotorStallTorque = MBMotorStallTorquekgcm*2.205/2.54; % in-lbf
MBMotorStallCurrent = 28; % Amps
MBMotorFreeCurrent = 5; % Amps
MBMotorFreeSpeed = 86; % rpm

heelAngle = -pi/2:pi/32:pi/2; % deg
deviationFromCenter = -pi/4:pi/64:pi/4; %deg
load = 15; % lbf


stage1 = 16/64;
stage2 = 16/64;
stage1orig = 18/52;
stage2orig = 18/52;

dtheta = pi/64;
heelRange = -32:1:32;
centerRange = -16:1:16;
armMax = 16;

dtime = 0.02;

ka = 30; %pendulum force
ks = 3; %fluid resistance to heel acceleration

ku = 3; %voltage effect on arm acceleration
kf = 3; %frictional resistance to arm acceleration

voltageInput = 0;


for t = 1:dtime:10  %simulate for 30 seconds at 0.1 timestep

       
       
       phi_dd = -ka * phi*dtheta - ks * phi_d*dtheta + calcMBRightingMoment(phi*dtheta, gamma*dtheta, load);;
       gamma_dd = ku * voltageInput * kf * gamma_d + calcMBTorque(phi*dtheta, gamma*dtheta, stage1, stage2, load);
       
%        torque = calcMBTorque(heel*dtheta, centerDev*dtheta, stage1, stage2, load);
%        pctTauMax = abs(torque)/MBMotorStallTorque;
%        rpmAtLoad = (1-pctTauMax)*MBMotorFreeSpeed*stage1*stage2; % rpm
%        I = pctTauMax*(MBMotorStallCurrent-MBMotorFreeCurrent)+MBMotorFreeCurrent;
%        rightingMoment(heel+33,centerDev+17) = calcMBRightingMoment(heel*dtheta, centerDev*dtheta, load);

       phi = phi + dtime * phi_d;  %step forward 
       phi_d = phi_d + dtime * phi_dd;
       gamma = gamma + dtime * gamma_d; 
       gamma_d = gamma_d + dtime*gamma_dd;
       
       if gamma > armMax*dtheta
           gamma = armMax;
       end
       if gamma < -armMax*dtheta
           gamma = -armMax*dtheta;
       end
       
       
       %animate
       Pm = 5*[sin(phi),cos(phi)];
       Pb = -3*[sin(gamma),cos(gamma)];
       
       axis(gca,'equal');
       axis([-3,3,-5,7]);
       
       mast = line([0,Pm(1)],[0,Pm(2)]);
       ballast = line([0,Pb(1)],[0,Pb(2)]);
       
       
       %dispay for time
       pause(dtime)
       
       
       delete(mast);
       delete(ballast);
       
end







% tau_m = calcMBTorque(-pi/2, 0, stage1, stage2, load);
%
% pctTauMax = abs(tau_m)/MBMotorStallTorque
%
% maxI = pctTauMax*(MBMotorStallCurrent-MBMotorFreeCurrent)+MBMotorFreeCurrent % Amps
% rpmAtLoad = (1-pctTauMax)*MBMotorFreeSpeed % rpm
% traverseSpeed = rpmAtLoad*stage1*stage2 % rpm


