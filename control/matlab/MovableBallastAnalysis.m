close all

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
tau = zeros(length(heelRange),length(centerRange));
tauOrig = zeros(length(heelRange),length(centerRange));
rightingMoment = zeros(length(heelRange),length(centerRange));

rpm = zeros(length(heelRange),length(centerRange));
rpmOrig = zeros(length(heelRange),length(centerRange));

current = zeros(length(heelRange),length(centerRange));

maxI = 0;
maxInomrange = 0;
timeOrig = 0;
time = 0;

for heel=heelRange
   for centerDev=centerRange
       torque = calcMBTorque(heel*dtheta, centerDev*dtheta, stage1, stage2, load);
       pctTauMax = abs(torque)/MBMotorStallTorque;
       rpmAtLoad = (1-pctTauMax)*MBMotorFreeSpeed*stage1*stage2; % rpm
       tau(heel+33,centerDev+17) = abs(torque);
       rpm(heel+33,centerDev+17) = rpmAtLoad;

       I = pctTauMax*(MBMotorStallCurrent-MBMotorFreeCurrent)+MBMotorFreeCurrent;
       current(heel+33,centerDev+17) = I;
       if I > maxI
           maxI = I;
       end

       if I > maxInomrange
           if abs(heel*dtheta) < deg2rad(30) && abs(centerDev*dtheta) < deg2rad(55)
               maxInomrange = I;
           end
       end

       rightingMoment(heel+33,centerDev+17) = calcMBRightingMoment(heel*dtheta, centerDev*dtheta, load);
   end
end



for heel=heelRange
   for centerDev=centerRange
       torque = calcMBTorque(heel*dtheta, centerDev*dtheta, stage1orig, stage2orig, load);
       pctTauMax = abs(torque)/MBMotorStallTorque;
       rpmAtLoad = (1-pctTauMax)*MBMotorFreeSpeed*stage1orig*stage2orig; % rpm
       tauOrig(heel+33,centerDev+17) = abs(torque);
       rpmOrig(heel+33,centerDev+17) = rpmAtLoad;
   end
end

heelIdx = round((-pi/6)/dtheta+33);
centerDevStartIdx = round(-pi/4/dtheta+17);
centerDevEndIdx = round(pi/4/dtheta+17);

centerDevRange = centerDevStartIdx:centerDevEndIdx;

for centerDevIdx=centerDevRange
   dtime = dtheta/(rpm(heelIdx, centerDevIdx)*2*pi/60);
   dtimeOrig = dtheta/(rpmOrig(heelIdx, centerDevIdx)*2*pi/60);
   time = time + dtime;
   timeOrig = timeOrig + dtimeOrig;
end

figure
newsurf = surf(centerRange*dtheta, heelRange*dtheta, tau, 'FaceColor', [255,100,0]/255);
hold on
oldsurf = surf(centerRange*dtheta, heelRange*dtheta, tauOrig, 'FaceColor', [100,255,0]/255);
xlabel('Angle from center (rad)')
ylabel('Heel angle (rad)')
zlabel('Motor Torque (in-lbf)')
legend([newsurf, oldsurf], {'11.56:1 Reduction', '8.35:1 Reduction'})

figure
newrpmsurf = surf(centerRange*dtheta, heelRange*dtheta, rpm, 'FaceColor', [255,100,0]/255);
hold on
oldrpmsurf = surf(centerRange*dtheta, heelRange*dtheta, rpmOrig, 'FaceColor', [100,255,0]/255);
xlabel('Angle from center (rad)')
ylabel('Heel angle (rad)')
zlabel('Arm speed (rpm)')
legend([newrpmsurf, oldrpmsurf], {'11.56:1 Reduction', '8.35:1 Reduction'})

figure
currentsurf = surf(centerRange*dtheta, heelRange*dtheta, current, 'FaceColor', [255,100,0]/255);
xlabel('Angle from center (rad)')
ylabel('Heel angle (rad)')
zlabel('Current Draw (Amps)')

figure
momentsurf = surf(centerRange*dtheta, heelRange*dtheta, rightingMoment, 'FaceColor', [255,255,100]/255);
xlabel('Angle from center (rad)')
ylabel('Heel angle (rad)')
zlabel('Righting Moment')

disp(maxI)
disp(maxInomrange)
disp(time)
disp(timeOrig)

% tau_m = calcMBTorque(-pi/2, 0, stage1, stage2, load);
%
% pctTauMax = abs(tau_m)/MBMotorStallTorque
%
% maxI = pctTauMax*(MBMotorStallCurrent-MBMotorFreeCurrent)+MBMotorFreeCurrent % Amps
% rpmAtLoad = (1-pctTauMax)*MBMotorFreeSpeed % rpm
% traverseSpeed = rpmAtLoad*stage1*stage2 % rpm


