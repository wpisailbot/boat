winchMotorStallTorque = 50; % in-lbf
winchMotorStallCurrent = 12; % Amps
winchMotorFreeCurrent = 1.6; % Amps
winchMotorFreeSpeed = 86; % rpm

drumDia = 2.5; %in
sheetLoad = 15; %lbf

travel = 38; %in

tau_m = sheetLoad*drumDia/2

pctTauMax = tau_m/winchMotorStallTorque

maxI = pctTauMax*(winchMotorStallCurrent-winchMotorFreeCurrent)+winchMotorFreeCurrent % Amps
rpmMaxLoad = (1-pctTauMax)*winchMotorFreeSpeed % rpm
speedMaxLoad = rpmMaxLoad*(drumDia*pi)/60 % in/s
traverseTime = travel/speedMaxLoad % s


