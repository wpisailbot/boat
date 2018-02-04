function [ rightingMoment ] = calcMBRightingMoment( heelAngle, deviationFromCenter, load )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
MBShaftElevation = deg2rad(28.6); % deg
verticalShaftOffset = 7.21 * .0254; % m

tf_BoatCenterline_Heeled = calcTransform(0, heelAngle,0, pi/2);
tf_Heeled_ShaftBase = calcTransform(verticalShaftOffset, 0, 0, MBShaftElevation);
tf_ShaftBase_ArmCS = calcTransform(0, deviationFromCenter, 0, 0);

tf_WCS_ArmCS = tf_BoatCenterline_Heeled*tf_Heeled_ShaftBase*tf_ShaftBase_ArmCS;

armInWCS = tf_WCS_ArmCS*[0; 25.25; -7.9; 1] * .0254;

rightingMoment = armInWCS(1)*load;

end

