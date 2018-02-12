function [ rightingMoment, dheel, dgamma ] = calcMBRightingMoment( heelAngle, deviationFromCenter, load )
% Provide the movable ballast righting moment as a function of
% heel angle and ballast shaft angle.
% Also returns derivative of righting moment with respect to heel
% and shaft angle (gamma)
MBShaftElevation = deg2rad(28.6); % deg
verticalShaftOffset = 7.21 * .0254; % m

[tf_BoatCenterline_Heeled, tf_BC_H_dheel] = calcTransform(0, heelAngle,0, pi/2);
[tf_Heeled_ShaftBase, ~] = calcTransform(verticalShaftOffset, 0, 0, MBShaftElevation);
[tf_ShaftBase_ArmCS, tf_SB_A_dgamma] = calcTransform(0, deviationFromCenter, 0, 0);

tf_WCS_ArmCS = tf_BoatCenterline_Heeled*tf_Heeled_ShaftBase*tf_ShaftBase_ArmCS;
tf_WCS_ArmCS_dheel = tf_BC_H_dheel * tf_Heeled_ShaftBase*tf_ShaftBase_ArmCS;
tf_WCS_ArmCS_dgamma = tf_BoatCenterline_Heeled*tf_Heeled_ShaftBase*tf_SB_A_dgamma;

arm = [0; 25.25; -7.9; 1] * .0254;
armInWCS = tf_WCS_ArmCS*arm;
arm_dheel = tf_WCS_ArmCS_dheel * arm;
arm_dgamma = tf_WCS_ArmCS_dgamma * arm;

rightingMoment = armInWCS(1)*load;
dheel = arm_dheel(1) * load;
dgamma = arm_dgamma(1) * load;

end

