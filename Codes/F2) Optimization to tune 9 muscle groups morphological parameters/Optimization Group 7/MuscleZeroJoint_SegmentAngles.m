function [XO, YO, XI, YI, MuscleLength] = MuscleZeroJoint_SegmentAngles(a1,a2,phi1,phi2,y1,XC1,YC1)

% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.

XO = XC1 + a1*cos(pi-(y1-pi)-phi1);
YO = YC1 - a1*sin(pi-(y1-pi)-phi1);

XI = XC1 + a2*cos(pi-(y1-pi)-phi2);
YI = YC1 - a2*sin(pi-(y1-pi)-phi2);

    
% Lmt1
Lmt1 = sqrt((XO-XI)^2+(YO-YI)^2);
Lmt = Lmt1 ;

    
MuscleLength = Lmt;


end

