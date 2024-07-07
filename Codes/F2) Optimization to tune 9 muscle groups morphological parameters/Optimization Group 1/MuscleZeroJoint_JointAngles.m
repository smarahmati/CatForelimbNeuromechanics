function [MuscleLength, MuscleVelocity, MuscleMomentArm] = MuscleZeroJoint_JointAngles(a1,a2,phi1,phi2)

% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.


%% Muscle length 

lmt1 = sqrt(a1^2 + a2^2 - 2*a1*a2*cos(phi1-phi2)); 
Lmt = lmt1;

MuscleLength = Lmt;
MuscleVelocity = 0;
MuscleMomentArm = 0;
  
  
  
end

