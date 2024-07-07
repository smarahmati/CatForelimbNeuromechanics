function [MuscleLength, MuscleVelocity, MuscleMomentArm] = MuscleOneJoint_JointAngles(a1,a2,phi1,phi2,R1,f1,n1,alpha1,alpha1_dot,handrule)

% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.


%-------common parameters --------------------------------------------
gamma1 = f1*acos(R1/a1);
gamma2 = -f1*acos(R1/a2);
beta1 = gamma1 + phi1;
beta2 = gamma2 + phi2;
theta1 = abs(f1-n1)*pi + f1*n1*(alpha1-n1*(beta1-beta2));


hr = handrule; % it is 1 for right hand rule, and -1 for left hand rule

%% Muscle length and moment computation

if (theta1>=0)
lmt1 = sqrt(a1^2-R1^2);
lmt2 = R1*theta1;
lmt3 = sqrt(a2^2-R1^2);
Lmt = lmt1 + lmt2 + lmt3;
Lmt_dot = R1*f1*n1*alpha1_dot;

  MR1 = f1*abs(R1*f1*n1);
  MR1 = hr*MR1;


else
lmt1 = sqrt(a1^2 + a2^2 - 2*a1*a2*cos(alpha1-n1*(phi1-phi2))); 
Lmt = lmt1;
Lmt_dot = (a1*a2*sin(alpha1 - n1*(phi1 - phi2))*alpha1_dot)/(a1^2 - 2*cos(alpha1 - n1*(phi1 - phi2))*a1*a2 + a2^2)^(1/2);

  MR1 = (f1*abs(a1*a2*sin(alpha1 - n1*(phi1 - phi2))))/abs(a1^2 - 2*cos(alpha1 - n1*(phi1 - phi2))*a1*a2 + a2^2)^(1/2);
  MR1 = hr*MR1;

end
    
    
MuscleLength = Lmt;
MuscleVelocity = Lmt_dot;
MuscleMomentArm = MR1;


  
  
  
end

