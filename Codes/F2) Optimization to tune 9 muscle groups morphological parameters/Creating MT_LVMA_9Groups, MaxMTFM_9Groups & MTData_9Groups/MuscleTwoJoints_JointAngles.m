function [MuscleLength, MuscleVelocity, MuscleMomentArmProximal, MuscleMomentArmDistal] = ...
          MuscleTwoJoints_JointAngles(a1,a2,phi1,phi2,R1,R2,f1,f2,n1,n2,ls,alpha1,alpha2,alpha1_dot,alpha2_dot,handrule)

% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.

%-------common parameters --------------------------------------------
gamma1 = f1*acos(R1/a1);
gamma2 = -f2*acos(R2/a2);
beta1 = gamma1 + phi1;
beta2 = gamma2 + phi2;
h1 = sqrt(ls^2-(f1*R1-f2*R2)^2);
h2 = sqrt(R2^2+h1^2);
h3 = sqrt(R1^2+h1^2);
rho1 = -f1*acos((h2^2-R1^2-ls^2)/(-2*R1*ls));
rho2 = f2*acos((h3^2-R2^2-ls^2)/(-2*R2*ls));
theta1 = abs(f1-n1)*pi + f1*n1*(alpha1-n1*(beta1-rho1));
theta2 = abs(f2-n2)*pi + f2*n2*(alpha2+n2*(beta2-rho2));


hr = handrule; % it is 1 for right hand rule, and -1 for left hand rule


%% Muscle length and moment computation

if (theta1>=0 && theta2>=0)
lmt1 = sqrt(a1^2-R1^2);
lmt2 = R1*theta1;
lmt3 = h1;
lmt4 = R2*theta2;
lmt5 = sqrt(a2^2-R2^2);
Lmt = lmt1 + lmt2 + lmt3 + lmt4 + lmt5;
Lmt_dot = R1*f1*n1*alpha1_dot + R2*f2*n2*alpha2_dot;


MR1 = f1*abs(R1*f1*n1);
MR2 = f2*abs(R2*f2*n2);
MR1 = MR1*hr;
MR2 = MR2*hr;

elseif (theta1<0 && theta2>=0)
h4 = sqrt(ls^2 + a1^2 - 2*a1*ls*cos(alpha1-n1*phi1));
lmt1 = sqrt(h4^2-R2^2);
rho2_m1 = f1*acos((a1^2-ls^2-h4^2)/(-2*ls*h4));
rho2_m2 = f2*acos(R2/h4);
rho2_m = f2*abs(rho2_m1+rho2_m2);
theta2_m = abs(f2-n2)*pi + f2*n2*(alpha2+n2*(beta2-rho2_m));
lmt2 = R2*theta2_m;
lmt3 = sqrt(a2^2-R2^2);
Lmt = lmt1 + lmt2 + lmt3;
Lmt_dot = R2*f2*n2*alpha2_dot + (a1*ls*sin(alpha1 - n1*phi1)*alpha1_dot)/(- R2^2 + a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(1/2) + ...
          R2*f2^2*n2^2*sign(f2*acos(R2/(a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(1/2)) + f1*acos((ls - a1*cos(alpha1 - n1*phi1))/...
          (a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(1/2)))*((f1*((a1*sin(alpha1 - n1*phi1)*alpha1_dot)/...
          (a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(1/2) - (a1*ls*sin(alpha1 - n1*phi1)*(ls - a1*cos(alpha1 - n1*phi1))*alpha1_dot)/...
          (a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(3/2)))/(1 - (ls - a1*cos(alpha1 - n1*phi1))^2/(a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2))^(1/2) - ...
          (R2*a1*f2*ls*sin(alpha1 - n1*phi1)*alpha1_dot)/((- R2^2/(a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2) + 1)^(1/2)*(a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(3/2)));


MR1 = f1*abs((a1*ls*sin(alpha1 - n1*phi1))/(- R2^2 + a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(1/2) + ...
      R2*f2^2*n2^2*sign(f1*acos((ls - a1*cos(alpha1 - n1*phi1))/(a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(1/2)) + ...
      f2*acos(R2/(a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(1/2)))*((2^(1/2)*a1^2*f1*(2*a1*sin(alpha1 - n1*phi1) - ...
      ls*sin(2*alpha1 - 2*n1*phi1)))/(2*(-(a1^2*(cos(2*alpha1 - 2*n1*phi1) - 1))/(a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2))^(1/2)* ...
      (a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(3/2)) - (R2*a1*f2*ls*sin(alpha1 - n1*phi1))/ ...
      ((- R2^2/(a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2) + 1)^(1/2)*(a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(3/2))));
MR2 = f2*abs(R2*f2*n2);
MR1 = MR1*hr;
MR2 = MR2*hr;

elseif (theta1>=0 && theta2<0)
lmt1 = sqrt(a1^2-R1^2);
h5 = sqrt(ls^2 + a2^2 - 2*a2*ls*cos(alpha2+n2*phi2));
rho1_m1 = f1*acos((a2^2-ls^2-h5^2)/(-2*ls*h5));
rho1_m2 = f2*acos(R1/h5);
rho1_m = -f1*abs(rho1_m1+rho1_m2);
theta1_m = abs(f1-n1)*pi + f1*n1*(alpha1-n1*(beta1-rho1_m));
lmt2 = R1*theta1_m;
lmt3 = sqrt(h5^2-R1^2);
Lmt = lmt1 + lmt2 + lmt3;
Lmt_dot = R1*f1*n1*alpha1_dot + (a2*ls*sin(alpha2 + n2*phi2)*alpha2_dot)/(- R1^2 + a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(1/2) + ...
          R1*f1^2*n1^2*sign(f2*acos(R1/(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(1/2)) + f1*acos((ls - a2*cos(alpha2 + n2*phi2))/...
          (a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(1/2)))*((f1*((a2*sin(alpha2 + n2*phi2)*alpha2_dot)/(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(1/2) - ...
          (a2*ls*sin(alpha2 + n2*phi2)*(ls - a2*cos(alpha2 + n2*phi2))*alpha2_dot)/(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(3/2)))/...
          (1 - (ls - a2*cos(alpha2 + n2*phi2))^2/(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2))^(1/2) - (R1*a2*f2*ls*sin(alpha2 + n2*phi2)*alpha2_dot)/...
          ((- R1^2/(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2) + 1)^(1/2)*(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(3/2)));


MR1 = f1*abs(R1*f1*n1);
MR2 = f2*abs((a2*ls*sin(alpha2 + n2*phi2))/(- R1^2 + a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(1/2) + ...
      R1*f1^2*n1^2*sign(f1*acos((ls - a2*cos(alpha2 + n2*phi2))/(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(1/2)) + ...
      f2*acos(R1/(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(1/2)))*((2^(1/2)*a2^2*f1*(2*a2*sin(alpha2 + n2*phi2) - ...
      ls*sin(2*alpha2 + 2*n2*phi2)))/(2*(-(a2^2*(cos(2*alpha2 + 2*n2*phi2) - 1))/(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2))^(1/2)* ...
      (a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(3/2)) - (R1*a2*f2*ls*sin(alpha2 + n2*phi2))/ ...
      ((- R1^2/(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2) + 1)^(1/2)*(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(3/2))));
MR1 = MR1*hr;
MR2 = MR2*hr;


else
h6 = a1*cos(pi-alpha1+n1*phi1);
h7 = a2*cos(pi-alpha2-n2*phi2);
h8 = ls + h6 + h7;
lmt1 = sqrt((f1*sqrt(a1^2-h6^2)-f2*sqrt(a2^2-h7^2))^2 + h8^2);
Lmt = lmt1;
Lmt_dot = -(2*(a1*sin(alpha1 - n1*phi1)*alpha1_dot + a2*sin(alpha2 + n2*phi2)*alpha2_dot)*(a1*cos(alpha1 - n1*phi1) - ls + a2*cos(alpha2 + n2*phi2)) ...
          - 2*(f1*(a1^2 - a1^2*cos(alpha1 - n1*phi1)^2)^(1/2) - f2*(a2^2 - a2^2*cos(alpha2 + n2*phi2)^2)^(1/2))*((a1^2*f1*cos(alpha1 - n1*phi1)*...
          sin(alpha1 - n1*phi1)*alpha1_dot)/(a1^2 - a1^2*cos(alpha1 - n1*phi1)^2)^(1/2) - (a2^2*f2*cos(alpha2 + n2*phi2)*sin(alpha2 + n2*phi2)*alpha2_dot)/...
          (a2^2 - a2^2*cos(alpha2 + n2*phi2)^2)^(1/2)))/(2*((a1*cos(alpha1 - n1*phi1) - ls + a2*cos(alpha2 + n2*phi2))^2 +...
          (f1*(a1^2 - a1^2*cos(alpha1 - n1*phi1)^2)^(1/2) - f2*(a2^2 - a2^2*cos(alpha2 + n2*phi2)^2)^(1/2))^2)^(1/2));



MR1 = (f1*abs(2*a1*sin(alpha1 - n1*phi1)*(a1*cos(alpha1 - n1*phi1) - ls + a2*cos(alpha2 + n2*phi2)) - (2*a1^2*f1*sin(2*alpha1 - 2*n1*phi1)* ...
      ((f1*(-a1^2*(cos(2*alpha1 - 2*n1*phi1) - 1))^(1/2))/2 - (f2*(-a2^2*(cos(2*alpha2 + 2*n2*phi2) - 1))^(1/2))/2))/ ...
      (-a1^2*(cos(2*alpha1 - 2*n1*phi1) - 1))^(1/2)))/(2*abs((f1*(-a1^2*(cos(alpha1 - n1*phi1)^2 - 1))^(1/2) - f2*(-a2^2*(cos(alpha2 + n2*phi2)^2 - 1))^(1/2))^2 + ...
      (a1*cos(alpha1 - n1*phi1) - ls + a2*cos(alpha2 + n2*phi2))^2)^(1/2));
MR2 = (f2*abs(2*a2*sin(alpha2 + n2*phi2)*(a1*cos(alpha1 - n1*phi1) - ls + a2*cos(alpha2 + n2*phi2)) + (2*a2^2*f2*cos(alpha2 + n2*phi2)*sin(alpha2 + n2*phi2)* ...
      (f1*(a1^2 - a1^2*cos(alpha1 - n1*phi1)^2)^(1/2) - f2*(a2^2 - a2^2*cos(alpha2 + n2*phi2)^2)^(1/2)))/(a2^2 - a2^2*cos(alpha2 + n2*phi2)^2)^(1/2)))/ ...
      (2*abs((f1*(a1^2 - a1^2*cos(alpha1 - n1*phi1)^2)^(1/2) - f2*(a2^2 - a2^2*cos(alpha2 + n2*phi2)^2)^(1/2))^2 + (a1*cos(alpha1 - n1*phi1) - ls + a2*cos(alpha2 + n2*phi2))^2)^(1/2));
MR1 = MR1*hr;
MR2 = MR2*hr;

end
    
    
MuscleLength = Lmt;
MuscleVelocity = Lmt_dot;
MuscleMomentArmProximal = MR1;
MuscleMomentArmDistal = MR2;

  
  
  
end

