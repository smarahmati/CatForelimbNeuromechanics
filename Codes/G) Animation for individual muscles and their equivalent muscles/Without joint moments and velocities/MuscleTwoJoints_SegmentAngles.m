function [XO, YO, XI, YI, XP1, YP1, XP2, YP2, XP3, YP3, XP4, YP4, XP5, YP5, theta1, theta2, theta1_m, theta2_m, theta1_FK,theta2_FK, ...
          alpha1, alpha2, gamma1, gamma2, beta1, beta2, h1, h2, h3, rho1, rho2, rho1_m, rho2_m, MuscleLength] = ...
         MuscleTwoJoints_SegmentAngles(a1,a2,phi1,phi2,R1,R2,f1,f2,n1,n2,ls,y1,y2,y3,XC1,YC1,XC2,YC2)

% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.

alpha1 = pi-n1*((y1-pi)-(y2-pi));
alpha2 = pi-n2*((y2-pi)-(y3-pi));


gamma1 = f1*acos(R1/a1);
gamma2 = -f2*acos(R2/a2);

beta1 = gamma1 + phi1;
beta2 = gamma2 + phi2;
h1 = sqrt(ls^2-(f1*R1-f2*R2)^2);
h2 = sqrt(R2^2+h1^2);
h3 = sqrt(R1^2+h1^2);
rho1 = -f1*acos((h2^2-R1^2-ls^2)/(-2*R1*ls));
rho2 = f2*acos((h3^2-R2^2-ls^2)/(-2*R2*ls));
rho1_m = -f1*abs(f1*acos((2*ls^2 - 2*a2*cos(alpha2 + n2*phi2)*ls)/(2*ls*(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(1/2))) + f2*acos(R1/(a2^2 - 2*cos(alpha2 + n2*phi2)*a2*ls + ls^2)^(1/2)));
rho2_m = f2*abs(f1*acos((2*ls^2 - 2*a1*cos(alpha1 - n1*phi1)*ls)/(2*ls*(a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(1/2))) + f2*acos(R2/(a1^2 - 2*cos(alpha1 - n1*phi1)*a1*ls + ls^2)^(1/2)));


s = size((y1-pi),1);


XP1 = nan(s,1);
YP1 = nan(s,1);
XP2 = nan(s,1);
YP2 = nan(s,1);
XP3 = nan(s,1);
YP3 = nan(s,1);
XP4 = nan(s,1);
YP4 = nan(s,1);
XP5 = nan(s,1);
YP5 = nan(s,1);
theta1_FK = nan(s,1);
theta2_FK = nan(s,1);
theta2_m = nan(s,1);
theta1_m = nan(s,1);

theta1 = abs(f1-n1)*pi + f1*n1*(alpha1-n1*(beta1-rho1));
theta2 = abs(f2-n2)*pi + f2*n2*(alpha2+n2*(beta2-rho2));

XO = XC1 + a1*cos((y1-pi)+phi1);
YO = YC1 + a1*sin((y1-pi)+phi1);

XI = XC2 + a2*cos((y3-pi)+phi2-pi);
YI = YC2 + a2*sin((y3-pi)+phi2-pi);


if (theta1>=0 && theta2>=0) %--------------------------------------------------------------------
XP1 = XC1 + R1*cos((y1-pi)+beta1);
YP1 = YC1 + R1*sin((y1-pi)+beta1);

XP2 = XC1 + R1*cos((y2-pi)+rho1-pi);
YP2 = YC1 + R1*sin((y2-pi)+rho1-pi);
    
XP3 = XC2 + R2*cos((y2-pi)+rho2);
YP3 = YC2 + R2*sin((y2-pi)+rho2);

XP4 = XC2 + R2*cos((y3-pi)+beta2-pi);
YP4 = YC2 + R2*sin((y3-pi)+beta2-pi);

% Lmt1
Lmt1 = sqrt((XO-XP1)^2+(YO-YP1)^2);

% Lmt2
  u = [XP1, YP1]-[XC1, YC1];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P1 = real(acos(CosTheta));
  
  u = [XP2, YP2]-[XC1, YC1];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P2 = real(acos(CosTheta));
  
  if  (YP1< YC1); theta_P1 = 2*pi-theta_P1; end
  if  (YP2< YC1); theta_P2 = 2*pi-theta_P2; end
  
  if (theta_P2>=theta_P1) && f1==1
    theta1_FK = theta_P2-theta_P1;
  end
  
  if (theta_P2<theta_P1) && f1==1
    theta1_FK =0;
    YP2 = YP1;
    XP2 = XP1;
  end
  
  if (sign(YP1- YC1) == sign(YP2- YC1)) && f1==-1 && (theta_P1>=theta_P2)
      theta1_FK = theta_P1 - theta_P2; 
  end
  
  if (sign(YP1- YC1) == sign(YP2- YC1)) && f1==-1 && (theta_P1<theta_P2)
     theta1_FK =0;
     YP2 = YP1;
     XP2 = XP1;
  end

  if (sign(YP1- YC1) ~= sign(YP2- YC1)) && f1==-1
      theta_P2 = theta_P2 -2*pi;
      theta1_FK = theta_P1 - theta_P2; 
  end
 
  
   Lmt2 = R1*theta1_FK;

  
% Lmt3
Lmt3 = sqrt((XP2-XP3)^2+(YP2-YP3)^2);

% Lmt4
  u = [XP3, YP3]-[XC2, YC2];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P3 = real(acos(CosTheta));
  
  u = [XP4, YP4]-[XC2, YC2];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P4 = real(acos(CosTheta));
  
  if  (YP3< YC2); theta_P3 = 2*pi-theta_P3; end
  if  (YP4< YC2); theta_P4 = 2*pi-theta_P4; end

  if (theta_P4>=theta_P3) && f2==1
    theta2_FK = theta_P4-theta_P3;
  end
  
  if (theta_P4<theta_P3) && f2==1
    theta2_FK =0;
    YP3 = YP4;
    XP3 = XP4;
  end
   
  if (sign(YP3- YC2) == sign(YP4- YC2)) && f2==-1 && (theta_P3>=theta_P4)
      theta2_FK = theta_P3 - theta_P4; 
  end
  
  if (sign(YP3- YC2) == sign(YP4- YC2)) && f2==-1 && (theta_P3<theta_P4)
     theta2_FK =0;
     YP3 = YP4;
     XP3 = XP4;
  end

  if (sign(YP3- YC2) ~= sign(YP4- YC2)) && f2==-1
      theta_P4 = theta_P4 -2*pi;
      theta2_FK = theta_P3 - theta_P4; 
  end
  
  Lmt4 = R2*theta2_FK;
  
% Lmt5
Lmt5 = sqrt((XP4-XI)^2+(YP4-YI)^2);

Lmt = Lmt1 + Lmt2 + Lmt3 + Lmt4 + Lmt5;
    

elseif (theta1<0 && theta2>=0) %---------------------------------------------------------------
XP4 = XC2 + R2*cos((y2-pi)+rho2_m);
YP4 = YC2 + R2*sin((y2-pi)+rho2_m);

XP5 = XC2 + R2*cos((y3-pi)+beta2-pi);
YP5 = YC2 + R2*sin((y3-pi)+beta2-pi);


theta2_m = abs(f2-n2)*pi + f2*n2*(alpha2+n2*(beta2-rho2_m));

% Lmt1
Lmt1 = sqrt((XO-XP4)^2+(YO-YP4)^2);

% Lmt2
  u = [XP4, YP4]-[XC2, YC2];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P4 = real(acos(CosTheta));
  
  u = [XP5, YP5]-[XC2, YC2];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P5 = real(acos(CosTheta));
  
  if  (YP4< YC2); theta_P4 = 2*pi-theta_P4; end
  if  (YP5< YC2); theta_P5 = 2*pi-theta_P5; end
  
  if (theta_P5>=theta_P4) && f2==1
    theta2_FK = theta_P5-theta_P4;
  end
  
  if (theta_P5<theta_P4) && f2==1
    theta2_FK =0;
    YP4 = YP5;
    XP4 = XP5;
    Lmt1 = sqrt((XO-XP5)^2+(YO-YP5)^2);
  end
  
  
  if (sign(YP4- YC2) == sign(YP5- YC2)) && f2==-1 && (theta_P4>=theta_P5)
      theta2_FK = theta_P4 - theta_P5; 
  end
  
  if (sign(YP4- YC2) == sign(YP5- YC2)) && f2==-1 && (theta_P4<theta_P5)
     theta2_FK =0;
     YP4 = YP5;
     XP4 = XP5;
     Lmt1 = sqrt((XO-XP5)^2+(YO-YP5)^2);
  end
  
  if (sign(YP4- YC2) ~= sign(YP5- YC2)) && f2==-1
      theta_P5 = theta_P5 -2*pi;
      theta2_FK = theta_P4 - theta_P5; 
  end

   Lmt2 = R2*theta2_FK;


% Lmt3
Lmt3 = sqrt((XP5-XI)^2+(YP5-YI)^2);

Lmt = Lmt1 + Lmt2 + Lmt3;


elseif (theta1>=0 && theta2<0) %---------------------------------------------------------------

XP1 = XC1 + R1*cos((y1-pi)+beta1);
YP1 = YC1 + R1*sin((y1-pi)+beta1);

XP3 = XC1 + R1*cos((y2-pi)+rho1_m-pi);
YP3 = YC1 + R1*sin((y2-pi)+rho1_m-pi);

theta1_m = abs(f1-n1)*pi + f1*n1*(alpha1-n1*(beta1-rho1_m));

% Lmt1
Lmt1 = sqrt((XO-XP1)^2+(YO-YP1)^2);

% Lmt2
  u = [XP1, YP1]-[XC1, YC1];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P1 = real(acos(CosTheta));
  
  u = [XP3, YP3]-[XC1, YC1];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P3 = real(acos(CosTheta));
  
  if  (YP1< YC1); theta_P1 = 2*pi-theta_P1; end
  if  (YP3< YC1); theta_P3 = 2*pi-theta_P3; end
  
  if (theta_P3>=theta_P1) && f1==1
    theta1_FK = theta_P3-theta_P1;
  end
  
  if (theta_P3<theta_P1) && f1==1
    theta1_FK =0;
    YP3 = YP1;
    XP3 = XP1;
  end  
  
  if (sign(YP1- YC1) == sign(YP3- YC1)) && f1==-1 && (theta_P1>=theta_P3)
      theta1_FK = theta_P1 - theta_P3; 
  end
  
  if (sign(YP1- YC1) == sign(YP3- YC1)) && f1==-1 && (theta_P1<theta_P3)
     theta2_FK =0;
     YP3 = YP1;
     XP3 = XP1;
  end
  
  if (sign(YP1- YC1) ~= sign(YP3- YC1)) && f1==-1
      theta_P3 = theta_P3 -2*pi;
      theta1_FK = theta_P1 - theta_P3; 
  end

   Lmt2 = R1*theta1_FK;
   
   

% Lmt3
Lmt3 = sqrt((XP3-XI)^2+(YP3-YI)^2);

if isnan(Lmt1)==1
 Lmt1 = 0;
end

if isnan(Lmt2)==1
 Lmt2 = 0;
end  

if isnan(Lmt3)==1
 Lmt3 = 0;
end  

Lmt = Lmt1 + Lmt2 + Lmt3;



else %------------------------------------------------------------------------------------------
Lmt = sqrt((XO-XI)^2+(YO-YI)^2);

end
    
    
MuscleLength = Lmt;

  
  
  
end

