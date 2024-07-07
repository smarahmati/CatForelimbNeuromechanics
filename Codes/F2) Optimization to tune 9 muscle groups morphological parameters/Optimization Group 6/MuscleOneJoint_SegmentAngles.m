function [XO, YO, XI, YI, XP1, YP1, XP2, YP2, theta1, theta1_FK, ...
          alpha1, gamma1, gamma2, beta1, beta2, MuscleLength] = ...
         MuscleOneJoint_SegmentAngles(a1,a2,phi1,phi2,R1,f1,n1,y1,y2,XC1,YC1)

% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.

alpha1 = pi-n1*((y1-pi)-(y2-pi));

%-------common parameters --------------------------------------------
gamma1 = f1*acos(R1/a1);
gamma2 = -f1*acos(R1/a2);
beta1 = gamma1 + phi1;
beta2 = gamma2 + phi2;
theta1 = abs(f1-n1)*pi + f1*n1*(alpha1-n1*(beta1-beta2));


s = size((y1-pi),1);

XP1 = nan(s,1);
YP1 = nan(s,1);
XP2 = nan(s,1);
YP2 = nan(s,1);
theta1_FK = nan(s,1);


XO = XC1 + a1*cos((y1-pi)+phi1);
YO = YC1 + a1*sin((y1-pi)+phi1);

XI = XC1 + a2*cos((y2-pi)+phi2-pi);
YI = YC1 + a2*sin((y2-pi)+phi2-pi);



if (theta1>=0) %-------------------------------------------------------------------------------
XP1 = XC1 + R1*cos((y1-pi)+beta1);
YP1 = YC1 + R1*sin((y1-pi)+beta1);

XP2 = XC1 + R1*cos((y2-pi)+beta2-pi);
YP2 = YC1 + R1*sin((y2-pi)+beta2-pi);
    
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
  
  if (theta_P2<theta_P1) && f1==1 && gamma1==0
    theta1_FK =0;
    YP2 = YP1;
    XP2 = XP1;
  end  
  
  if (theta_P2<theta_P1) && f1==1 && gamma2==0
    theta1_FK =0;
    YP1 = YP2;
    XP1 = XP2;
  end  
  
  if (sign(YP1- YC1) == sign(YP2- YC1)) && f1==-1 && (theta_P1>=theta_P2)
      theta1_FK = theta_P1 - theta_P2; 
  end
  
  if (sign(YP1- YC1) == sign(YP2- YC1)) && f1==-1 && (theta_P1<theta_P2) && gamma1==0
     theta2_FK =0;
     YP2 = YP1;
     XP2 = XP1;
  end
  
  if (sign(YP1- YC1) == sign(YP2- YC1)) && f1==-1 && (theta_P1<theta_P2) && gamma2==0
     theta2_FK =0;
     YP1 = YP2;
     XP1 = XP2;
  end
  
  if (sign(YP1- YC1) ~= sign(YP2- YC1)) && f1==-1
      theta_P2 = theta_P2 -2*pi;
      theta1_FK = theta_P1 - theta_P2; 
  end
  
  

Lmt2 = R1*theta1_FK;

% Lmt3 
Lmt3 = sqrt((XP2-XI)^2+(YP2-YI)^2);

Lmt = Lmt1 + Lmt2 + Lmt3;
else  %-------------------------------------------------------------------------------
    
% Lmt1
Lmt1 = sqrt((XO-XI)^2+(YO-YI)^2);
Lmt = Lmt1 ;
end
    
    
MuscleLength = Lmt;

  
  
  
end

