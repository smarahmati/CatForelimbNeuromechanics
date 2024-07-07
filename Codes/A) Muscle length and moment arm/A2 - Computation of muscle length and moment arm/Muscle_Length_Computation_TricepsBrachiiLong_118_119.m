clear
clc

% f = gcf;
% exportgraphics(f,'Fig1_Original.png','Resolution',600)
MotionData = load('MotionData.mat');
MusculoskeletalData = load('MusculoskeletalData.mat');

%% Motion Data
MotionData = MotionData.MotionData; 

% Joint Angles
SHLa = MotionData.GenericCat.Angles.AngShou.Total*(pi/180);
ELBa = MotionData.GenericCat.Angles.AngElbow.Total*(pi/180);
WRTa = MotionData.GenericCat.Angles.AngWrist.Total*(pi/180);
MCPa = MotionData.GenericCat.Angles.AngMc.Total*(pi/180);

% Joint Angular Velocities
SHLa_dot = MotionData.GenericCat.AngularVelocities.VAShou.Total*(pi/180);
ELBa_dot = MotionData.GenericCat.AngularVelocities.VAElbow.Total*(pi/180);
WRTa_dot = MotionData.GenericCat.AngularVelocities.VAWrist.Total*(pi/180);
MCPa_dot = MotionData.GenericCat.AngularVelocities.VAMc.Total*(pi/180);

% Segment Angles
q1base = MotionData.GenericCat.Angles.AngScap.Total*(pi/180);
q1 = q1base + pi;
q2 = SHLa + q1 - pi;
q3 = -ELBa + q2 + pi;
q4 = -WRTa + q3 + pi;
q5 = -MCPa + q4 + pi;


% Segment Length (Mo is a brief for Motion) 
SCP_Mo2D = MotionData.GenericCat.MLW.ScapulaLength;
Luarm_Mo2D = MotionData.GenericCat.MLW.UpperArmLength;
Lfarm_Mo2D = MotionData.GenericCat.MLW.ForeArmLength;
Lcar_Mo2D = MotionData.GenericCat.MLW.CarpalsLength;
Lfdig_Mo2D = MotionData.GenericCat.MLW.ForeDigitsLength;



%---------- Forward Kinematics in 2D --------------------------------------------------------------------------
%----------Joint center of forelimbs based on measured length--------------------------------------------------
SHL(:,1) = MotionData.GenericCat.Coordinates.xSho.Total; % reference
SHL(:,2) = MotionData.GenericCat.Coordinates.ySho.Total; % reference

SCP(:,1) = SHL(:,1) + SCP_Mo2D*cos(q1-pi);
SCP(:,2) = SHL(:,2) + SCP_Mo2D*sin(q1-pi);

ELB(:,1) = SHL(:,1)-Luarm_Mo2D*cos(q2-pi);
ELB(:,2) = SHL(:,2)-Luarm_Mo2D*sin(q2-pi);

WRT(:,1) = ELB(:,1)-Lfarm_Mo2D*cos(q3-pi);
WRT(:,2) = ELB(:,2)-Lfarm_Mo2D*sin(q3-pi);

MCP(:,1) = WRT(:,1)-Lcar_Mo2D*cos(q4-pi);
MCP(:,2) = WRT(:,2)-Lcar_Mo2D*sin(q4-pi);

FT(:,1) = MCP(:,1)-Lfdig_Mo2D*cos(q5-pi);
FT(:,2) = MCP(:,2)-Lfdig_Mo2D*sin(q5-pi);



% Time
CT = MotionData.GenericCat.CycleTime.Average; % Cycle Time
dt = CT/(size(q1,1)-1);
time = (0:size(q1,1)-1)'*dt;


%% Musculoskeletal Data
MuscleGeometricParameters = MusculoskeletalData.MusculoskeletalData.Two_Dimension.MuscleGeometricParameters; 
a1_ALL = MuscleGeometricParameters(:,1);
a2_ALL = MuscleGeometricParameters(:,2);
phi1_ALL = MuscleGeometricParameters(:,3);
phi2_ALL = MuscleGeometricParameters(:,4);
R1_ALL = MuscleGeometricParameters(:,5);
R2_ALL = MuscleGeometricParameters(:,6);
f1_ALL = MuscleGeometricParameters(:,7);
f2_ALL = MuscleGeometricParameters(:,8);
n1_ALL = MuscleGeometricParameters(:,9);
n2_ALL = MuscleGeometricParameters(:,10);


% Segment Length (MS is a brief for Musculoskeletal)
SCP_MS2D = MusculoskeletalData.MusculoskeletalData.Two_Dimension.Segment_Length.Scapula_Length;
Luarm_MS2D = MusculoskeletalData.MusculoskeletalData.Two_Dimension.Segment_Length.Upperarm_Length;
Lfarm_MS2D = MusculoskeletalData.MusculoskeletalData.Two_Dimension.Segment_Length.Forearm_Length;
Lcar_MS2D = MusculoskeletalData.MusculoskeletalData.Two_Dimension.Segment_Length.Carpals_Length;

SCP_MS3D = MusculoskeletalData.MusculoskeletalData.Three_Dimension.Segment_Length.Scapula_Length;
Luarm_MS3D = MusculoskeletalData.MusculoskeletalData.Three_Dimension.Segment_Length.Upperarm_Length;
Lfarm_MS3D = MusculoskeletalData.MusculoskeletalData.Three_Dimension.Segment_Length.Forearm_Length;
Lcar_MS3D = MusculoskeletalData.MusculoskeletalData.Three_Dimension.Segment_Length.Carpals_Length;



%% X & Y min and max values
MinX = min(min([SCP(:,1), SHL(:,1), ELB(:,1), WRT(:,1), MCP(:,1), FT(:,1)]));
MaxX = max(max([SCP(:,1), SHL(:,1), ELB(:,1), WRT(:,1), MCP(:,1), FT(:,1)]));

MinY = min(min([SCP(:,2), SHL(:,2), ELB(:,2), WRT(:,2), MCP(:,2), FT(:,2)]));
MaxY = max(max([SCP(:,2), SHL(:,2), ELB(:,2), WRT(:,2), MCP(:,2), FT(:,2)]));



%% Scaling of musculoskeletal model to motion model
Scale_Scap2D = SCP_Mo2D/SCP_MS2D;
Scale_Uarm2D = Luarm_Mo2D/Luarm_MS2D;
Scale_farm2D = Lfarm_Mo2D/Lfarm_MS2D;
Scale_car2D = Lcar_Mo2D/Lcar_MS2D;


%% Computation of muscle length by two approaches

% Triceps Brachii Long(118>119)
MN = 75;
ls = Luarm_Mo2D;
alpha1 = SHLa;
alpha2 = ELBa;
alpha1_dot = SHLa_dot;
alpha2_dot = ELBa_dot;
y1 = q1;
y2 = q2;
y3 = q3;
XC1 = SHL(:,1);
YC1 = SHL(:,2);
XC2 = ELB(:,1);
YC2 = ELB(:,2);
Sc_a1_r1 = Scale_Scap2D;
Sc_a2_r2 = Scale_farm2D;
handrule = -1; % it is 1 for right hand rule, and -1 for left hand rule

a1 = a1_ALL(MN,1)*Sc_a1_r1;
a2 = a2_ALL(MN,1)*Sc_a2_r2;
phi1 = phi1_ALL(MN,1);
phi2 = phi2_ALL(MN,1);
R1 = R1_ALL(MN,1)*Sc_a1_r1;
R2 = R2_ALL(MN,1)*Sc_a2_r2;
f1 = f1_ALL(MN,1);
f2 = f2_ALL(MN,1);
n1 = n1_ALL(MN,1);
n2 = n2_ALL(MN,1);


% Modification factor to avoid muscle moment arm becomes less than joint radius
R2 = 0.80*R2;


if a1<R1 
msg = 'Error occurred: a1<R1';
error(msg)
end

if a2<R2
msg = 'Error occurred: a2<R2';
error(msg)
end



for i=1:size(alpha1,1)
[MuscleLength, MuscleVelocity, MuscleMomentArmProximal, MuscleMomentArmDistal] = MuscleTwoJoints_JointAngles(a1,a2,phi1,phi2,R1,R2,f1,f2,n1,n2,ls,alpha1(i,1),alpha2(i,1),alpha1_dot(i,1),alpha2_dot(i,1),handrule);
MuscleLength_time_JA(i,1) =  MuscleLength;
MuscleVelocity_time_JA(i,1) =  MuscleVelocity;
MuscleMomentArmProximal_time(i,1) = MuscleMomentArmProximal;
MuscleMomentArmDistal_time(i,1) = MuscleMomentArmDistal;
% lmt1_time(i,1) = lmt1;
% lmt2_time(i,1) = lmt2;
% lmt3_time(i,1) = lmt3;
% theta2_m_time(i,1) = theta2_m;
% rho2_m1_time(i,1) = rho2_m1;
% rho2_m2_time(i,1) = rho2_m2;
end



for i=1:size(alpha1,1)
 [XO, YO, XI, YI, XP1, YP1, XP2, YP2, XP3, YP3, XP4, YP4, XP5, YP5, theta1, theta2, theta1_m, theta2_m, theta1_FK,theta2_FK, ...
  alpha1, alpha2, gamma1, gamma2, beta1, beta2, h1, h2, h3, rho1, rho2, rho1_m, rho2_m, MuscleLength] = ...
  MuscleTwoJoints_SegmentAngles(a1,a2,phi1,phi2,R1,R2,f1,f2,n1,n2,ls,y1(i,1),y2(i,1),y3(i,1),XC1(i,1),YC1(i,1),XC2(i,1),YC2(i,1));

XO_time(i,1) = XO;
YO_time(i,1) = YO;

XI_time(i,1) = XI;
YI_time(i,1) = YI;

XP1_time(i,1) = XP1;
YP1_time(i,1) = YP1;

XP2_time(i,1) = XP2;
YP2_time(i,1) = YP2;

XP3_time(i,1) = XP3;
YP3_time(i,1) = YP3;

XP4_time(i,1) = XP4;
YP4_time(i,1) = YP4;

XP5_time(i,1) = XP5;
YP5_time(i,1) = YP5;

theta1_time(i,1) = theta1; % theta1 as the presented criterion
theta2_time(i,1) = theta2; % theta2 as the presented criterion

theta1_FK_time(i,1) = theta1_FK; % theta1 based on forwad kinematics
theta2_FK_time(i,1) = theta2_FK; % theta2 based on forwad kinematics

theta1_m_time(i,1) = theta1_m; % modified theta1 based on rho1_m
theta2_m_time(i,1) = theta2_m; % modified theta1 based on rho2_m

alpha1_time(i,1) = alpha1;
alpha2_time(i,1) = alpha2;
gamma1_time(i,1) = gamma1;
gamma2_time(i,1) = gamma2;
beta1_time(i,1) = beta1;
beta2_time(i,1) = beta2;
h1_time(i,1) = h1;
h2_time(i,1) = h2;
h3_time(i,1) = h3;
rho1_time(i,1) = rho1;
rho2_time(i,1) = rho2;
rho1_m_time(i,1) = rho1_m;
rho2_m_time(i,1) = rho2_m;

MuscleLength_time_SA(i,1) =  MuscleLength; % Worse -> we consider the following computation for muscle length based on forward kinematics 
end




%% Write information
cmn = 38;                                                                  % considered muscle number
mn = 'Triceps Brachii Long';                                               % muscle name
ml = MuscleLength_time_JA;                                                 % muscle length
mv = MuscleVelocity_time_JA;                                               % muscle Velocity
ma = [MuscleMomentArmProximal_time, MuscleMomentArmDistal_time, time*0];   % muscle momemet arm
me = [1, 1, 0];                                                            % muscle effective joints


MuscleLengthVelocityMomentArm = load('MuscleLengthVelocityMomentArm.mat');
MuscleLengthVelocityMomentArm = MuscleLengthVelocityMomentArm.MuscleLengthVelocityMomentArm;

%-----Muscle name, length and moment mrm ---------------------------------
MuscleLengthVelocityMomentArm{cmn,1}.Name = mn;
MuscleLengthVelocityMomentArm{cmn,1}.Length = ml;
MuscleLengthVelocityMomentArm{cmn,1}.Velocity = mv;
MuscleLengthVelocityMomentArm{cmn,1}.MomentArm = ma;
MuscleLengthVelocityMomentArm{cmn,1}.EffectiveJoints = me;

save('MuscleLengthVelocityMomentArm.mat', 'MuscleLengthVelocityMomentArm');




%% Write current muscle geometric parameters

ScaledMuscleMorphologicalParameters = load('ScaledMuscleMorphologicalParameters.mat');
ScaledMuscleMorphologicalParameters = ScaledMuscleMorphologicalParameters.ScaledMuscleMorphologicalParameters;

ScaledMuscleMorphologicalParameters{MN, 1}.Name = mn;
T = table(a1, a2, phi1, phi2, R1, R2, f1, f2, n1, n2);
ScaledMuscleMorphologicalParameters{MN, 1}.Parameters = T;  

save('ScaledMuscleMorphologicalParameters.mat', 'ScaledMuscleMorphologicalParameters');



%% Plot

figure(1)
plot(time, MuscleLength_time_JA,'-k','LineWidth',1)
hold on
plot(time, MuscleLength_time_SA,'Ob','LineWidth',1, 'MarkerSize', 5)
hold on
grid on
xlabel('Time(s)');
ylabel('Muscle Length (m)');
legend('Joint Angles', 'Forward Kinematics')
title('Comparision of muscle length based on joint angles and forward kinematics')
% set(gca,'XTick',[])
savefig('MuscleLength_TricepsBrachiiLong.fig')


figure(2)
plot(time, MuscleVelocity_time_JA,'-k','LineWidth',1)
grid on
xlabel('Time(s)');
ylabel('Muscle Velocity (m/s)');
legend('MV based on symbolic derivation of muscle length')
title('Muscle velocity based on joint angle')
% set(gca,'XTick',[])
savefig('MuscleVelocity_TricepsBrachiiLong.fig')



figure(3)
plot(time, MuscleMomentArmProximal_time,'-k','LineWidth',1)
hold on
plot(time, MuscleMomentArmDistal_time,'-b','LineWidth',1)
hold on
plot(time, -R1+time*0,'--k','LineWidth',1)
hold on
plot(time, -R2+time*0,'--b','LineWidth',1)
grid on
xlabel('Time(s)');
ylabel('Muscle Moment Arm (m)');
legend('Proximal Joint', 'Distal Joint','Proximal Joint Radius','Distal Joint Radius')
title('Muscle Moment Arm based on joint angles')
% set(gca,'XTick',[])
savefig('MuscleLength_TricepsBrachiiLong.fig')



%{
figure(3)
plot(time, MuscleMomentArmProximal_time,'-k','LineWidth',1)
hold on
grid on
xlabel('Time(s)');
ylabel('Proximal Muscle Moment Arm (m)');
% legend('Proximal Joint')
title('Muscle Moment Arm based on joint angles')
% set(gca,'XTick',[])

figure(4)
plot(time, MuscleMomentArmDistal_time,'-b','LineWidth',1)
hold on
grid on
xlabel('Time(s)');
ylabel('Distal Muscle Moment Arm (m)');
% legend('Distal Joint')
title('Muscle Moment Arm based on joint angles')
% set(gca,'XTick',[])
%}



%% Simulation

f=0;


h = figure(4);
% pause(2);
for i= 1
% 1:1:size(SCP,1)
       
      
ms = 20; % Marker Size      
% Joint Centers
s1 = scatter(SCP(i,1),SCP(i,2),ms,'MarkerFaceColor',[0.5, 0.5, 0],'MarkerEdgeColor',[0.5, 0.5, 0]);
hold on
alpha(s1,0.7)

s2 = scatter(SHL(i,1),SHL(i,2),ms,'MarkerFaceColor',[1, 0, 0],'MarkerEdgeColor',[1, 0, 0]);
hold on
alpha(s2,0.7)

s3 = scatter(ELB(i,1),ELB(i,2),ms,'MarkerFaceColor',[0, 1, 0],'MarkerEdgeColor',[0, 1, 0]);
hold on
alpha(s3,0.7)

s4 = scatter(WRT(i,1),WRT(i,2),ms,'MarkerFaceColor',[0, 0, 1],'MarkerEdgeColor',[0, 0, 1]);
hold on
alpha(s4,0.7)

s5 = scatter(MCP(i,1),MCP(i,2),ms,'MarkerFaceColor',[0.5, 0, 0.5],'MarkerEdgeColor',[0.5, 0, 0.5]);
hold on
alpha(s5,0.7)

s6 = scatter(FT(i,1),FT(i,2),ms,'MarkerFaceColor',[0, 0.5, 0.5],'MarkerEdgeColor',[0, 0.5, 0.5]);
hold on
alpha(s6,0.7)


% Attachment Points
ATO = scatter(XO_time(i,1), YO_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP1 = scatter(XP1_time(i,1), YP1_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP2 = scatter(XP2_time(i,1), YP2_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP3 = scatter(XP3_time(i,1), YP3_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP4 = scatter(XP4_time(i,1), YP4_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP5 = scatter(XP5_time(i,1), YP5_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATI = scatter(XI_time(i,1), YI_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
 
 %% Line Model
 
 
 %------------------  left fore limbs -----------------------------------
 line([SCP(i,1) SHL(i,1)],[SCP(i,2) SHL(i,2)],'color',[0 0 0],'LineWidth',1.5)
 hold on
 line([SHL(i,1) ELB(i,1)],[SHL(i,2) ELB(i,2)],'color',[0 0 0],'LineWidth',1.5)
 hold on
 line([ELB(i,1) WRT(i,1)],[ELB(i,2) WRT(i,2)],'color',[0 0 0],'LineWidth',1.5)
 hold on
 line([WRT(i,1) MCP(i,1)],[WRT(i,2) MCP(i,2)],'color',[0 0 0],'LineWidth',1.5)
 hold on
 line([MCP(i,1) FT(i,1)],[MCP(i,2) FT(i,2)],'color',[0 0 0],'LineWidth',1.5)
 hold on

% FA = 1000; 
% quiver3(P1_S(i,1),P1_S(i,2),P1_S(i,3),FA*Trunk_normal(i,1),FA*Trunk_normal(i,2),FA*Trunk_normal(i,3),'LineWidth',2)
% hold on
% quiver3(P1_S(i,1),P1_S(i,2),P1_S(i,3),FA*Pelvis_normal(i,1),FA*Pelvis_normal(i,2),FA*Pelvis_normal(i,3),'LineWidth',2)


%% Joint surface

th = 0:pi/1000:2*pi;
x = XC1(i,1);
y = YC1(i,1);
r = R1;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
Cj1 = plot(xunit, yunit,'color','cyan');

x = XC2(i,1);
y = YC2(i,1);
r = R2;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
Cj2 = plot(xunit, yunit,'color','cyan');


%% Attachment points

if (theta1_time(i,1)>=0 && theta2_time(i,1)>=0) %-----------------------------------------------
% Lmt1    
  line([XO_time(i,1) XP1_time(i,1)],[YO_time(i,1) YP1_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on
 
% Lmt2   
  u = [XP1_time(i,1), YP1_time(i,1)]-[XC1(i,1), YC1(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P1(i,1) = real(acos(CosTheta));
  
  u = [XP2_time(i,1), YP2_time(i,1)]-[XC1(i,1), YC1(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P2(i,1) = real(acos(CosTheta));
  
  if  YP1_time(i,1)< YC1(i,1); theta_P1(i,1) = 2*pi-theta_P1(i,1); end
  if  YP2_time(i,1)< YC1(i,1); theta_P2(i,1) = 2*pi-theta_P2(i,1); end
  
  if (theta_P2(i,1)>=theta_P1(i,1)) && f1==1
    th = theta_P1(i,1):pi/1000:theta_P2(i,1);
  end
  
  if (theta_P2(i,1)<theta_P1(i,1)) && f1==1
    th = 0;
    YP2_time(i,1) = YP1_time(i,1);
    XP2_time(i,1) = XP1_time(i,1);
  end
  
  if (sign(YP1_time(i,1)- YC1(i,1)) == sign(YP2_time(i,1)- YC1(i,1))) && f1==-1 && (theta_P1(i,1)>=theta_P2(i,1))
      th = theta_P2(i,1):pi/1000:theta_P1(i,1); 
  end
  
  if (sign(YP1_time(i,1)- YC1(i,1)) == sign(YP2_time(i,1)- YC1(i,1))) && f1==-1 && (theta_P1(i,1)<theta_P2(i,1))
     th = 0;
     YP2_time(i,1) = YP1_time(i,1);
     XP2_time(i,1) = XP1_time(i,1);
  end

  if (sign(YP1_time(i,1)- YC1(i,1)) ~= sign(YP2_time(i,1)- YC1(i,1))) && f1==-1
      theta_P2(i,1) = theta_P2(i,1) -2*pi;
      th = theta_P2(i,1):pi/1000:theta_P1(i,1); 
  end

  
  x = XC1(i,1);
  y = YC1(i,1);
  r = R1;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj3 = plot(xunit, yunit,'color',[1 0 0]);
  hold on
  
% Lmt3   
  line([XP2_time(i,1) XP3_time(i,1)],[YP2_time(i,1) YP3_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on
  
% Lmt4     
  u = [XP3_time(i,1), YP3_time(i,1)]-[XC2(i,1), YC2(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P3(i,1) = real(acos(CosTheta));
  
  u = [XP4_time(i,1), YP4_time(i,1)]-[XC2(i,1), YC2(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P4(i,1) = real(acos(CosTheta));
  
  if  YP3_time(i,1)< YC2(i,1); theta_P3(i,1) = 2*pi-theta_P3(i,1); end
  if  YP4_time(i,1)< YC2(i,1); theta_P4(i,1) = 2*pi-theta_P4(i,1); end
  

  if (theta_P4(i,1)>=theta_P3(i,1)) && f2==1
    th = theta_P3(i,1):pi/1000:theta_P4(i,1);
  end
  
  if (theta_P4(i,1)<theta_P3(i,1)) && f2==1
    th = 0;
    YP3_time(i,1) = YP4_time(i,1);
    XP3_time(i,1) = XP4_time(i,1);
  end
  
  
  if (sign(YP3_time(i,1)- YC2(i,1)) == sign(YP4_time(i,1)- YC2(i,1))) && f2==-1 && (theta_P3(i,1)>=theta_P4(i,1))
      th =  theta_P4(i,1):pi/1000:theta_P3(i,1); 
  end
  
  if (sign(YP3_time(i,1)- YC2(i,1)) == sign(YP4_time(i,1)- YC2(i,1))) && f2==-1 && (theta_P3(i,1)<theta_P4(i,1))
     th =0;
     YP3_time(i,1) = YP4_time(i,1);
     XP3_time(i,1) = XP4_time(i,1);
  end

  if (sign(YP3_time(i,1)- YC2(i,1)) ~= sign(YP4_time(i,1)- YC2(i,1))) && f2==-1
      theta_P4(i,1) = theta_P4(i,1) -2*pi;
      th = theta_P4(i,1):pi/1000:theta_P3(i,1); 
  end


   
  x = XC2(i,1);
  y = YC2(i,1);
  r = R2;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj4 = plot(xunit, yunit,'color',[1 0 0]);
  hold on
  
% Lmt5   
  line([XP4_time(i,1) XI_time(i,1)],[YP4_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on  
  
  
elseif (theta1_time(i,1)<0 && theta2_time(i,1)>=0) %-----------------------------------------------
% Lmt1    
  line([XO_time(i,1) XP4_time(i,1)],[YO_time(i,1) YP4_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on  
 
  Lmt1_2(i,1) = sqrt((XO_time(i,1)-XP4_time(i,1))^2+(YO_time(i,1)-YP4_time(i,1))^2);   
  
  
% Lmt2   
  u = [XP4_time(i,1), YP4_time(i,1)]-[XC2(i,1), YC2(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P4(i,1) = real(acos(CosTheta));
  
  u = [XP5_time(i,1), YP5_time(i,1)]-[XC2(i,1), YC2(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P5(i,1) = real(acos(CosTheta));
  
  if  YP4_time(i,1)< YC2(i,1); theta_P4(i,1) = 2*pi-theta_P4(i,1); end
  if  YP5_time(i,1)< YC2(i,1); theta_P5(i,1) = 2*pi-theta_P5(i,1); end
  
  if (theta_P5(i,1)>=theta_P4(i,1)) && f2==1
    th = theta_P4(i,1):pi/1000:theta_P5(i,1);
  end
  
  if (theta_P5(i,1)<theta_P4(i,1)) && f2==1
    th =0;
    YP4_time(i,1) = YP5_time(i,1);
    XP4_time(i,1) = XP5_time(i,1);
  end
  
  
  if (sign(YP4_time(i,1)- YC2(i,1)) == sign(YP5_time(i,1)- YC2(i,1))) && f2==-1 && (theta_P4(i,1)>=theta_P5(i,1))
      th = theta_P5(i,1):pi/1000:theta_P4(i,1); 
  end
  
  if (sign(YP4_time(i,1)- YC2(i,1)) == sign(YP5_time(i,1)- YC2(i,1))) && f2==-1 && (theta_P4(i,1)<theta_P5(i,1))
     th = 0;
     YP4_time(i,1) = YP5_time(i,1);
     XP4_time(i,1) = XP5_time(i,1);
  end
  
  if (sign(YP4_time(i,1)- YC2(i,1)) ~= sign(YP5_time(i,1)- YC2(i,1))) && f2==-1
      theta_P5(i,1) = theta_P5(i,1) - 2*pi;
      th = theta_P5(i,1):pi/1000:theta_P4(i,1); 
  end
  
  x = XC2(i,1);
  y = YC2(i,1);
  r = R2;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj3 = plot(xunit, yunit,'color',[1 0 0]);
  hold on    
  
  Lmt2_2(i,1) = R2*(th(end)-th(1));
  
% Lmt3    
  line([XP5_time(i,1) XI_time(i,1)],[YP5_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on 
  
  Lmt3_2(i,1) = sqrt((XI_time(i,1)-XP5_time(i,1))^2+(YI_time(i,1)-YP5_time(i,1))^2);  

  Lmt_2(i,1) = Lmt1_2(i,1) + Lmt2_2(i,1) + Lmt3_2(i,1);

elseif (theta1_time(i,1)>=0 && theta2_time(i,1)<0) %-----------------------------------------------
% Lmt1    
  line([XO_time(i,1) XP1_time(i,1)],[YO_time(i,1) YP1_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on  
    
% Lmt2   
  u = [XP1_time(i,1), YP1_time(i,1)]-[XC1(i,1), YC1(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P1(i,1) = real(acos(CosTheta));
  
  u = [XP3_time(i,1), YP3_time(i,1)]-[XC1(i,1), YC1(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P3(i,1) = real(acos(CosTheta));
  
  if  YP1_time(i,1)< YC1(i,1); theta_P1(i,1) = 2*pi-theta_P1(i,1); end
  if  YP3_time(i,1)< YC1(i,1); theta_P3(i,1) = 2*pi-theta_P3(i,1); end
  
  if (theta_P3(i,1)>=theta_P1(i,1)) && f1==1
    th = theta_P1(i,1):pi/1000:theta_P3(i,1);
  end
  
  if (theta_P3(i,1)<theta_P1(i,1)) && f1==1
    th =0;
    YP3_time(i,1) = YP1_time(i,1);
    XP3_time(i,1) = XP1_time(i,1);
  end  
  
  if (sign(YP1_time(i,1)- YC1(i,1)) == sign(YP3_time(i,1)- YC1(i,1))) && f1==-1 && (theta_P1(i,1)>=theta_P3(i,1))
      th = theta_P3(i,1):pi/1000:theta_P1(i,1); 
  end
  
  if (sign(YP1_time(i,1)- YC1(i,1)) == sign(YP3_time(i,1)- YC1(i,1))) && f1==-1 && (theta_P1(i,1)<theta_P3(i,1))
     th =0;
     YP3_time(i,1) = YP1_time(i,1);
     XP3_time(i,1) = XP1_time(i,1);
  end
  
  if (sign(YP1_time(i,1)- YC1(i,1)) ~= sign(YP3_time(i,1)- YC1(i,1))) && f1==-1
      theta_P3(i,1) = theta_P3(i,1) -2*pi;
      th = theta_P3(i,1):pi/1000:theta_P1(i,1); 
  end
  
  
  x = XC1(i,1);
  y = YC1(i,1);
  r = R1;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj3 = plot(xunit, yunit,'color',[1 0 0]);
  hold on    

% Lmt3    
  line([XP3_time(i,1) XI_time(i,1)],[YP3_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on 
  
else %----------------------------------------------------------------------------------------
% Lmt   
  line([XO_time(i,1) XI_time(i,1)],[YO_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on   
  
  Lmt_2(i,1) = sqrt((XO_time(i,1)-XI_time(i,1))^2+(YO_time(i,1)-YI_time(i,1))^2);   
  
  
end


  
  
%% figure properties 
box on
grid on
az = 0;
el = 90;
view(az,el);
axis equal
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');  


% xlim([MinX MaxX])
  ylim([MinY MaxY])

 
 

txt_title = 'Triceps Brachii Long';
title(txt_title)



hold off

%
 %% save figure to create gif file
filename = ['Muscle Simulation',' (', txt_title, ') ', '.gif'];
% h= figure(4);
drawnow
% Capture the plot as an image
frame = getframe(h);
im = frame2im(frame);
[imind,cm] = rgb2ind(im,250);
% Write to the GIF File
if f == 0
imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
else
imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.1);
end
f=f+1;
F(i) = getframe(gcf); 
%}

end  



