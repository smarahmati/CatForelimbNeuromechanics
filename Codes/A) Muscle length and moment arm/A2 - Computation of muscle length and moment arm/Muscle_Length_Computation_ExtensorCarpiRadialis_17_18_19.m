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

% Extensor Carpi Radialis (17->18) --------------------------------------------------------------------------------------------------
MN = 10;
ls = Lfarm_Mo2D;
alpha1OW = ELBa;
alpha2OW = WRTa;
alpha1OW_dot = ELBa_dot;
alpha2OW_dot = WRTa_dot;
y1OW = q2;
y2OW = q3;
y3OW = q4;
XC1OW = ELB(:,1);
YC1OW = ELB(:,2);
XC2OW = WRT(:,1);
YC2OW = WRT(:,2);
Sc_a1 = Scale_Uarm2D;
Sc_a2 = Scale_car2D;
handrule = -1; % it is 1 for right hand rule, and -1 for left hand rule

% phi1_ALL(MN,1) = -0.6;
% R1_ALL(MN,1) = 0.0094;
% phi2_ALL(MN,1) = -2.2;
% R2_ALL(MN,1) = R2_ALL(75,1)*0.9;

a1OW = a1_ALL(MN,1)*Sc_a1;
a2OW = a2_ALL(MN,1)*Sc_a2;
phi1OW = phi1_ALL(MN,1);
phi2OW = phi2_ALL(MN,1);
R1OW = (R1_ALL(MN,1)*Sc_a1 + R1_ALL(MN,1)*Scale_Uarm2D)/2;
R2OW = (R2_ALL(MN,1)*Sc_a2 + R2_ALL(MN,1)*Scale_Uarm2D)/2;
f1OW = f1_ALL(MN,1);
f2OW = f2_ALL(MN,1);
n1OW = n1_ALL(MN,1);
n2OW = n2_ALL(MN,1);


% After scaling it is possible that the equal values of R1OW and a1OW or 
% R1OW and a2OW do not be equall anymore, the following commands make sure they
% are equal even after scaling
if a1_ALL(MN,1) == R1_ALL(MN,1)
   R1OW = a1OW; 
end

if a2_ALL(MN,1) == R2_ALL(MN,1)
   R2OW = a2OW; 
end



% Modification factor to avoid muscle moment arm becomes less than joint radius
R2OW = 0.7*R2OW;



if a1OW<R1OW 
msg = 'Error occurred: a1OW<R1OW';
error(msg)
end

if a2OW<R2OW
msg = 'Error occurred: a2OW<R2OW';
error(msg)
end



for i=1:size(alpha1OW,1)
[MuscleLengthOW, MuscleVelocityOW, MuscleMomentArmProximalOW, MuscleMomentArmDistalOW] = ...
MuscleTwoJoints_JointAngles(a1OW,a2OW,phi1OW,phi2OW,R1OW,R2OW,f1OW,f2OW,n1OW,n2OW,ls,alpha1OW(i,1),alpha2OW(i,1),alpha1OW_dot(i,1),alpha2OW_dot(i,1),handrule);
MuscleLengthOW_time_JA(i,1) =  MuscleLengthOW;
MuscleVelocityOW_time_JA(i,1) =  MuscleVelocityOW;
MuscleMomentArmProximalOW_time(i,1) = MuscleMomentArmProximalOW;
MuscleMomentArmDistalOW_time(i,1) = MuscleMomentArmDistalOW;
% lmt1_time(i,1) = lmt1;
% lmt2_time(i,1) = lmt2;
% lmt3_time(i,1) = lmt3;
% theta2OW_m_time(i,1) = theta2OW_m;
% rho2OW_m1_time(i,1) = rho2OW_m1;
% rho2OW_m2_time(i,1) = rho2OW_m2;
end



for i=1:size(alpha1OW,1)
 [XO, YO, XW, YW, XP1OW, YP1OW, XP2OW, YP2OW, XP3OW, YP3OW, XP4OW, YP4OW, XP5OW, YP5OW, theta1OW, theta2OW, theta1OW_m, theta2OW_m, theta1OW_FK,theta2OW_FK, ...
  alpha1OW, alpha2OW, gamma1OW, gamma2OW, beta1OW, beta2OW, h1OW, h2OW, h3OW, rho1OW, rho2OW, rho1OW_m, rho2OW_m, MuscleLengthOW] = ...
  MuscleTwoJoints_SegmentAngles(a1OW,a2OW,phi1OW,phi2OW,R1OW,R2OW,f1OW,f2OW,n1OW,n2OW,ls,y1OW(i,1),y2OW(i,1),y3OW(i,1),XC1OW(i,1),YC1OW(i,1),XC2OW(i,1),YC2OW(i,1));

XO_time(i,1) = XO;
YO_time(i,1) = YO;

XW_time(i,1) = XW;
YW_time(i,1) = YW;

XP1OW_time(i,1) = XP1OW;
YP1OW_time(i,1) = YP1OW;

XP2OW_time(i,1) = XP2OW;
YP2OW_time(i,1) = YP2OW;

XP3OW_time(i,1) = XP3OW;
YP3OW_time(i,1) = YP3OW;

XP4OW_time(i,1) = XP4OW;
YP4OW_time(i,1) = YP4OW;

XP5OW_time(i,1) = XP5OW;
YP5OW_time(i,1) = YP5OW;

theta1OW_time(i,1) = theta1OW; % theta1OW as the presented criterion
theta2OW_time(i,1) = theta2OW; % theta2OW as the presented criterion

theta1OW_FK_time(i,1) = theta1OW_FK; % theta1OW based on forwad kinematics
theta2OW_FK_time(i,1) = theta2OW_FK; % theta2OW based on forwad kinematics

theta1OW_m_time(i,1) = theta1OW_m; % modified theta1OW based on rho1OW_m
theta2OW_m_time(i,1) = theta2OW_m; % modified theta1OW based on rho2OW_m

alpha1OW_time(i,1) = alpha1OW;
alpha2OW_time(i,1) = alpha2OW;
gamma1OW_time(i,1) = gamma1OW;
gamma2OW_time(i,1) = gamma2OW;
beta1OW_time(i,1) = beta1OW;
beta2OW_time(i,1) = beta2OW;
h1OW_time(i,1) = h1OW;
h2OW_time(i,1) = h2OW;
h3OW_time(i,1) = h3OW;
rho1OW_time(i,1) = rho1OW;
rho2OW_time(i,1) = rho2OW;
rho1OW_m_time(i,1) = rho1OW_m;
rho2OW_m_time(i,1) = rho2OW_m;

MuscleLengthOW_time_SA(i,1) =  MuscleLengthOW; % Worse -> we consider the following computation for muscle length based on forward kinematics 
end


% Extensor Carpi Radialis (18->19) --------------------------------------------------------------------
MN = 11;
y1WI = q4;
XC1WI = WRT(:,1);
YC1WI = WRT(:,2);
Sc_a1 = Scale_car2D;
Sc_a2 = Scale_car2D;
handrule = -1; % it is 1 for right hand rule, and -1 for left hand rule



a1WI = a1_ALL(MN,1)*Sc_a1;
a2WI = a2_ALL(MN,1)*Sc_a2;
phi1WI = phi1_ALL(MN,1);
phi2WI = phi2_ALL(MN,1);




for i=1:size(y1WI,1)
[MuscleLengthWI, MuscleVelocityWI, MuscleMomentArmWI] = MuscleZeroJoint_JointAngles(a1WI,a2WI,phi1WI,phi2WI);
MuscleLengthWI_time_JA(i,1) = MuscleLengthWI;
MuscleVelocityWI_time_JA(i,1) = MuscleVelocityWI;
MuscleMomentArmWI_time(i,1) = MuscleMomentArmWI;
end



for i=1:size(y1WI,1)
 [XW, YW, XI, YI, MuscleLengthWI] = ...
  MuscleZeroJoint_SegmentAngles(a1WI,a2WI,phi1WI,phi2WI,y1WI(i,1),XC1WI(i,1),YC1WI(i,1));

XW_time(i,1) = XW;
YW_time(i,1) = YW;

XI_time(i,1) = XI;
YI_time(i,1) = YI;

MuscleLengthWI_time_SA(i,1) =  MuscleLengthWI; % Worse -> we consider the follWIing computation for muscle length based on forward kinematics
end




%% Write information
cmn = 7;                                                                      % considered muscle number
mn = 'Extensor Carpi Radialis';                                               % muscle name
ml = MuscleLengthOW_time_JA + MuscleLengthWI_time_JA;                         % muscle length
mv = MuscleVelocityOW_time_JA + MuscleVelocityWI_time_JA;                     % muscle velocity
ma = [time*0, MuscleMomentArmProximalOW_time, MuscleMomentArmDistalOW_time];  % muscle momemet arm
me = [0, 1, 1];                                                               % muscle effective joints


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

R1WI = nan;
R2WI = nan;
f1WI = nan;
f2WI = nan;
n1WI = nan;
n2WI = nan;
ScaledMuscleMorphologicalParameters{MN-1, 1}.Name = mn;
TOW = table(a1OW, a2OW, phi1OW, phi2OW, R1OW, R2OW, f1OW, f2OW, n1OW, n2OW);
ScaledMuscleMorphologicalParameters{MN-1, 1}.Parameters = TOW;  

ScaledMuscleMorphologicalParameters{MN, 1}.Name = mn;
TWI = table(a1WI, a2WI, phi1WI, phi2WI, R1WI, R2WI, f1WI, f2WI, n1WI, n2WI);
ScaledMuscleMorphologicalParameters{MN, 1}.Parameters = TWI;  


save('ScaledMuscleMorphologicalParameters.mat', 'ScaledMuscleMorphologicalParameters');



%% Plot

figure(1)
plot(time, MuscleLengthOW_time_JA + MuscleLengthWI_time_JA,'-k','LineWidth',1)
hold on
plot(time, MuscleLengthOW_time_SA + MuscleLengthWI_time_SA,'Ob','LineWidth',1, 'MarkerSize', 5)
hold on
grid on
xlabel('Time(s)');
ylabel('Muscle Length (m)');
legend('Joint Angles', 'Forward Kinematics')
title('Comparision of muscle length based on joint angles and forward kinematics')
% set(gca,'XTick',[])
savefig('MuscleLength_ExtensorCarpiRadialis.fig')


figure(2)
plot(time, MuscleVelocityOW_time_JA + MuscleVelocityWI_time_JA,'-k','LineWidth',1)
hold on
grid on
xlabel('Time(s)');
ylabel('Muscle Velocity (m/s)');
legend('Joint Angles')
% set(gca,'XTick',[])
savefig('MuscleVelocity_ExtensorCarpiRadialis.fig')



figure(3)
plot(time, MuscleMomentArmProximalOW_time,'-k','LineWidth',1)
hold on
plot(time, MuscleMomentArmDistalOW_time,'-b','LineWidth',1)
hold on
plot(time, R1OW+time*0,'--k','LineWidth',1)
hold on
plot(time, R2OW+time*0,'--b','LineWidth',1)
grid on
xlabel('Time(s)');
ylabel('Muscle Moment Arm (m)');
legend('Proximal Joint', 'Distal Joint','Proximal Joint Radius','Distal Joint Radius')
title('Muscle Moment Arm based on joint angles')
% set(gca,'XTick',[])
savefig('MuscleMomentArm_ExtensorCarpiRadialis.fig')




%{
figure(3)
plot(time, MuscleMomentArmProximalOW_time,'-k','LineWidth',1)
hold on
grid on
xlabel('Time(s)');
ylabel('Proximal Muscle Moment Arm (m)');
% legend('Proximal Joint')
title('Muscle Moment Arm based on joint angles')
% set(gca,'XTick',[])

figure(4)
plot(time, MuscleMomentArmDistalOW_time,'-b','LineWidth',1)
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


h = figure(5);
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
ATP1 = scatter(XP1OW_time(i,1), YP1OW_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP2 = scatter(XP2OW_time(i,1), YP2OW_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP3 = scatter(XP3OW_time(i,1), YP3OW_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP4 = scatter(XP4OW_time(i,1), YP4OW_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP5 = scatter(XP5OW_time(i,1), YP5OW_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATI6 = scatter(XW_time(i,1), YW_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATI7 = scatter(XI_time(i,1), YI_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
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
x = XC1OW(i,1);
y = YC1OW(i,1);
r = R1OW;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
Cj1 = plot(xunit, yunit,'color','cyan');

x = XC2OW(i,1);
y = YC2OW(i,1);
r = R2OW;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
Cj2 = plot(xunit, yunit,'color','cyan');


%% Attachment points

%---------------------------Origin to Way---------------------------------------------

if (theta1OW_time(i,1)>=0 && theta2OW_time(i,1)>=0) %-----------------------------------------------
% Lmt1    
  line([XO_time(i,1) XP1OW_time(i,1)],[YO_time(i,1) YP1OW_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on
 
% Lmt2   
  u = [XP1OW_time(i,1), YP1OW_time(i,1)]-[XC1OW(i,1), YC1OW(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P1(i,1) = real(acos(CosTheta));
  
  u = [XP2OW_time(i,1), YP2OW_time(i,1)]-[XC1OW(i,1), YC1OW(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P2(i,1) = real(acos(CosTheta));
  
  if  YP1OW_time(i,1)< YC1OW(i,1); theta_P1(i,1) = 2*pi-theta_P1(i,1); end
  if  YP2OW_time(i,1)< YC1OW(i,1); theta_P2(i,1) = 2*pi-theta_P2(i,1); end
  
  if (theta_P2(i,1)>=theta_P1(i,1)) && f1OW==1
    th = theta_P1(i,1):pi/1000:theta_P2(i,1);
  end
  
  if (theta_P2(i,1)<theta_P1(i,1)) && f1OW==1
    th = 0;
    YP2OW_time(i,1) = YP1OW_time(i,1);
    XP2OW_time(i,1) = XP1OW_time(i,1);
  end
  
  if (sign(YP1OW_time(i,1)- YC1OW(i,1)) == sign(YP2OW_time(i,1)- YC1OW(i,1))) && f1OW==-1 && (theta_P1(i,1)>=theta_P2(i,1))
      th = theta_P2(i,1):pi/1000:theta_P1(i,1); 
  end
  
  if (sign(YP1OW_time(i,1)- YC1OW(i,1)) == sign(YP2OW_time(i,1)- YC1OW(i,1))) && f1OW==-1 && (theta_P1(i,1)<theta_P2(i,1))
     th = 0;
     YP2OW_time(i,1) = YP1OW_time(i,1);
     XP2OW_time(i,1) = XP1OW_time(i,1);
  end

  if (sign(YP1OW_time(i,1)- YC1OW(i,1)) ~= sign(YP2OW_time(i,1)- YC1OW(i,1))) && f1OW==-1
      theta_P2(i,1) = theta_P2(i,1) -2*pi;
      th = theta_P2(i,1):pi/1000:theta_P1(i,1); 
  end

  
  x = XC1OW(i,1);
  y = YC1OW(i,1);
  r = R1OW;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj3 = plot(xunit, yunit,'color',[1 0 0]);
  hold on
  
% Lmt3   
  line([XP2OW_time(i,1) XP3OW_time(i,1)],[YP2OW_time(i,1) YP3OW_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on
  
% Lmt4     
  u = [XP3OW_time(i,1), YP3OW_time(i,1)]-[XC2OW(i,1), YC2OW(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P3(i,1) = real(acos(CosTheta));
  
  u = [XP4OW_time(i,1), YP4OW_time(i,1)]-[XC2OW(i,1), YC2OW(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P4(i,1) = real(acos(CosTheta));
  
  if  YP3OW_time(i,1)< YC2OW(i,1); theta_P3(i,1) = 2*pi-theta_P3(i,1); end
  if  YP4OW_time(i,1)< YC2OW(i,1); theta_P4(i,1) = 2*pi-theta_P4(i,1); end
  

  if (theta_P4(i,1)>=theta_P3(i,1)) && f2OW==1
    th = theta_P3(i,1):pi/1000:theta_P4(i,1);
  end
  
  if (theta_P4(i,1)<theta_P3(i,1)) && f2OW==1
    th = 0;
    YP3OW_time(i,1) = YP4OW_time(i,1);
    XP3OW_time(i,1) = XP4OW_time(i,1);
  end
  
  
  if (sign(YP3OW_time(i,1)- YC2OW(i,1)) == sign(YP4OW_time(i,1)- YC2OW(i,1))) && f2OW==-1 && (theta_P3(i,1)>=theta_P4(i,1))
      th =  theta_P4(i,1):pi/1000:theta_P3(i,1); 
  end
  
  if (sign(YP3OW_time(i,1)- YC2OW(i,1)) == sign(YP4OW_time(i,1)- YC2OW(i,1))) && f2OW==-1 && (theta_P3(i,1)<theta_P4(i,1))
     th =0;
     YP3OW_time(i,1) = YP4OW_time(i,1);
     XP3OW_time(i,1) = XP4OW_time(i,1);
  end

  if (sign(YP3OW_time(i,1)- YC2OW(i,1)) ~= sign(YP4OW_time(i,1)- YC2OW(i,1))) && f2OW==-1
      theta_P4(i,1) = theta_P4(i,1) -2*pi;
      th = theta_P4(i,1):pi/1000:theta_P3(i,1); 
  end


   
  x = XC2OW(i,1);
  y = YC2OW(i,1);
  r = R2OW;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj4 = plot(xunit, yunit,'color',[1 0 0]);
  hold on
  
% Lmt5   
  line([XP4OW_time(i,1) XW_time(i,1)],[YP4OW_time(i,1) YW_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on  
  
  
elseif (theta1OW_time(i,1)<0 && theta2OW_time(i,1)>=0) %-----------------------------------------------
% Lmt1    
  line([XO_time(i,1) XP4OW_time(i,1)],[YO_time(i,1) YP4OW_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on  
 
  Lmt1_2(i,1) = sqrt((XO_time(i,1)-XP4OW_time(i,1))^2+(YO_time(i,1)-YP4OW_time(i,1))^2);   
  
  
% Lmt2   
  u = [XP4OW_time(i,1), YP4OW_time(i,1)]-[XC2OW(i,1), YC2OW(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P4(i,1) = real(acos(CosTheta));
  
  u = [XP5OW_time(i,1), YP5OW_time(i,1)]-[XC2OW(i,1), YC2OW(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P5(i,1) = real(acos(CosTheta));
  
  if  YP4OW_time(i,1)< YC2OW(i,1); theta_P4(i,1) = 2*pi-theta_P4(i,1); end
  if  YP5OW_time(i,1)< YC2OW(i,1); theta_P5(i,1) = 2*pi-theta_P5(i,1); end
  
  if (theta_P5(i,1)>=theta_P4(i,1)) && f2OW==1
    th = theta_P4(i,1):pi/1000:theta_P5(i,1);
  end
  
  if (theta_P5(i,1)<theta_P4(i,1)) && f2OW==1
    th =0;
    YP4OW_time(i,1) = YP5OW_time(i,1);
    XP4OW_time(i,1) = XP5OW_time(i,1);
  end
  
  
  if (sign(YP4OW_time(i,1)- YC2OW(i,1)) == sign(YP5OW_time(i,1)- YC2OW(i,1))) && f2OW==-1 && (theta_P4(i,1)>=theta_P5(i,1))
      th = theta_P5(i,1):pi/1000:theta_P4(i,1); 
  end
  
  if (sign(YP4OW_time(i,1)- YC2OW(i,1)) == sign(YP5OW_time(i,1)- YC2OW(i,1))) && f2OW==-1 && (theta_P4(i,1)<theta_P5(i,1))
     th = 0;
     YP4OW_time(i,1) = YP5OW_time(i,1);
     XP4OW_time(i,1) = XP5OW_time(i,1);
  end
  
  if (sign(YP4OW_time(i,1)- YC2OW(i,1)) ~= sign(YP5OW_time(i,1)- YC2OW(i,1))) && f2OW==-1
      theta_P5(i,1) = theta_P5(i,1) - 2*pi;
      th = theta_P5(i,1):pi/1000:theta_P4(i,1); 
  end
  
  x = XC2OW(i,1);
  y = YC2OW(i,1);
  r = R2OW;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj3 = plot(xunit, yunit,'color',[1 0 0]);
  hold on    
  
  Lmt2_2(i,1) = R2OW*(th(end)-th(1));
  
% Lmt3    
  line([XP5OW_time(i,1) XW_time(i,1)],[YP5OW_time(i,1) YW_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on 
  
  Lmt3_2(i,1) = sqrt((XW_time(i,1)-XP5OW_time(i,1))^2+(YW_time(i,1)-YP5OW_time(i,1))^2);  

  Lmt_2(i,1) = Lmt1_2(i,1) + Lmt2_2(i,1) + Lmt3_2(i,1);

elseif (theta1OW_time(i,1)>=0 && theta2OW_time(i,1)<0) %-----------------------------------------------
% Lmt1    
  line([XO_time(i,1) XP1OW_time(i,1)],[YO_time(i,1) YP1OW_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on  
    
% Lmt2   
  u = [XP1OW_time(i,1), YP1OW_time(i,1)]-[XC1OW(i,1), YC1OW(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P1(i,1) = real(acos(CosTheta));
  
  u = [XP3OW_time(i,1), YP3OW_time(i,1)]-[XC1OW(i,1), YC1OW(i,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P3(i,1) = real(acos(CosTheta));
  
  if  YP1OW_time(i,1)< YC1OW(i,1); theta_P1(i,1) = 2*pi-theta_P1(i,1); end
  if  YP3OW_time(i,1)< YC1OW(i,1); theta_P3(i,1) = 2*pi-theta_P3(i,1); end
  
  if (theta_P3(i,1)>=theta_P1(i,1)) && f1OW==1
    th = theta_P1(i,1):pi/1000:theta_P3(i,1);
  end
  
  if (theta_P3(i,1)<theta_P1(i,1)) && f1OW==1
    th =0;
    YP3OW_time(i,1) = YP1OW_time(i,1);
    XP3OW_time(i,1) = XP1OW_time(i,1);
  end  
  
  if (sign(YP1OW_time(i,1)- YC1OW(i,1)) == sign(YP3OW_time(i,1)- YC1OW(i,1))) && f1OW==-1 && (theta_P1(i,1)>=theta_P3(i,1))
      th = theta_P3(i,1):pi/1000:theta_P1(i,1); 
  end
  
  if (sign(YP1OW_time(i,1)- YC1OW(i,1)) == sign(YP3OW_time(i,1)- YC1OW(i,1))) && f1OW==-1 && (theta_P1(i,1)<theta_P3(i,1))
     th =0;
     YP3OW_time(i,1) = YP1OW_time(i,1);
     XP3OW_time(i,1) = XP1OW_time(i,1);
  end
  
  if (sign(YP1OW_time(i,1)- YC1OW(i,1)) ~= sign(YP3OW_time(i,1)- YC1OW(i,1))) && f1OW==-1
      theta_P3(i,1) = theta_P3(i,1) -2*pi;
      th = theta_P3(i,1):pi/1000:theta_P1(i,1); 
  end
  
  
  x = XC1OW(i,1);
  y = YC1OW(i,1);
  r = R1OW;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj3 = plot(xunit, yunit,'color',[1 0 0]);
  hold on    

% Lmt3    
  line([XP3OW_time(i,1) XW_time(i,1)],[YP3OW_time(i,1) YW_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on 
  
else %----------------------------------------------------------------------------------------
% Lmt   
  line([XO_time(i,1) XW_time(i,1)],[YO_time(i,1) YW_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on   
  
  Lmt_2(i,1) = sqrt((XO_time(i,1)-XW_time(i,1))^2+(YO_time(i,1)-YW_time(i,1))^2);   
  
  
  
  
%---------------------------Way to Insertion---------------------------------------------
% Lmt   
  line([XW_time(i,1) XI_time(i,1)],[YW_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on    
  
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

 
 

txt_title = 'Extensor Carpi Radialis (Revised)';
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