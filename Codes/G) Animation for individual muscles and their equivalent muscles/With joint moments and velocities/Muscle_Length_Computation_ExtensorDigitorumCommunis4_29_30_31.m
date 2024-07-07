% clear
% clc

MuscleMomentArmOW_time = [];
MuscleMomentArmWI_time = [];

% f = gcf;
% exportgraphics(f,'Fig1_Original.png','Resolution',600)
MotionData = load('MotionData.mat');
MusculoskeletalData = load('MusculoskeletalData.mat');

MuscleCluster = load('MuscleCluster.mat');
MuscleCluster = MuscleCluster.MuscleCluster;

C1 = MuscleCluster{1};
C2 = MuscleCluster{2};
C3 = MuscleCluster{3};
C4 = MuscleCluster{4};
C5 = MuscleCluster{5};
C6 = MuscleCluster{6};
C7 = MuscleCluster{7};
C8 = MuscleCluster{8};
C9 = MuscleCluster{9};

cmn = 10;              % considered muscle number 

if ismember(cmn, C1) 
MC = cmap(1,:);
elseif ismember(cmn, C2) 
MC = cmap(2,:);
elseif ismember(cmn, C3) 
MC = cmap(3,:);
elseif ismember(cmn, C4) 
MC = cmap(4,:);
elseif ismember(cmn, C5) 
MC = cmap(5,:);
elseif ismember(cmn, C6) 
MC = cmap(6,:);
elseif ismember(cmn, C7) 
MC = cmap(7,:);
elseif ismember(cmn, C8) 
MC = cmap(8,:);
elseif ismember(cmn, C9) 
MC = cmap(9,:);
end



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


% Extensor Digitorum Communis 4(29->30) --------------------------------------------------------------------
MN = 18;
alpha1OW = ELBa;
alpha1OW_dot = ELBa_dot;
y1OW = q2;
y2OW = q3;
XC1OW = ELB(:,1);
YC1OW = ELB(:,2);
Sc_a1 = Scale_Uarm2D;
Sc_a2 = Scale_farm2D;
handrule = -1; % it is 1 for right hand rule, and -1 for left hand rule


a1OW = a1_ALL(MN,1)*Sc_a1;
a2OW = a2_ALL(MN,1)*Sc_a2;
phi1OW = phi1_ALL(MN,1);
phi2OW = phi2_ALL(MN,1);
R1OW = (R1_ALL(MN,1)*Sc_a1 + R1_ALL(MN,1)*Sc_a2)/2;
R2OW = (R2_ALL(MN,1)*Sc_a1 + R2_ALL(MN,1)*Sc_a2)/2;
f1OW = f1_ALL(MN,1);
f2OW = f2_ALL(MN,1);
n1OW = n1_ALL(MN,1);
n2OW = n2_ALL(MN,1);

% After scaling it is possible that the equal values of R1 and a1 or 
% R1 and a2 do not be equall anymore, the following commands make sure they
% are equal even after scaling
if a2_ALL(MN,1) == R1_ALL(MN,1)
   R1OW = a2OW; 
elseif  a1_ALL(MN,1) == R1_ALL(MN,1)
   R1OW = a1OW;
end



if a1OW<R1OW 
msg = 'Error occurred: a1OW<R1OW';
error(msg)
end

if a2OW<R1OW
msg = 'Error occurred: a2OW<R1OW';
error(msg)
end



for i=1:size(alpha1OW,1)
[MuscleLengthOW, MuscleVelocityOW, MuscleMomentArmOW] = MuscleOneJoint_JointAngles(a1OW,a2OW,phi1OW,phi2OW,R1OW,f1OW,n1OW,alpha1OW(i,1),alpha1OW_dot(i,1),handrule);
MuscleLengthOW_time_JA(i,1) = MuscleLengthOW;
MuscleVelocityOW_time_JA(i,1) = MuscleVelocityOW;
MuscleMomentArmOW_time(i,1) = MuscleMomentArmOW;
end




for i=1:size(alpha1OW,1)
 [XO, YO, XW, YW, XP1OW, YP1OW, XP2OW, YP2OW, theta1OW, theta1OW_FK, alpha1OW, gamma1OW, gamma2OW, beta1OW, beta2OW, MuscleLengthOW] = ...
  MuscleOneJoint_SegmentAngles(a1OW,a2OW,phi1OW,phi2OW,R1OW,f1OW,n1OW,y1OW(i,1),y2OW(i,1),XC1OW(i,1),YC1OW(i,1));

XO_time(i,1) = XO;
YO_time(i,1) = YO;

XW_time(i,1) = XW;
YW_time(i,1) = YW;

XP1OW_time(i,1) = XP1OW;
YP1OW_time(i,1) = YP1OW;

XP2OW_time(i,1) = XP2OW;
YP2OW_time(i,1) = YP2OW;


theta1OW_time(i,1) = theta1OW; % theta1OW as the presented criterion

theta1OW_FK_time(i,1) = theta1OW_FK; % theta1OW based on forwad kinematics


alpha1OW_time(i,1) = alpha1OW;
gamma1OW_time(i,1) = gamma1OW;
gamma2OW_time(i,1) = gamma2OW;
beta1OW_time(i,1) = beta1OW;
beta2OW_time(i,1) = beta2OW;

MuscleLengthOW_time_SA(i,1) =  MuscleLengthOW; % Worse -> we consider the following computation for muscle length based on forward kinematics 
end



% Extensor Digitorum Communis 4(30->31)  --------------------------------------------------------------------
MN = 19;
alpha1WI = WRTa;
alpha1WI_dot = WRTa_dot;
y1WI = q3;
y2WI = q4;
XC1WI = WRT(:,1);
YC1WI = WRT(:,2);
Sc_a1 = Scale_farm2D;
Sc_a2 = Scale_car2D;


a1WI = a1_ALL(MN,1)*Sc_a1;
a2WI = a2_ALL(MN,1)*Sc_a2;
phi1WI = phi1_ALL(MN,1);
phi2WI = phi2_ALL(MN,1);
R1WI = (R1_ALL(MN,1)*Sc_a1 + R1_ALL(MN,1)*Sc_a2)/2;
R2WI = (R2_ALL(MN,1)*Sc_a1 + R2_ALL(MN,1)*Sc_a2)/2;
f1WI = f1_ALL(MN,1);
f2WI = f2_ALL(MN,1);
n1WI = n1_ALL(MN,1);
n2WI = n2_ALL(MN,1);

% After scaling it is possible that the equal values of R1 and a1 or 
% R1 and a2 do not be equall anymore, the following commands make sure they
% are equal even after scaling
if a2_ALL(MN,1) == R1_ALL(MN,1)
   R1WI = a2WI; 
elseif  a1_ALL(MN,1) == R1_ALL(MN,1)
   R1WI = a1WI;
end

% Modification factor to avoid muscle moment arm becomes less than joint radius
R1WI = 0.8*R1WI;

if a1WI<R1WI 
msg = 'Error occurred: a1WI<R1WI';
error(msg)
end

if a2WI<R1WI
msg = 'Error occurred: a2WI<R1WI';
error(msg)
end



for i=1:size(alpha1WI,1)
[MuscleLengthWI, MuscleVelocityWI, MuscleMomentArmWI] = MuscleOneJoint_JointAngles(a1WI,a2WI,phi1WI,phi2WI,R1WI,f1WI,n1WI,alpha1WI(i,1),alpha1WI_dot(i,1),handrule);
MuscleLengthWI_time_JA(i,1) = MuscleLengthWI;
MuscleVelocityWI_time_JA(i,1) = MuscleVelocityWI;
MuscleMomentArmWI_time(i,1) = MuscleMomentArmWI;
end



for i=1:size(alpha1WI,1)
 [XW, YW, XI, YI, XP1WI, YP1WI, XP2WI, YP2WI, theta1WI, theta1WI_FK, alpha1WI, gamma1WI, gamma2WI, beta1WI, beta2WI, MuscleLengthWI] = ...
  MuscleOneJoint_SegmentAngles(a1WI,a2WI,phi1WI,phi2WI,R1WI,f1WI,n1WI,y1WI(i,1),y2WI(i,1),XC1WI(i,1),YC1WI(i,1));

XW_time(i,1) = XW;
YW_time(i,1) = YW;

XI_time(i,1) = XI;
YI_time(i,1) = YI;

XP1WI_time(i,1) = XP1WI;
YP1WI_time(i,1) = YP1WI;

XP2WI_time(i,1) = XP2WI;
YP2WI_time(i,1) = YP2WI;


theta1WI_time(i,1) = theta1WI; % theta1WI as the presented criterion

theta1WI_FK_time(i,1) = theta1WI_FK; % theta1WI based on forwad kinematics


alpha1WI_time(i,1) = alpha1WI;
gamma1WI_time(i,1) = gamma1WI;
gamma2WI_time(i,1) = gamma2WI;
beta1WI_time(i,1) = beta1WI;
beta2WI_time(i,1) = beta2WI;

MuscleLengthWI_time_SA(i,1) =  MuscleLengthWI; % Worse -> we consider the follWIing computation for muscle length based on forward kinematics 
end


%% Write information
mn = 'Extensor Digitorum Communis 4';                           % muscle name
ml = MuscleLengthOW_time_JA + MuscleLengthWI_time_JA;           % muscle length
mv = MuscleVelocityOW_time_JA + MuscleVelocityWI_time_JA;       % muscle velocity
ma = [time*0, MuscleMomentArmOW_time, MuscleMomentArmWI_time];  % muscle momemet arm
me = [0, 1, 1];                                                 % muscle effective joints


MuscleLengthVelocityMomentArm = load('MuscleLengthVelocityMomentArm.mat');
MuscleLengthVelocityMomentArm = MuscleLengthVelocityMomentArm.MuscleLengthVelocityMomentArm;

%-----Muscle name, length and moment mrm ---------------------------------
MuscleLengthVelocityMomentArm{cmn,1}.Name = mn;
MuscleLengthVelocityMomentArm{cmn,1}.Length = ml;
MuscleLengthVelocityMomentArm{cmn,1}.Velocity = mv;
MuscleLengthVelocityMomentArm{cmn,1}.MomentArm = ma;
MuscleLengthVelocityMomentArm{cmn,1}.EffectiveJoints = me;

% save('MuscleLengthVelocityMomentArm.mat', 'MuscleLengthVelocityMomentArm');



LSCP = SCP;
LSHL = SHL;
LELB = ELB;
LWRT = WRT;
LMCP = MCP;
LFT = FT;




%% Simulation


h = figure(3);
% pause(2);
       
      
ms = 20; % Marker Size      
% Joint Centers
% s1 = scatter(LSCP(ii,1),LSCP(ii,2),ms,'MarkerFaceColor',[0.5, 0.5, 0],'MarkerEdgeColor',[0.5, 0.5, 0]);
% hold on
% alpha(s1,0.7)

s2 = scatter(LSHL(ii,1),LSHL(ii,2),ms,'MarkerFaceColor',[1, 0, 0],'MarkerEdgeColor',[1, 0, 0]);
hold on
alpha(s2,0.7)

s3 = scatter(LELB(ii,1),LELB(ii,2),ms,'MarkerFaceColor',[0, 1, 0],'MarkerEdgeColor',[0, 1, 0]);
hold on
alpha(s3,0.7)

s4 = scatter(LWRT(ii,1),LWRT(ii,2),ms,'MarkerFaceColor',[0, 0, 1],'MarkerEdgeColor',[0, 0, 1]);
hold on
alpha(s4,0.7)

% s5 = scatter(LMCP(ii,1),LMCP(ii,2),ms,'MarkerFaceColor',[0.5, 0, 0.5],'MarkerEdgeColor',[0.5, 0, 0.5]);
% hold on
% alpha(s5,0.7)
% 
% s6 = scatter(LFT(ii,1),LFT(ii,2),ms,'MarkerFaceColor',[0, 0.5, 0.5],'MarkerEdgeColor',[0, 0.5, 0.5]);
% hold on
% alpha(s6,0.7)


% Attachment Points
% ATO = scatter(XO_time(ii,1), YO_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
% hold on
% ATP1OW = scatter(XP1OW_time(ii,1), YP1OW_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
% hold on
% ATP2OW = scatter(XP2OW_time(ii,1), YP2OW_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
% hold on
% ATW = scatter(XW_time(ii,1), YW_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
% hold on
% ATP1WI = scatter(XP1WI_time(ii,1), YP1WI_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
% hold on
% ATP2WI = scatter(XP2WI_time(ii,1), YP2WI_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
% hold on
% ATI = scatter(XI_time(ii,1), YI_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
 
 %% Line Model
 
 
 %------------------  left fore limbs -----------------------------------
 Li = line([LSCP(ii,1) LSHL(ii,1)],[LSCP(ii,2) LSHL(ii,2)],'color',[0 0 0],'LineWidth',1,'LineStyle','-');
 hold on
 Li = line([LSHL(ii,1) LELB(ii,1)],[LSHL(ii,2) LELB(ii,2)],'color',[0 0 0],'LineWidth',1,'LineStyle','-');
 hold on
 Li = line([LELB(ii,1) LWRT(ii,1)],[LELB(ii,2) LWRT(ii,2)],'color',[0 0 0],'LineWidth',1,'LineStyle','-');
 hold on
 Li = line([LWRT(ii,1) LMCP(ii,1)],[LWRT(ii,2) LMCP(ii,2)],'color',[0 0 0],'LineWidth',1,'LineStyle','-');
 hold on
%  Li = line([LMCP(ii,1) LFT(ii,1)],[LMCP(ii,2) LFT(ii,2)],'color',[0 0 0],'LineWidth',1,'LineStyle','-');
 hold on

% FA = 1000; 
% quiver3(P1_S(ii,1),P1_S(ii,2),P1_S(ii,3),FA*Trunk_normal(ii,1),FA*Trunk_normal(ii,2),FA*Trunk_normal(ii,3),'LineWidth',2)
% hold on
% quiver3(P1_S(ii,1),P1_S(ii,2),P1_S(ii,3),FA*Pelvis_normal(ii,1),FA*Pelvis_normal(ii,2),FA*Pelvis_normal(ii,3),'LineWidth',2)


%% Joint surface

th = 0:pi/1000:2*pi;
x = XC1OW(ii,1);
y = YC1OW(ii,1);
r = R1OW;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
% Cj1OW = plot(xunit, yunit,'color','cyan');


x = XC1WI(ii,1);
y = YC1WI(ii,1);
r = R1WI;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
% Cj1WI = plot(xunit, yunit,'color','cyan');



%% Attachment points

%---------------------------Origin to Way---------------------------------------------
if (theta1OW_time(ii,1)>=0)
% Lmt1    
  Li = line([XO_time(ii,1) XP1OW_time(ii,1)],[YO_time(ii,1) YP1OW_time(ii,1)],'color',[1 0 0],'LineWidth',1,'LineStyle','-');
  Li.Color=[MC,0.5];
  hold on

% Lmt2   
  u = [XP1OW_time(ii,1), YP1OW_time(ii,1)]-[XC1OW(ii,1), YC1OW(ii,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P1(ii,1) = real(acos(CosTheta));
  
  u = [XP2OW_time(ii,1), YP2OW_time(ii,1)]-[XC1OW(ii,1), YC1OW(ii,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P2(ii,1) = real(acos(CosTheta));
  
  if  YP1OW_time(ii,1)< YC1OW(ii,1); theta_P1(ii,1) = 2*pi-theta_P1(ii,1); end
  if  YP2OW_time(ii,1)< YC1OW(ii,1); theta_P2(ii,1) = 2*pi-theta_P2(ii,1); end
  
  if (theta_P2(ii,1)>=theta_P1(ii,1)) && f1OW==1
    th = theta_P1(ii,1):pi/1000:theta_P2(ii,1);
  end
  
  if (theta_P2(ii,1)<theta_P1(ii,1)) && f1OW==1 && gamma1OW==0
    th = 0;
    YP2OW_time(ii,1) = YP1OW_time(ii,1);
    XP2OW_time(ii,1) = XP1OW_time(ii,1);
  end  
  
  if (theta_P2(ii,1)<theta_P1(ii,1)) && f1OW==1 && gamma2OW==0
    th = 0;
    YP1OW_time(ii,1) = YP2OW_time(ii,1);
    XP1OW_time(ii,1) = XP2OW_time(ii,1);
  end  
  
  if (sign(YP1OW_time(ii,1)- YC1OW(ii,1)) == sign(YP2OW_time(ii,1)- YC1OW(ii,1))) && f1OW==-1 && (theta_P1(ii,1)>=theta_P2(ii,1))
      th = theta_P2(ii,1):pi/1000:theta_P1(ii,1); 
  end
  
  if (sign(YP1OW_time(ii,1)- YC1OW(ii,1)) == sign(YP2OW_time(ii,1)- YC1OW(ii,1))) && f1OW==-1 && (theta_P1(ii,1)<theta_P2(ii,1)) && gamma1OW==0
     th = 0;
     YP2OW_time(ii,1) = YP1OW_time(ii,1);
     XP2OW_time(ii,1) = XP1OW_time(ii,1);
  end
  
  if (sign(YP1OW_time(ii,1)- YC1OW(ii,1)) == sign(YP2OW_time(ii,1)- YC1OW(ii,1))) && f1OW==-1 && (theta_P1(ii,1)<theta_P2(ii,1)) && gamma2OW==0
     th = 0;
     YP1OW_time(ii,1) = YP2OW_time(ii,1);
     XP1OW_time(ii,1) = XP2OW_time(ii,1);
  end
  
  if (sign(YP1OW_time(ii,1)- YC1OW(ii,1)) ~= sign(YP2OW_time(ii,1)- YC1OW(ii,1))) && f1OW==-1
      theta_P2(ii,1) = theta_P2(ii,1) -2*pi;
      th = theta_P2(ii,1):pi/1000:theta_P1(ii,1); 
  end
      
  
  x = XC1OW(ii,1);
  y = YC1OW(ii,1);
  r = R1OW;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj2 = plot(xunit, yunit,'color',[1 0 0],'LineWidth',1,'LineStyle','-');
  Cj2.Color=[MC,0.5];
  hold on
  
  
% Lmt3   
  Li = line([XP2OW_time(ii,1) XW_time(ii,1)],[YP2OW_time(ii,1) YW_time(ii,1)],'color',[1 0 0],'LineWidth',1,'LineStyle','-');
  Li.Color=[MC,0.5];
  hold on  
  
  
else
% Lmt   
  Li = line([XO_time(ii,1) XW_time(ii,1)],[YO_time(ii,1) YW_time(ii,1)],'color',[1 0 0],'LineWidth',1,'LineStyle','-');
  Li.Color=[MC,0.5];
  hold on   
  
end
  
  

%---------------------------Way to Insertion---------------------------------------------
if (theta1WI_time(ii,1)>=0)
% Lmt1    
  Li = line([XW_time(ii,1) XP1WI_time(ii,1)],[YW_time(ii,1) YP1WI_time(ii,1)],'color',[1 0 0],'LineWidth',1,'LineStyle','-');
  Li.Color=[MC,0.5];
  hold on

% Lmt2   
  u = [XP1WI_time(ii,1), YP1WI_time(ii,1)]-[XC1WI(ii,1), YC1WI(ii,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P1(ii,1) = real(acos(CosTheta));
  
  u = [XP2WI_time(ii,1), YP2WI_time(ii,1)]-[XC1WI(ii,1), YC1WI(ii,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P2(ii,1) = real(acos(CosTheta));
  
  if  YP1WI_time(ii,1)< YC1WI(ii,1); theta_P1(ii,1) = 2*pi-theta_P1(ii,1); end
  if  YP2WI_time(ii,1)< YC1WI(ii,1); theta_P2(ii,1) = 2*pi-theta_P2(ii,1); end
  
  if (theta_P2(ii,1)>=theta_P1(ii,1)) && f1WI==1
    th = theta_P1(ii,1):pi/1000:theta_P2(ii,1);
  end
  
  if (theta_P2(ii,1)<theta_P1(ii,1)) && f1WI==1 && gamma1WI==0
    th = 0;
    YP2WI_time(ii,1) = YP1WI_time(ii,1);
    XP2WI_time(ii,1) = XP1WI_time(ii,1);
  end  
  
  if (theta_P2(ii,1)<theta_P1(ii,1)) && f1WI==1 && gamma2WI==0
    th = 0;
    YP1WI_time(ii,1) = YP2WI_time(ii,1);
    XP1WI_time(ii,1) = XP2WI_time(ii,1);
  end  
  
  if (sign(YP1WI_time(ii,1)- YC1WI(ii,1)) == sign(YP2WI_time(ii,1)- YC1WI(ii,1))) && f1WI==-1 && (theta_P1(ii,1)>=theta_P2(ii,1))
      th = theta_P2(ii,1):pi/1000:theta_P1(ii,1); 
  end
  
  if (sign(YP1WI_time(ii,1)- YC1WI(ii,1)) == sign(YP2WI_time(ii,1)- YC1WI(ii,1))) && f1WI==-1 && (theta_P1(ii,1)<theta_P2(ii,1)) && gamma1WI==0
     th = 0;
     YP2WI_time(ii,1) = YP1WI_time(ii,1);
     XP2WI_time(ii,1) = XP1WI_time(ii,1);
  end
  
  if (sign(YP1WI_time(ii,1)- YC1WI(ii,1)) == sign(YP2WI_time(ii,1)- YC1WI(ii,1))) && f1WI==-1 && (theta_P1(ii,1)<theta_P2(ii,1)) && gamma2WI==0
     th = 0;
     YP1WI_time(ii,1) = YP2WI_time(ii,1);
     XP1WI_time(ii,1) = XP2WI_time(ii,1);
  end
  
  if (sign(YP1WI_time(ii,1)- YC1WI(ii,1)) ~= sign(YP2WI_time(ii,1)- YC1WI(ii,1))) && f1WI==-1
      theta_P2(ii,1) = theta_P2(ii,1) -2*pi;
      th = theta_P2(ii,1):pi/1000:theta_P1(ii,1); 
  end
      
  
  x = XC1WI(ii,1);
  y = YC1WI(ii,1);
  r = R1WI;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj2 = plot(xunit, yunit,'color',[1 0 0],'LineWidth',1,'LineStyle','-');
  Cj2.Color=[MC,0.5];
  hold on
  
  
% Lmt3   
  Li = line([XP2WI_time(ii,1) XI_time(ii,1)],[YP2WI_time(ii,1) YI_time(ii,1)],'color',[1 0 0],'LineWidth',1,'LineStyle','-');
  Li.Color=[MC,0.5];
  hold on  
  
  
else
% Lmt   
  Li = line([XW_time(ii,1) XI_time(ii,1)],[YW_time(ii,1) YI_time(ii,1)],'color',[1 0 0],'LineWidth',1,'LineStyle','-');
  Li.Color=[MC,0.5];
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


  dis = MaxY-MinY;
% xlim([MinX MaxX])
  ylim([MinY-0.05*dis MaxY+0.05*dis])
 

txt_title = 'Extensor Digitorum Communis 4';
% title(txt_title)
% hold off



