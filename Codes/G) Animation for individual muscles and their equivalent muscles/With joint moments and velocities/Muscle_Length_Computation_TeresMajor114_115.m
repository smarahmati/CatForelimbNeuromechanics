% clear
% clc

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

cmn = 36;              % considered muscle number 

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

% Teres Major(114->115)
MN = 73;
alpha1 = SHLa;
alpha1_dot = SHLa_dot;
y1 = q1;
y2 = q2;
XC1 = SHL(:,1);
YC1 = SHL(:,2);
Sc_a1_r1 = Scale_Scap2D;
Sc_a2_r2 = Scale_Uarm2D;
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

% After scaling it is possible that the equal values of R1 and a1 or 
% R1 and a2 do not be equall anymore, the following commands make sure they
% are equal even after scaling
if a2_ALL(MN,1) == R1_ALL(MN,1)
   R1 = a2; 
elseif  a1_ALL(MN,1) == R1_ALL(MN,1)
   R1 = a1;
end


if a1<R1 
msg = 'Error occurred: a1<R1';
error(msg)
end

if a2<R1
msg = 'Error occurred: a2<R1';
error(msg)
end


for i=1:size(alpha1,1)
[MuscleLength, MuscleVelocity, MuscleMomentArm] = MuscleOneJoint_JointAngles(a1,a2,phi1,phi2,R1,f1,n1,alpha1(i,1),alpha1_dot(i,1),handrule);
MuscleLength_time_JA(i,1) =  MuscleLength;
MuscleVelocity_time_JA(i,1) =  MuscleVelocity;
MuscleMomentArm_time(i,1) = MuscleMomentArm;
end



for i=1:size(alpha1,1)
 [XO, YO, XI, YI, XP1, YP1, XP2, YP2, theta1, theta1_FK, alpha1, gamma1, gamma2, beta1, beta2, MuscleLength] = ...
  MuscleOneJoint_SegmentAngles(a1,a2,phi1,phi2,R1,f1,n1,y1(i,1),y2(i,1),XC1(i,1),YC1(i,1));

XO_time(i,1) = XO;
YO_time(i,1) = YO;

XI_time(i,1) = XI;
YI_time(i,1) = YI;

XP1_time(i,1) = XP1;
YP1_time(i,1) = YP1;

XP2_time(i,1) = XP2;
YP2_time(i,1) = YP2;


theta1_time(i,1) = theta1; % theta1 as the presented criterion

theta1_FK_time(i,1) = theta1_FK; % theta1 based on forwad kinematics


alpha1_time(i,1) = alpha1;
gamma1_time(i,1) = gamma1;
gamma2_time(i,1) = gamma2;
beta1_time(i,1) = beta1;
beta2_time(i,1) = beta2;

MuscleLength_time_SA(i,1) =  MuscleLength; % Worse -> we consider the following computation for muscle length based on forward kinematics 
end



%% Write information
mn = 'Teres Major';                                    % muscle name
ml = MuscleLength_time_JA;                             % muscle length
mv = MuscleVelocity_time_JA;                           % muscle Velocity
ma = [MuscleMomentArm_time, time*0, time*0];           % muscle momemet arm
me = [1, 0, 0];                                        % muscle effective joints

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
%pause(2);
       
      
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
% ATP1 = scatter(XP1_time(ii,1), YP1_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
% hold on
% ATP2 = scatter(XP2_time(ii,1), YP2_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
% hold on
% ATI = scatter(XI_time(ii,1), YI_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
% hold on
 
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
x = XC1(ii,1);
y = YC1(ii,1);
r = R1;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
% Cj1 = plot(xunit, yunit,'color','cyan');



%% Attachment points

if (theta1_time(ii,1)>=0)
% Lmt1    
  Li = line([XO_time(ii,1) XP1_time(ii,1)],[YO_time(ii,1) YP1_time(ii,1)],'color',[1 0 0],'LineWidth',1,'LineStyle','-');
  Li.Color=[MC,0.5];
  hold on

% Lmt2   
  u = [XP1_time(ii,1), YP1_time(ii,1)]-[XC1(ii,1), YC1(ii,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P1(ii,1) = real(acos(CosTheta));
  
  u = [XP2_time(ii,1), YP2_time(ii,1)]-[XC1(ii,1), YC1(ii,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P2(ii,1) = real(acos(CosTheta));
  
  if  YP1_time(ii,1)< YC1(ii,1); theta_P1(ii,1) = 2*pi-theta_P1(ii,1); end
  if  YP2_time(ii,1)< YC1(ii,1); theta_P2(ii,1) = 2*pi-theta_P2(ii,1); end
  
  if (theta_P2(ii,1)>=theta_P1(ii,1)) && f1==1
    th = theta_P1(ii,1):pi/1000:theta_P2(ii,1);
  end
  
  if (theta_P2(ii,1)<theta_P1(ii,1)) && f1==1 && gamma1==0
    th = 0;
    YP2_time(ii,1) = YP1_time(ii,1);
    XP2_time(ii,1) = XP1_time(ii,1);
  end  
  
  if (theta_P2(ii,1)<theta_P1(ii,1)) && f1==1 && gamma2==0
    th = 0;
    YP1_time(ii,1) = YP2_time(ii,1);
    XP1_time(ii,1) = XP2_time(ii,1);
  end  
  
  if (sign(YP1_time(ii,1)- YC1(ii,1)) == sign(YP2_time(ii,1)- YC1(ii,1))) && f1==-1 && (theta_P1(ii,1)>=theta_P2(ii,1))
      th = theta_P2(ii,1):pi/1000:theta_P1(ii,1); 
  end
  
  if (sign(YP1_time(ii,1)- YC1(ii,1)) == sign(YP2_time(ii,1)- YC1(ii,1))) && f1==-1 && (theta_P1(ii,1)<theta_P2(ii,1)) && gamma1==0
     th = 0;
     YP2_time(ii,1) = YP1_time(ii,1);
     XP2_time(ii,1) = XP1_time(ii,1);
  end
  
  if (sign(YP1_time(ii,1)- YC1(ii,1)) == sign(YP2_time(ii,1)- YC1(ii,1))) && f1==-1 && (theta_P1(ii,1)<theta_P2(ii,1)) && gamma2==0
     th = 0;
     YP1_time(ii,1) = YP2_time(ii,1);
     XP1_time(ii,1) = XP2_time(ii,1);
  end
  
  if (sign(YP1_time(ii,1)- YC1(ii,1)) ~= sign(YP2_time(ii,1)- YC1(ii,1))) && f1==-1
      theta_P2(ii,1) = theta_P2(ii,1) -2*pi;
      th = theta_P2(ii,1):pi/1000:theta_P1(ii,1); 
  end
      
  
  x = XC1(ii,1);
  y = YC1(ii,1);
  r = R1;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj2 = plot(xunit, yunit,'color',[1 0 0]);
  Cj2.Color=[MC,0.5];
  hold on
  
  
% Lmt3   
  Li = line([XP2_time(ii,1) XI_time(ii,1)],[YP2_time(ii,1) YI_time(ii,1)],'color',[1 0 0],'LineWidth',1,'LineStyle','-');
  Li.Color=[MC,0.5];
  hold on  
  
  
else
% Lmt   
  Li = line([XO_time(ii,1) XI_time(ii,1)],[YO_time(ii,1) YI_time(ii,1)],'color',[1 0 0],'LineWidth',1,'LineStyle','-');
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
 
 

txt_title = 'Teres Major';
% title(txt_title)
% hold off



