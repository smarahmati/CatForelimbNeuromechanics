% clear
% clc



NineMuscleMorphParaInitial = load('NineMuscleMorphParaInitial.mat');
NineMuscleMorphParaInitial = NineMuscleMorphParaInitial.NineMuscleMorphParaInitial;

G = 3; % Muscle Group Number (G = 1 to 9)
x0(1) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.a1;
x0(2) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.a2;
x0(3) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.phi1;
x0(4) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.phi2;
x0(5) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.R1;
x0(6) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.R2;

if G==1
MC = cmap(1,:);
elseif G==2 
MC = cmap(2,:);
elseif G==3
MC = cmap(3,:);
elseif G==4 
MC = cmap(4,:);
elseif G==5
MC = cmap(5,:);
elseif G==6
MC = cmap(6,:);
elseif G==7
MC = cmap(7,:);
elseif G==8
MC = cmap(8,:);
elseif G==9
MC = cmap(9,:);
end

xopt = x0;

MotionData = load('MotionData.mat');
MotionData = MotionData.MotionData;

% 40 Muscles
MuscleLengthVelocityMomentArm = load('MuscleLengthVelocityMomentArm.mat');
MuscleLengthVelocityMomentArm = MuscleLengthVelocityMomentArm.MuscleLengthVelocityMomentArm;

% 40 Muscles
MaxMuscleForceMoment = load('MaxMuscleForceMoment.mat');
MaxMuscleForceMoment = MaxMuscleForceMoment.MaxMuscleForceMoment;

% 9 Muscles
MuscleMP_9Groups = load('MuscleMP_9Groups.mat');
MuscleMP_9Groups = MuscleMP_9Groups.MuscleMP_9Groups;
Data_MuscleMP_9Groups = table2array(MuscleMP_9Groups(1:end, 2:end));

% Cluster Function
MuscleCluster = load('MuscleCluster.mat');
MuscleCluster = MuscleCluster.MuscleCluster;


%% Motion Data

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



%% X & Y min and max values
MinX = min(min([SCP(:,1), SHL(:,1), ELB(:,1), WRT(:,1), MCP(:,1), FT(:,1)]));
MaxX = max(max([SCP(:,1), SHL(:,1), ELB(:,1), WRT(:,1), MCP(:,1), FT(:,1)]));

MinY = min(min([SCP(:,2), SHL(:,2), ELB(:,2), WRT(:,2), MCP(:,2), FT(:,2)]));
MaxY = max(max([SCP(:,2), SHL(:,2), ELB(:,2), WRT(:,2), MCP(:,2), FT(:,2)]));






%% Computation of muscle length by two approaches

% Biceps Brachii 
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
handrule = -1; % it is 1 for right hand rule, and -1 for left hand rule


a1 = xopt(1);
a2 = xopt(2);
phi1 = xopt(3);
phi2 = xopt(4);
R1 = xopt(5);
R2 = xopt(6);
f1 = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.f1;
f2 = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.f2;
n1 = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.n1;
n2 = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.n2;


% To avoid attachment point inside joint radius
if a1<R1 
a1 = R1;
end

if a2<R2
a2 = R2;
end



% if a1<R1 
% msg = 'Error occurred: a1<R1';
% error(msg)
% end
% 
% if a2<R2
% msg = 'Error occurred: a2<R2';
% error(msg)
% end



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






%% Computation of muscle force and moment
% Biceps Brachii 
% Muscle Group Number (G = 1 to 9)

LMT = MuscleLength_time_JA;
VMT = MuscleVelocity_time_JA;


% activation
a = repmat(1, size(LMT,1),1);

% Muscle mechanical parameters 
LF0 = Data_MuscleMP_9Groups(G,8);         % Optimal fascicle length (mm)
Vmax_LF0 = -Data_MuscleMP_9Groups(G,14);  % Vamx/LF0 (1/s) is negative for current formulation
SO = Data_MuscleMP_9Groups(G,11);         % Percentage of sl-twitch fibres
if SO <= 1, SO = 1; end 
PCSA = Data_MuscleMP_9Groups(G,6);        % Physiological cross sectional area (cm^2)
PA = Data_MuscleMP_9Groups(G,7);          % Pennation angle (deg)
LF0 = LF0/1000;                           % mm to m
Vmax = Vmax_LF0*LF0;                      % Maximum velocity (m/s)
LT0 = 0.5*LF0;                            % we do not kn its value for n
TM = 2.3;                                 % Specific tensions of cat muscles (kg/cm^2)
g = 9.806;                                % Gravity acceleration (m/s^2)
FM_max = TM*PCSA*g;                       % Maximum muscle force (Newton)
aV_FMmax = 0.00915*SO-0.00467;            % aV/Fm_max = 0.00915· S − 0.00467 (normalized by Fm_max)
aV = aV_FMmax*FM_max;                     % aV = aV_FMmax*FM_max
Max_LMT = max(LMT);                       % Maximum musculotendon length

[LT, LF, VF, NLF, NVF, FPE, FCE_L, FCE_V, FM, FMT] = MTForce(a, time, LMT, VMT, LT0, LF0, aV, Vmax, FM_max, PA, Max_LMT);
MAMT = [MuscleMomentArmProximal_time, MuscleMomentArmDistal_time, MuscleMomentArmProximal_time*0];
% Moment Arm of Musculotendon
MAMT_Shoulder = MAMT(:,1);
MAMT_Elbow = MAMT(:,2);
MAMT_Wrist = MAMT(:,3);

% Moment of Musculotendon
MMT_Shoulder = MAMT(:,1).*FMT;
MMT_Elbow = MAMT(:,2).*FMT;
MMT_Wrist = MAMT(:,3).*FMT;





%%
% ---------------- Moment groups -------------------
% figure(1)

GroupsOfMoments_Shoulder = MaxMuscleForceMoment{1, 1}.Moment.Shoulder*0;  % just to set size of Group Of Muscle Moments
GroupsOfMoments_Elbow = MaxMuscleForceMoment{1, 1}.Moment.Shoulder*0;     % just to set size of Group Of Muscle Moments
GroupsOfMoments_Wrist = MaxMuscleForceMoment{1, 1}.Moment.Shoulder*0;     % just to set size of Group Of Muscle Moments

for i=1:size(MuscleCluster{G, 1},1)
Moment_Shoulder = MaxMuscleForceMoment{MuscleCluster{G, 1}(i), 1}.Moment.Shoulder;  
Moment_Elbow = MaxMuscleForceMoment{MuscleCluster{G, 1}(i), 1}.Moment.Elbow;  
Moment_Wrist = MaxMuscleForceMoment{MuscleCluster{G, 1}(i), 1}.Moment.Wrist;  

GroupsOfMoments_Shoulder = GroupsOfMoments_Shoulder + Moment_Shoulder;
GroupsOfMoments_Elbow = GroupsOfMoments_Elbow + Moment_Elbow;
GroupsOfMoments_Wrist = GroupsOfMoments_Wrist + Moment_Wrist;

minL(i,1) = min(MuscleLengthVelocityMomentArm{MuscleCluster{G, 1}(i), 1}.Length);  
maxL(i,1) = max(MuscleLengthVelocityMomentArm{MuscleCluster{G, 1}(i), 1}.Length);  
end

Ave_minL = mean(minL);
Ave_maxL = mean(maxL);


% plot(GroupsOfMoments_Elbow,'-k','LineWidth',2)
% hold on
% plot(MMT_Elbow,'--r','LineWidth',1)
% legend('Group of muscles','Combined muscle')
% xlabel('Time Step')
% ylabel('Elbow Moment (Nm)')
% title('Elbow Moment')
% ylim([min(GroupsOfMoments_Elbow)-3 max(GroupsOfMoments_Elbow)+3])



% percentage error
PE = 100*sum(abs(GroupsOfMoments_Elbow-MMT_Elbow))./sum(abs(GroupsOfMoments_Elbow));


% cost function
CF = sum(((GroupsOfMoments_Elbow-MMT_Elbow)./std(GroupsOfMoments_Elbow)).^2);


if ii==1

%% Write output
Name = 'Biceps Brachii';
me = [1, 1, 0];                                  % muscle effective joints


load('MuscleLengthVelocityMomentArm_9Groups.mat', 'MuscleLengthVelocityMomentArm_9Groups');
load('MusculoskeletalData_9Groups.mat', 'MusculoskeletalData_9Groups');
load('MaxMuscleForceMoment_9Groups.mat', 'MaxMuscleForceMoment_9Groups');


%----- Musculoskeletal data -----------------------------------------------
% MusculoskeletalData_9Groups.JointCenter.Scapula = SCP;
% MusculoskeletalData_9Groups.JointCenter.Shoulder = SHL;
% MusculoskeletalData_9Groups.JointCenter.Elbow = ELB;
% MusculoskeletalData_9Groups.JointCenter.Wrist = WRT;
% MusculoskeletalData_9Groups.JointCenter.Metacarpophalangeal  = MCP;
% MusculoskeletalData_9Groups.JointCenter.Foot  = FT;

MusculoskeletalData_9Groups.Muscles{G,1}.NameOfMuscle = Name;
MusculoskeletalData_9Groups.Muscles{G,1}.Origin = [XO_time YO_time];
MusculoskeletalData_9Groups.Muscles{G,1}.Insertion = [XI_time YI_time];
MusculoskeletalData_9Groups.Muscles{G,1}.EffectiveJoints = me;
%-------- Muscle Geometric Parameters -------------------------------------
MGP = table({Name}, a1, a2, phi1, phi2, R1, R2, f1, f2, n1, n2);
MusculoskeletalData_9Groups.Muscles{G,1}.MuscleGeometricParameters = MGP;



%----- Muscle name, length and moment arm ---------------------------------
MuscleLengthVelocityMomentArm_9Groups{G,1}.NameOfMuscle = Name;
MuscleLengthVelocityMomentArm_9Groups{G,1}.Length = LMT;
MuscleLengthVelocityMomentArm_9Groups{G,1}.Velocity = VMT;
MuscleLengthVelocityMomentArm_9Groups{G,1}.MomentArm = MAMT;
MuscleLengthVelocityMomentArm_9Groups{G,1}.EffectiveJoints = me;



%----- Muscle name, fascicle, tendon length, force and moment -------------
MaxMuscleForceMoment_9Groups{G,1}.NameOfMuscle = Name;
MaxMuscleForceMoment_9Groups{G,1}.Length.Fascicle = LF;
MaxMuscleForceMoment_9Groups{G,1}.Length.OptimalFascicle = LF0;
MaxMuscleForceMoment_9Groups{G,1}.Length.Tendon = LT;
MaxMuscleForceMoment_9Groups{G,1}.Force.FCE_L = FCE_L; % It has been already normalized
MaxMuscleForceMoment_9Groups{G,1}.Force.FCE_V = FCE_V; % It has been already normalized
MaxMuscleForceMoment_9Groups{G,1}.Force.FM = FM;
MaxMuscleForceMoment_9Groups{G,1}.Force.FMT = FMT;
MaxMuscleForceMoment_9Groups{G,1}.Force.MaximumIsometricForce = FM_max;
MaxMuscleForceMoment_9Groups{G,1}.Moment.Shoulder = MMT_Shoulder;
MaxMuscleForceMoment_9Groups{G,1}.Moment.Elbow = MMT_Elbow;
MaxMuscleForceMoment_9Groups{G,1}.Moment.Wrist = MMT_Wrist;


save('MuscleLengthVelocityMomentArm_9Groups.mat', 'MuscleLengthVelocityMomentArm_9Groups');
save('MusculoskeletalData_9Groups.mat', 'MusculoskeletalData_9Groups');
save('MaxMuscleForceMoment_9Groups.mat', 'MaxMuscleForceMoment_9Groups');





%{
%% Combined Muscle vs group of muscles
i = 1;
h = figure(3);  
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
  line([XO_time(i,1) XP1_time(i,1)],[YO_time(i,1) YP1_time(i,1)],'color',MC,'LineWidth',4);
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
  Cj3 = plot(xunit, yunit,'color',MC,'LineWidth',4);
  hold on
  
% Lmt3   
  line([XP2_time(i,1) XP3_time(i,1)],[YP2_time(i,1) YP3_time(i,1)],'color',MC,'LineWidth',4);
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
  Cj4 = plot(xunit, yunit,'color',MC,'LineWidth',4);
  hold on
  
% Lmt5   
  line([XP4_time(i,1) XI_time(i,1)],[YP4_time(i,1) YI_time(i,1)],'color',MC,'LineWidth',4);
  hold on  
  
  
elseif (theta1_time(i,1)<0 && theta2_time(i,1)>=0) %-----------------------------------------------
% Lmt1    
  line([XO_time(i,1) XP4_time(i,1)],[YO_time(i,1) YP4_time(i,1)],'color',MC,'LineWidth',4);
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
  Cj3 = plot(xunit, yunit,'color',MC,'LineWidth',4);
  hold on    
  
  Lmt2_2(i,1) = R2*(th(end)-th(1));
  
% Lmt3    
  line([XP5_time(i,1) XI_time(i,1)],[YP5_time(i,1) YI_time(i,1)],'color',MC,'LineWidth',4);
  hold on 
  
  Lmt3_2(i,1) = sqrt((XI_time(i,1)-XP5_time(i,1))^2+(YI_time(i,1)-YP5_time(i,1))^2);  

  Lmt_2(i,1) = Lmt1_2(i,1) + Lmt2_2(i,1) + Lmt3_2(i,1);

elseif (theta1_time(i,1)>=0 && theta2_time(i,1)<0) %-----------------------------------------------
% Lmt1    
  line([XO_time(i,1) XP1_time(i,1)],[YO_time(i,1) YP1_time(i,1)],'color',MC,'LineWidth',4);
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
  Cj3 = plot(xunit, yunit,'color',MC,'LineWidth',4);
  hold on    

% Lmt3    
  line([XP3_time(i,1) XI_time(i,1)],[YP3_time(i,1) YI_time(i,1)],'color',MC,'LineWidth',4);
  hold on 
  
else %----------------------------------------------------------------------------------------
% Lmt   
  line([XO_time(i,1) XI_time(i,1)],[YO_time(i,1) YI_time(i,1)],'color',MC,'LineWidth',4);
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


txt_title = Name;
title(txt_title)
hold off


% L1 = findobj(h,'type','line');
% copyobj(L1,findobj(hgload('AllMuscleGroups.fig'),'type','axes'));
% close(h)

%}
end




%% Simulation Combined Muscle

h = figure(3);
    
ms = 20; % Marker Size      
% Joint Centers
% s1 = scatter(SCP(ii,1),SCP(ii,2),ms,'MarkerFaceColor',[0.5, 0.5, 0],'MarkerEdgeColor',[0.5, 0.5, 0]);
% hold on
% alpha(s1,0.7)

s2 = scatter(SHL(ii,1),SHL(ii,2),ms,'MarkerFaceColor',[1, 0, 0],'MarkerEdgeColor',[1, 0, 0]);
hold on
alpha(s2,0.7)

s3 = scatter(ELB(ii,1),ELB(ii,2),ms,'MarkerFaceColor',[0, 1, 0],'MarkerEdgeColor',[0, 1, 0]);
hold on
alpha(s3,0.7)

s4 = scatter(WRT(ii,1),WRT(ii,2),ms,'MarkerFaceColor',[0, 0, 1],'MarkerEdgeColor',[0, 0, 1]);
hold on
alpha(s4,0.7)

% s5 = scatter(MCP(ii,1),MCP(ii,2),ms,'MarkerFaceColor',[0.5, 0, 0.5],'MarkerEdgeColor',[0.5, 0, 0.5]);
% hold on
% alpha(s5,0.7)
% 
% s6 = scatter(FT(ii,1),FT(ii,2),ms,'MarkerFaceColor',[0, 0.5, 0.5],'MarkerEdgeColor',[0, 0.5, 0.5]);
% hold on
% alpha(s6,0.7)


% Attachment Points
ATO = scatter(XO_time(ii,1), YO_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP1 = scatter(XP1_time(ii,1), YP1_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP2 = scatter(XP2_time(ii,1), YP2_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP3 = scatter(XP3_time(ii,1), YP3_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP4 = scatter(XP4_time(ii,1), YP4_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATP5 = scatter(XP5_time(ii,1), YP5_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
ATI = scatter(XI_time(ii,1), YI_time(ii,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
 
 %% Line Model
 
 
 %------------------  left fore limbs -----------------------------------
 line([SCP(ii,1) SHL(ii,1)],[SCP(ii,2) SHL(ii,2)],'color',[0 0 0],'LineWidth',1.5)
 hold on
 line([SHL(ii,1) ELB(ii,1)],[SHL(ii,2) ELB(ii,2)],'color',[0 0 0],'LineWidth',1.5)
 hold on
 line([ELB(ii,1) WRT(ii,1)],[ELB(ii,2) WRT(ii,2)],'color',[0 0 0],'LineWidth',1.5)
 hold on
 line([WRT(ii,1) MCP(ii,1)],[WRT(ii,2) MCP(ii,2)],'color',[0 0 0],'LineWidth',1.5)
 hold on
 % line([MCP(ii,1) FT(ii,1)],[MCP(ii,2) FT(ii,2)],'color',[0 0 0],'LineWidth',1.5)
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

x = XC2(ii,1);
y = YC2(ii,1);
r = R2;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
% Cj2 = plot(xunit, yunit,'color','cyan');


%% Attachment points

if (theta1_time(ii,1)>=0 && theta2_time(ii,1)>=0) %-----------------------------------------------
% Lmt1    
  line([XO_time(ii,1) XP1_time(ii,1)],[YO_time(ii,1) YP1_time(ii,1)],'color',MC,'LineWidth',3);
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
  
  if (theta_P2(ii,1)<theta_P1(ii,1)) && f1==1
    th = 0;
    YP2_time(ii,1) = YP1_time(ii,1);
    XP2_time(ii,1) = XP1_time(ii,1);
  end
  
  if (sign(YP1_time(ii,1)- YC1(ii,1)) == sign(YP2_time(ii,1)- YC1(ii,1))) && f1==-1 && (theta_P1(ii,1)>=theta_P2(ii,1))
      th = theta_P2(ii,1):pi/1000:theta_P1(ii,1); 
  end
  
  if (sign(YP1_time(ii,1)- YC1(ii,1)) == sign(YP2_time(ii,1)- YC1(ii,1))) && f1==-1 && (theta_P1(ii,1)<theta_P2(ii,1))
     th = 0;
     YP2_time(ii,1) = YP1_time(ii,1);
     XP2_time(ii,1) = XP1_time(ii,1);
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
  Cj3 = plot(xunit, yunit,'color',MC,'LineWidth',3);
  hold on
  
% Lmt3   
  line([XP2_time(ii,1) XP3_time(ii,1)],[YP2_time(ii,1) YP3_time(ii,1)],'color',MC,'LineWidth',3);
  hold on
  
% Lmt4     
  u = [XP3_time(ii,1), YP3_time(ii,1)]-[XC2(ii,1), YC2(ii,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P3(ii,1) = real(acos(CosTheta));
  
  u = [XP4_time(ii,1), YP4_time(ii,1)]-[XC2(ii,1), YC2(ii,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P4(ii,1) = real(acos(CosTheta));
  
  if  YP3_time(ii,1)< YC2(ii,1); theta_P3(ii,1) = 2*pi-theta_P3(ii,1); end
  if  YP4_time(ii,1)< YC2(ii,1); theta_P4(ii,1) = 2*pi-theta_P4(ii,1); end
  

  if (theta_P4(ii,1)>=theta_P3(ii,1)) && f2==1
    th = theta_P3(ii,1):pi/1000:theta_P4(ii,1);
  end
  
  if (theta_P4(ii,1)<theta_P3(ii,1)) && f2==1
    th = 0;
    YP3_time(ii,1) = YP4_time(ii,1);
    XP3_time(ii,1) = XP4_time(ii,1);
  end
  
  
  if (sign(YP3_time(ii,1)- YC2(ii,1)) == sign(YP4_time(ii,1)- YC2(ii,1))) && f2==-1 && (theta_P3(ii,1)>=theta_P4(ii,1))
      th =  theta_P4(ii,1):pi/1000:theta_P3(ii,1); 
  end
  
  if (sign(YP3_time(ii,1)- YC2(ii,1)) == sign(YP4_time(ii,1)- YC2(ii,1))) && f2==-1 && (theta_P3(ii,1)<theta_P4(ii,1))
     th =0;
     YP3_time(ii,1) = YP4_time(ii,1);
     XP3_time(ii,1) = XP4_time(ii,1);
  end

  if (sign(YP3_time(ii,1)- YC2(ii,1)) ~= sign(YP4_time(ii,1)- YC2(ii,1))) && f2==-1
      theta_P4(ii,1) = theta_P4(ii,1) -2*pi;
      th = theta_P4(ii,1):pi/1000:theta_P3(ii,1); 
  end


   
  x = XC2(ii,1);
  y = YC2(ii,1);
  r = R2;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj4 = plot(xunit, yunit,'color',MC,'LineWidth',3);
  hold on
  
% Lmt5   
  line([XP4_time(ii,1) XI_time(ii,1)],[YP4_time(ii,1) YI_time(ii,1)],'color',MC,'LineWidth',3);
  hold on  
  
  
elseif (theta1_time(ii,1)<0 && theta2_time(ii,1)>=0) %-----------------------------------------------
% Lmt1    
  line([XO_time(ii,1) XP4_time(ii,1)],[YO_time(ii,1) YP4_time(ii,1)],'color',MC,'LineWidth',3);
  hold on  
 
  Lmt1_2(ii,1) = sqrt((XO_time(ii,1)-XP4_time(ii,1))^2+(YO_time(ii,1)-YP4_time(ii,1))^2);   
  
  
% Lmt2   
  u = [XP4_time(ii,1), YP4_time(ii,1)]-[XC2(ii,1), YC2(ii,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P4(ii,1) = real(acos(CosTheta));
  
  u = [XP5_time(ii,1), YP5_time(ii,1)]-[XC2(ii,1), YC2(ii,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P5(ii,1) = real(acos(CosTheta));
  
  if  YP4_time(ii,1)< YC2(ii,1); theta_P4(ii,1) = 2*pi-theta_P4(ii,1); end
  if  YP5_time(ii,1)< YC2(ii,1); theta_P5(ii,1) = 2*pi-theta_P5(ii,1); end
  
  if (theta_P5(ii,1)>=theta_P4(ii,1)) && f2==1
    th = theta_P4(ii,1):pi/1000:theta_P5(ii,1);
  end
  
  if (theta_P5(ii,1)<theta_P4(ii,1)) && f2==1
    th =0;
    YP4_time(ii,1) = YP5_time(ii,1);
    XP4_time(ii,1) = XP5_time(ii,1);
  end
  
  
  if (sign(YP4_time(ii,1)- YC2(ii,1)) == sign(YP5_time(ii,1)- YC2(ii,1))) && f2==-1 && (theta_P4(ii,1)>=theta_P5(ii,1))
      th = theta_P5(ii,1):pi/1000:theta_P4(ii,1); 
  end
  
  if (sign(YP4_time(ii,1)- YC2(ii,1)) == sign(YP5_time(ii,1)- YC2(ii,1))) && f2==-1 && (theta_P4(ii,1)<theta_P5(ii,1))
     th = 0;
     YP4_time(ii,1) = YP5_time(ii,1);
     XP4_time(ii,1) = XP5_time(ii,1);
  end
  
  if (sign(YP4_time(ii,1)- YC2(ii,1)) ~= sign(YP5_time(ii,1)- YC2(ii,1))) && f2==-1
      theta_P5(ii,1) = theta_P5(ii,1) - 2*pi;
      th = theta_P5(ii,1):pi/1000:theta_P4(ii,1); 
  end
  
  x = XC2(ii,1);
  y = YC2(ii,1);
  r = R2;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj3 = plot(xunit, yunit,'color',MC,'LineWidth',3);
  hold on    
  
  Lmt2_2(ii,1) = R2*(th(end)-th(1));
  
% Lmt3    
  line([XP5_time(ii,1) XI_time(ii,1)],[YP5_time(ii,1) YI_time(ii,1)],'color',MC,'LineWidth',3);
  hold on 
  
  Lmt3_2(ii,1) = sqrt((XI_time(ii,1)-XP5_time(ii,1))^2+(YI_time(ii,1)-YP5_time(ii,1))^2);  

  Lmt_2(ii,1) = Lmt1_2(ii,1) + Lmt2_2(ii,1) + Lmt3_2(ii,1);

elseif (theta1_time(ii,1)>=0 && theta2_time(ii,1)<0) %-----------------------------------------------
% Lmt1    
  line([XO_time(ii,1) XP1_time(ii,1)],[YO_time(ii,1) YP1_time(ii,1)],'color',MC,'LineWidth',3);
  hold on  
    
% Lmt2   
  u = [XP1_time(ii,1), YP1_time(ii,1)]-[XC1(ii,1), YC1(ii,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P1(ii,1) = real(acos(CosTheta));
  
  u = [XP3_time(ii,1), YP3_time(ii,1)]-[XC1(ii,1), YC1(ii,1)];
  v = [1, 0];
  CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
  theta_P3(ii,1) = real(acos(CosTheta));
  
  if  YP1_time(ii,1)< YC1(ii,1); theta_P1(ii,1) = 2*pi-theta_P1(ii,1); end
  if  YP3_time(ii,1)< YC1(ii,1); theta_P3(ii,1) = 2*pi-theta_P3(ii,1); end
  
  if (theta_P3(ii,1)>=theta_P1(ii,1)) && f1==1
    th = theta_P1(ii,1):pi/1000:theta_P3(ii,1);
  end
  
  if (theta_P3(ii,1)<theta_P1(ii,1)) && f1==1
    th =0;
    YP3_time(ii,1) = YP1_time(ii,1);
    XP3_time(ii,1) = XP1_time(ii,1);
  end  
  
  if (sign(YP1_time(ii,1)- YC1(ii,1)) == sign(YP3_time(ii,1)- YC1(ii,1))) && f1==-1 && (theta_P1(ii,1)>=theta_P3(ii,1))
      th = theta_P3(ii,1):pi/1000:theta_P1(ii,1); 
  end
  
  if (sign(YP1_time(ii,1)- YC1(ii,1)) == sign(YP3_time(ii,1)- YC1(ii,1))) && f1==-1 && (theta_P1(ii,1)<theta_P3(ii,1))
     th =0;
     YP3_time(ii,1) = YP1_time(ii,1);
     XP3_time(ii,1) = XP1_time(ii,1);
  end
  
  if (sign(YP1_time(ii,1)- YC1(ii,1)) ~= sign(YP3_time(ii,1)- YC1(ii,1))) && f1==-1
      theta_P3(ii,1) = theta_P3(ii,1) -2*pi;
      th = theta_P3(ii,1):pi/1000:theta_P1(ii,1); 
  end
  
  
  x = XC1(ii,1);
  y = YC1(ii,1);
  r = R1;
  xunit = r * cos(th) + x;
  yunit = r * sin(th) + y;
  Cj3 = plot(xunit, yunit,'color',MC,'LineWidth',3);
  hold on    

% Lmt3    
  line([XP3_time(ii,1) XI_time(ii,1)],[YP3_time(ii,1) YI_time(ii,1)],'color',MC,'LineWidth',3);
  hold on 
  
else %----------------------------------------------------------------------------------------
% Lmt   
  line([XO_time(ii,1) XI_time(ii,1)],[YO_time(ii,1) YI_time(ii,1)],'color',MC,'LineWidth',3);
  hold on   
  
  Lmt_2(ii,1) = sqrt((XO_time(ii,1)-XI_time(ii,1))^2+(YO_time(ii,1)-YI_time(ii,1))^2);   
  
  
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

 
 

title('Musculoskeletal Simulation')





