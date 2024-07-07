function [ CF ] = Cost_Function (xopt)


NineMuscleMorphParaInitial = load('NineMuscleMorphParaInitial.mat');
NineMuscleMorphParaInitial = NineMuscleMorphParaInitial.NineMuscleMorphParaInitial;

G = 5; % Muscle Group Number (G = 1 to 9)

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

% Elbow Flexor - Wrist Dorsiflexor(O->W) --------------------------------------------------------------------
alpha1OW = ELBa;
alpha1OW_dot = ELBa_dot;
y1OW = q2;
y2OW = q3;
XC1OW = ELB(:,1);
YC1OW = ELB(:,2);
handrule = -1; % it is 1 for right hand rule, and -1 for left hand rule


a1OW = xopt(1);
a2OW = xopt(2);
phi1OW = xopt(3);
phi2OW = xopt(4);
R1OW = xopt(7);
R2OW = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.R2OW;
f1OW = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.f1OW;
f2OW = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.f2OW;
n1OW = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.n1OW;
n2OW = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.n2OW;

% To avoid attachment point inside joint radius
if a1OW<R1OW 
a1OW = R1OW;
end

if a2OW<R1OW
a2OW = R1OW;
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



% Elbow Flexor - Wrist Dorsiflexor(W->I)  --------------------------------------------------------------------
alpha1WI = WRTa;
alpha1WI_dot = WRTa_dot;
y1WI = q3;
y2WI = q4;
XC1WI = WRT(:,1);
YC1WI = WRT(:,2);


a1WI = sqrt(Lfarm_Mo2D^2 + a2OW^2 - 2*Lfarm_Mo2D*a2OW*cos(phi2OW));
a2WI = xopt(5);
phi1WI = -asin(a2OW*sin(phi2OW)/a1WI);
phi2WI = xopt(6);
R1WI = xopt(8);
R2WI = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.R2WI;
f1WI = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.f1WI;
f2WI = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.f2WI;
n1WI = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.n1WI;
n2WI = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.n2WI;


% To avoid attachment point inside joint radius
if a1WI<R1WI 
a1WI = R1WI;
end

if a2WI<R1WI
a2WI = R1WI;
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





%% Computation of muscle force and moment
% Wrist Extensor
% Muscle Group Number (G = 1 to 9)

LMT = MuscleLengthOW_time_JA + MuscleLengthWI_time_JA;
VMT = MuscleVelocityOW_time_JA + MuscleVelocityWI_time_JA;


% activation
a = repmat(1, size(LMT,1),1);

% Muscle mechanical parameters 
LF0 = Data_MuscleMP_9Groups(G,8);         % Optimal fascicle length (mm)
Vmax_LF0 = -Data_MuscleMP_9Groups(G,14);  % Vamx/LF0 (1/s) is negative for current formulation
SO = Data_MuscleMP_9Groups(G,11);         % Percentage of slow-twitch fibres
if SO <= 1, SO = 1; , end 
PCSA = Data_MuscleMP_9Groups(G,6);        % Physiological cross sectional area (cm^2)
PA = Data_MuscleMP_9Groups(G,7);          % Pennation angle (deg)
LF0 = LF0/1000;                           % mm to m
Vmax = Vmax_LF0*LF0;                      % Maximum velocity (m/s)
LT0 = 0.5*LF0;                            % we do not know its value for now
TM = 2.3;                                 % Specific tensions of cat muscles (kg/cm^2)
g = 9.806;                                % Gravity acceleration (m/s^2)
FM_max = TM*PCSA*g;                       % Maximum muscle force (Newton)
aV_FMmax = 0.00915*SO-0.00467;            % aV/Fm_max = 0.00915· S − 0.00467 (normalized by Fm_max)
aV = aV_FMmax*FM_max;                     % aV = aV_FMmax*FM_max
Max_LMT = max(LMT);                       % Maximum musculotendon length

[LT, LF, VF, NLF, NVF, FPE, FCE_L, FCE_V, FM, FMT] = MTForce(a, time, LMT, VMT, LT0, LF0, aV, Vmax, FM_max, PA, Max_LMT);
MAMT = [MuscleMomentArmWI_time*0, MuscleMomentArmOW_time, MuscleMomentArmWI_time];
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
figure(1)

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

subplot(2,1,1)
plot(GroupsOfMoments_Elbow,'-k','LineWidth',2)
hold on
plot(MMT_Elbow,'--r','LineWidth',1)
legend('Group of muscles','Combined muscle')
xlabel('Time Step')
ylabel('Elbow Moment (Nm)')
title('Elbow Moment')
% ylim([min(GroupsOfMoments_Wrist)-3 max(GroupsOfMoments_Wrist)+3])


subplot(2,1,2)
plot(GroupsOfMoments_Wrist,'-k','LineWidth',2)
hold on
plot(MMT_Wrist,'--r','LineWidth',1)
legend('Group of muscles','Combined muscle')
xlabel('Time Step')
ylabel('Wrist Moment (Nm)')
title('Wrist Moment')
% ylim([min(GroupsOfMoments_Wrist)-3 max(GroupsOfMoments_Wrist)+3])




% percentage error
PE = 100*(sum(abs(GroupsOfMoments_Elbow-MMT_Elbow))./sum(abs(GroupsOfMoments_Elbow))+...
          sum(abs(GroupsOfMoments_Wrist-MMT_Wrist))./sum(abs(GroupsOfMoments_Wrist)))/2;


% cost function
CF = (sum(((GroupsOfMoments_Elbow-MMT_Elbow)./std(GroupsOfMoments_Elbow)).^2) + ...
      sum(((GroupsOfMoments_Wrist-MMT_Wrist)./std(GroupsOfMoments_Wrist)).^2))/2;


end



