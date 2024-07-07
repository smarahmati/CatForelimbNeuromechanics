clear
clc



NineMuscleMorphParaInitial = load('NineMuscleMorphParaInitial.mat');
NineMuscleMorphParaInitial = NineMuscleMorphParaInitial.NineMuscleMorphParaInitial;

G = 3; % Muscle Group Number (G = 1 to 9)
x0(1) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.a1;
x0(2) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.a2;
x0(3) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.phi1;
x0(4) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.phi2;
x0(5) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.R1;
x0(6) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.R2;



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
plot(GroupsOfMoments_Shoulder,'-k','LineWidth',2)
hold on
plot(MMT_Shoulder,'--r','LineWidth',1)
legend('Group of muscles','Combined muscle')
xlabel('Time Step')
ylabel('Shoulder Moment (Nm)')
title('Shoulder Moment')
% ylim([min(GroupsOfMoments_Elbow)-3 max(GroupsOfMoments_Elbow)+3])


subplot(2,1,2)
plot(GroupsOfMoments_Elbow,'-k','LineWidth',2)
hold on
plot(MMT_Elbow,'--r','LineWidth',1)
legend('Group of muscles','Combined muscle')
xlabel('Time Step')
ylabel('Elbow Moment (Nm)')
title('Elbow Moment')
% ylim([min(GroupsOfMoments_Elbow)-3 max(GroupsOfMoments_Elbow)+3])




% percentage error
PE = 100*sum(abs(GroupsOfMoments_Elbow-MMT_Elbow))./sum(abs(GroupsOfMoments_Elbow));


% cost function
CF = sum(((GroupsOfMoments_Elbow-MMT_Elbow)./std(GroupsOfMoments_Elbow)).^2);




%% plot (3in3 figs)

Name = 'Biceps Brachii ';

figure(2)
subplot(3,3,1)
plot(time, LMT,'-k','LineWidth',1)
hold on
grid on
xlabel('Time(s)');
ylabel('LMT (m)');
title(['Length of Musculotendon ', '(', Name, ')'])
set(gca,'XTick',[])
hold on
y_min = min(LMT);
y_max = max(LMT);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');
legend([l1, l2], 'Swing', 'Stance');     


subplot(3,3,4)
plot(time, LF,'--b','LineWidth',1)
hold on
grid on
xlabel('Time(s)');
ylabel('LF (m)');
title(['Length of Muscle Fiber'])
set(gca,'XTick',[])
y_min = min(LF);
y_max = max(LF);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');


subplot(3,3,7)
plot(time, VF,'-.r','LineWidth',1)
hold on
grid on
xlabel('Time(s)');
ylabel('VF (m/s)')
title(['Velocity of Muscle Fiber '])
% set(gca,'XTick',[])
hold on
y_min = min(VF);
y_max = max(VF);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');



subplot(3,3,2)
plot(NLF, FPE,'.m','LineWidth',1)
hold on
grid on
xlabel('Normalized length of muscle (LF/LF0)');
ylabel('Normalized Force (FPE/FMmax)');
title(['Muscle parallel elastic element ', '(', Name, ')'])
%set(gca,'XTick',[])
% axis equal

subplot(3,3,5)
plot(NLF, FCE_L,'.b','LineWidth',1)
hold on
grid on
xlabel('Normalized length of muscle (LF/LF0)');
ylabel('Normalized Force (FCEl/FMmax)');
title(['Muscle force-length relationship'])
%set(gca,'XTick',[])
% axis equal

subplot(3,3,8)
plot(NVF, FCE_V,'.r','LineWidth',1)
hold on
grid on
xlabel('Normalized velocity of muscle (VF/VMmax)');
ylabel('Normalized Force (FCEv/FMmax)');
title(['Muscle force-velocity relationship'])
%set(gca,'XTick',[])
hold on
% axis equal



if (MAMT(1,1)~=0) && (MAMT(1,2)==0) && (MAMT(1,3)==0)
subplot(3,3,3)
p1 = plot(time, MAMT(:,1),'-m','LineWidth',1);
grid on
xlabel('time (s)');
ylabel('Moment arm of MT (m)');
title(['Shoulder', ' (',Name,')'])
set(gca,'XTick',[])
y_min = min(MAMT(:,1));
y_max = max(MAMT(:,1));
if y_min == y_max
  y_min = y_min-0.1*y_min;
  y_max = y_max-0.1*y_max;
end
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');




subplot(3,3,6)
plot(time, FMT,'--b','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('MT force (N)');
% title(Name)
set(gca,'XTick',[])
y_min = min(FMT);
y_max = max(FMT);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');

subplot(3,3,9)
plot(time, MMT_Shoulder,'-k','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('MT moment(Nm)');
% title(Name)
% set(gca,'XTick',[])
y_min = min(MMT_Shoulder);
y_max = max(MMT_Shoulder);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');



elseif (MAMT(1,1)==0) && (MAMT(1,2)~=0) && (MAMT(1,3)==0)
subplot(3,3,3)
plot(time, MAMT(:,2),'-m','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('Moment arm of MT (m)');
title(['Elbow', ' (',Name,')'])
set(gca,'XTick',[])
y_min = min(MAMT(:,2));
y_max = max(MAMT(:,2));
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');

subplot(3,3,6)
plot(time, FMT,'--b','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('MT force (N)');
% title(Name)
set(gca,'XTick',[])
y_min = min(FMT);
y_max = max(FMT);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');

subplot(3,3,9)
plot(time, MMT_Elbow,'-k','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('MT moment(Nm)');
% title(Name)
% set(gca,'XTick',[])
y_min = min(MMT_Elbow);
y_max = max(MMT_Elbow);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');



elseif (MAMT(1,1)==0) && (MAMT(1,2)==0) && (MAMT(1,3)~=0)
subplot(3,3,3)
plot(time, MAMT(:,3),'-m','LineWidth',1);
grid on
xlabel('time (s)');
ylabel('Moment arm of MT (m)');
title(['Wrist', ' (',Name,')'])
set(gca,'XTick',[])
y_min = min(MAMT(:,3));
y_max = max(MAMT(:,3));
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');


subplot(3,3,6)
plot(time, FMT,'--b','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('MT force (N)');
% title(Name)
set(gca,'XTick',[])
y_min = min(FMT);
y_max = max(FMT);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');

subplot(3,3,9)
plot(time, MMT_Wrist,'-k','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('MT moment(Nm)');
% title(Name)
% set(gca,'XTick',[])
y_min = min(MMT_Wrist);
y_max = max(MMT_Wrist);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');



elseif (MAMT(1,1)~=0) && (MAMT(1,2)~=0) && (MAMT(1,3)==0)
subplot(3,3,3)
p1 = plot(time, MAMT(:,1),'-m','LineWidth',1);
hold on
p2 = plot(time, MAMT(:,2),'-.m','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('Moment arm of MT (m)');
title(['Shoulder & Elbow', ' (',Name,')'])
set(gca,'XTick',[])
y_min = min([MAMT(:,1); MAMT(:,2)]);
y_max = max([MAMT(:,1); MAMT(:,2)]);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');
legend([p1, p2], 'Shoulder', 'Elbow');     


subplot(3,3,6)
plot(time, FMT,'--b','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('MT force (N)');
% title(Name)
set(gca,'XTick',[])
y_min = min(FMT);
y_max = max(FMT);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');

subplot(3,3,9)
p1 = plot(time, MMT_Shoulder,'-k','LineWidth',1);
hold on
p2 = plot(time, MMT_Elbow,'-.k','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('MT moment(Nm)');
% title(Name)
% set(gca,'XTick',[])
y_min = min([MMT_Shoulder; MMT_Elbow]);
y_max = max([MMT_Shoulder; MMT_Elbow]);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');
legend([p1, p2], 'Shoulder', 'Elbow');   




elseif (MAMT(1,1)==0) && (MAMT(1,2)~=0) && (MAMT(1,3)~=0)
subplot(3,3,3)
p1 = plot(time, MAMT(:,2),'-m','LineWidth',1);
hold on
p2 = plot(time, MAMT(:,3),'-.m','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('Moment arm of MT (m)');
title(['Elbow & Wrist', ' (',Name,')'])
set(gca,'XTick',[])
y_min = min([MAMT(:,2); MAMT(:,3)]);
y_max = max([MAMT(:,2); MAMT(:,3)]);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');
legend([p1, p2], 'Elbow', 'Wrist');     


subplot(3,3,6)
plot(time, FMT,'--b','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('MT force (N)');
% title(Name)
set(gca,'XTick',[])
y_min = min(FMT);
y_max = max(FMT);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');

subplot(3,3,9)
p1 = plot(time, MMT_Elbow,'-k','LineWidth',1);
hold on
p2 = plot(time, MMT_Wrist,'-.k','LineWidth',1);
hold on
grid on
xlabel('time (s)');
ylabel('MT moment(Nm)');
% title(Name)
% set(gca,'XTick',[])
y_min = min([MMT_Elbow; MMT_Wrist]);
y_max = max([MMT_Elbow; MMT_Wrist]);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','g');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');
legend([p1, p2], 'Elbow', 'Wrist');   

end






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
  line([XO_time(i,1) XP1_time(i,1)],[YO_time(i,1) YP1_time(i,1)],'color',[1 0 0],'LineWidth',4);
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
  Cj3 = plot(xunit, yunit,'color',[1 0 0],'LineWidth',4);
  hold on
  
% Lmt3   
  line([XP2_time(i,1) XP3_time(i,1)],[YP2_time(i,1) YP3_time(i,1)],'color',[1 0 0],'LineWidth',4);
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
  Cj4 = plot(xunit, yunit,'color',[1 0 0],'LineWidth',4);
  hold on
  
% Lmt5   
  line([XP4_time(i,1) XI_time(i,1)],[YP4_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',4);
  hold on  
  
  
elseif (theta1_time(i,1)<0 && theta2_time(i,1)>=0) %-----------------------------------------------
% Lmt1    
  line([XO_time(i,1) XP4_time(i,1)],[YO_time(i,1) YP4_time(i,1)],'color',[1 0 0],'LineWidth',4);
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
  Cj3 = plot(xunit, yunit,'color',[1 0 0],'LineWidth',4);
  hold on    
  
  Lmt2_2(i,1) = R2*(th(end)-th(1));
  
% Lmt3    
  line([XP5_time(i,1) XI_time(i,1)],[YP5_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',4);
  hold on 
  
  Lmt3_2(i,1) = sqrt((XI_time(i,1)-XP5_time(i,1))^2+(YI_time(i,1)-YP5_time(i,1))^2);  

  Lmt_2(i,1) = Lmt1_2(i,1) + Lmt2_2(i,1) + Lmt3_2(i,1);

elseif (theta1_time(i,1)>=0 && theta2_time(i,1)<0) %-----------------------------------------------
% Lmt1    
  line([XO_time(i,1) XP1_time(i,1)],[YO_time(i,1) YP1_time(i,1)],'color',[1 0 0],'LineWidth',4);
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
  Cj3 = plot(xunit, yunit,'color',[1 0 0],'LineWidth',4);
  hold on    

% Lmt3    
  line([XP3_time(i,1) XI_time(i,1)],[YP3_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',4);
  hold on 
  
else %----------------------------------------------------------------------------------------
% Lmt   
  line([XO_time(i,1) XI_time(i,1)],[YO_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',4);
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



L1 = findobj(h,'type','line');
copyobj(L1,findobj(hgload('AllMuscleGroups.fig'),'type','axes'));
close(h)



%% Simulation Combined Muscle


f=0;


h = figure(5);
pause(2);
for i= 1:size(SCP,1)
       
      
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
  Cj3 = plot(xunit, yunit,'color',[1 0 0],'LineWidth',1.5);
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
  Cj4 = plot(xunit, yunit,'color',[1 0 0],'LineWidth',1.5);
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
  Cj3 = plot(xunit, yunit,'color',[1 0 0],'LineWidth',1.5);
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
  Cj3 = plot(xunit, yunit,'color',[1 0 0],'LineWidth',1.5);
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

 
 

txt_title = Name;
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












