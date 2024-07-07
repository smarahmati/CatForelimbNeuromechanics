clear
clc



NineMuscleMorphParaInitial = load('NineMuscleMorphParaInitial.mat');
NineMuscleMorphParaInitial = NineMuscleMorphParaInitial.NineMuscleMorphParaInitial;

G = 1; % Muscle Group Number (G = 1 to 9)
x0(1) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.a1;
x0(2) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.a2;
x0(3) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.phi1;
x0(4) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.phi2;
x0(5) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.R1;


%----Different initial guess inside boundaries--------
%-------------- 1 ------------------------------------
% x0 = [0.0347057308920415,0.0103559715018728,-0.230266389709403,1.90740512008528,0.00725351384300108]
% xopt = [0.0343985979641218,0.00959806697337583,-0.270493577772424,1.71742117336004,0.00793720119258581]
% PE = 5.9027
% CF = 2.9752

%-------------- 2 ------------------------------------
% x0 = [0.0300153875690889,0.00842180930684077,-0.292211399587022,1.82235732212320,0.00479954555805423]
% xopt = [0.0382707774464613,0.00794756931553267,-0.372976861151188,2.26762142267820,0.00793719908435372]
% PE = 5.9027
% CF = 2.9752

%-------------- 3 ------------------------------------
% x0 = [0.0527335397632468,0.0177901194899987,0.00782551992411873,2.23429493198335,0.0166855891029358]
% xopt = [0.0564002012493833,0.00794548203380911,-0.277010006173251,2.39160180521177,0.00794485115987297]
% PE = 5.9080
% CF = 2.9756


xopt = [0.0564002012493833,0.00794548203380911,-0.277010006173251,2.39160180521177,0.00794485115987297];

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

% Shoulder Protractor
alpha1 = SHLa;
alpha1_dot = SHLa_dot;
y1 = q1;
y2 = q2;
XC1 = SHL(:,1);
YC1 = SHL(:,2);
handrule = -1; % it is 1 for right hand rule, and -1 for left hand rule


a1 = xopt(1);
a2 = xopt(2);
phi1 = xopt(3);
phi2 = xopt(4);
R1 = xopt(5);
R2 = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.R2;
f1 = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.f1;
f2 = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.f2;
n1 = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.n1;
n2 = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.n2;


% To avoid attachment point inside joint radius
if a1<R1 
a1 = R1;
end

if a2<R1
a2 = R1;
end


% if a1<R1 
% msg = 'Error occurred: a1<R1';
% error(msg)
% end
% 
% if a2<R1
% msg = 'Error occurred: a2<R1';
% error(msg)
% end


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





%% Computation of muscle force and moment
% Shoulder Protractor
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
MAMT = [MuscleMomentArm_time, MuscleMomentArm_time*0, MuscleMomentArm_time*0];
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


plot(GroupsOfMoments_Shoulder,'-k','LineWidth',2)
hold on
plot(MMT_Shoulder,'--r','LineWidth',1)
legend('Group of muscles','Combined muscle')
xlabel('Time Step')
ylabel('Shoulder Moment (Nm)')
title('Shoulder Moment')
% ylim([min(GroupsOfMoments_Shoulder)-3 max(GroupsOfMoments_Shoulder)+3])


% percentage error
PE = 100*sum(abs(GroupsOfMoments_Shoulder-MMT_Shoulder))./sum(abs(GroupsOfMoments_Shoulder));

% cost function
CF = sum(((GroupsOfMoments_Shoulder-MMT_Shoulder)./std(GroupsOfMoments_Shoulder)).^2);

R = corrcoef(GroupsOfMoments_Shoulder, MMT_Shoulder)

%% plot (3in3 figs)

Name = 'Shoulder Protractor';

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
ATI = scatter(XI_time(i,1), YI_time(i,1), 10, 'MarkerEdgeColor',[0, 0, 0]);
hold on
 
 % Line Model
 
 
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


% Joint surface

th = 0:pi/1000:2*pi;
x = XC1(i,1);
y = YC1(i,1);
r = R1;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
Cj1 = plot(xunit, yunit,'color','cyan');



% Attachment points

if (theta1_time(i,1)>=0)
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
  
  if (theta_P2(i,1)<theta_P1(i,1)) && f1==1 && gamma1==0
    th = 0;
    YP2_time(i,1) = YP1_time(i,1);
    XP2_time(i,1) = XP1_time(i,1);
  end  
  
  if (theta_P2(i,1)<theta_P1(i,1)) && f1==1 && gamma2==0
    th = 0;
    YP1_time(i,1) = YP2_time(i,1);
    XP1_time(i,1) = XP2_time(i,1);
  end  
  
  if (sign(YP1_time(i,1)- YC1(i,1)) == sign(YP2_time(i,1)- YC1(i,1))) && f1==-1 && (theta_P1(i,1)>=theta_P2(i,1))
      th = theta_P2(i,1):pi/1000:theta_P1(i,1); 
  end
  
  if (sign(YP1_time(i,1)- YC1(i,1)) == sign(YP2_time(i,1)- YC1(i,1))) && f1==-1 && (theta_P1(i,1)<theta_P2(i,1)) && gamma1==0
     th = 0;
     YP2_time(i,1) = YP1_time(i,1);
     XP2_time(i,1) = XP1_time(i,1);
  end
  
  if (sign(YP1_time(i,1)- YC1(i,1)) == sign(YP2_time(i,1)- YC1(i,1))) && f1==-1 && (theta_P1(i,1)<theta_P2(i,1)) && gamma2==0
     th = 0;
     YP1_time(i,1) = YP2_time(i,1);
     XP1_time(i,1) = XP2_time(i,1);
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
  Cj2 = plot(xunit, yunit,'color',[1 0 0],'LineWidth',4);
  hold on
  
  
% Lmt3   
  line([XP2_time(i,1) XI_time(i,1)],[YP2_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',4);
  hold on  
  
  
else
% Lmt   
  line([XO_time(i,1) XI_time(i,1)],[YO_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',4);
  hold on   
  
end

  
  
% figure properties 
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
% ylim([MinY MaxY])

txt_title = Name;
title(txt_title)
hold off


L1 = findobj(h,'type','line');
copyobj(L1,findobj(hgload('AllMuscleGroups.fig'),'type','axes'));
close(h)



%% Simulation Combined Muscle

f=0;


h = figure(5);
pause(2);
for i= 1:1:size(SCP,1)
       
      
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



%% Attachment points

if (theta1_time(i,1)>=0)
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
  
  if (theta_P2(i,1)<theta_P1(i,1)) && f1==1 && gamma1==0
    th = 0;
    YP2_time(i,1) = YP1_time(i,1);
    XP2_time(i,1) = XP1_time(i,1);
  end  
  
  if (theta_P2(i,1)<theta_P1(i,1)) && f1==1 && gamma2==0
    th = 0;
    YP1_time(i,1) = YP2_time(i,1);
    XP1_time(i,1) = XP2_time(i,1);
  end  
  
  if (sign(YP1_time(i,1)- YC1(i,1)) == sign(YP2_time(i,1)- YC1(i,1))) && f1==-1 && (theta_P1(i,1)>=theta_P2(i,1))
      th = theta_P2(i,1):pi/1000:theta_P1(i,1); 
  end
  
  if (sign(YP1_time(i,1)- YC1(i,1)) == sign(YP2_time(i,1)- YC1(i,1))) && f1==-1 && (theta_P1(i,1)<theta_P2(i,1)) && gamma1==0
     th = 0;
     YP2_time(i,1) = YP1_time(i,1);
     XP2_time(i,1) = XP1_time(i,1);
  end
  
  if (sign(YP1_time(i,1)- YC1(i,1)) == sign(YP2_time(i,1)- YC1(i,1))) && f1==-1 && (theta_P1(i,1)<theta_P2(i,1)) && gamma2==0
     th = 0;
     YP1_time(i,1) = YP2_time(i,1);
     XP1_time(i,1) = XP2_time(i,1);
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
  Cj2 = plot(xunit, yunit,'color',[1 0 0],'LineWidth',1.5);
  hold on
  
  
% Lmt3   
  line([XP2_time(i,1) XI_time(i,1)],[YP2_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
  hold on  
  
  
else
% Lmt   
  line([XO_time(i,1) XI_time(i,1)],[YO_time(i,1) YI_time(i,1)],'color',[1 0 0],'LineWidth',1.5);
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











