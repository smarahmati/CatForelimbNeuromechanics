function [Mean_MMT_Wrist] = Muscle_Moment_G7(x)

% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
% This script computes the maximum muscle moment averaged over the cycle as a function of muscle parameters.

a1OW = x(1);
a2OW = x(2);
phi1OW = x(3);
phi2OW = x(4);
a2WI = x(5);
phi2WI = x(6);
R1WI = x(7);
LF0 = x(8);
Vmax_LF0 = x(9);
SO = x(10);
PCSA = x(11);
PA = x(12);

NineMuscleMorphParaInitial = load('NineMuscleMorphParaInitial.mat');
NineMuscleMorphParaInitial = NineMuscleMorphParaInitial.NineMuscleMorphParaInitial;

G = 7; % Muscle Group Number (G = 1 to 9)
MotionData = load('MotionData.mat');
MotionData = MotionData.MotionData;




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


% Wrist Extensor (O->W) --------------------------------------------------------------------
y1OW = q3;
XC1OW = ELB(:,1);
YC1OW = ELB(:,2);
handrule = -1; % it is 1 for right hand rule, and -1 for left hand rule

%{
a1OW = xopt(1);
a2OW = xopt(2);
phi1OW = xopt(3);
phi2OW = xopt(4);
%}

for i=1:size(y1OW,1)
[MuscleLengthOW, MuscleVelocityOW, MuscleMomentArmOW] = MuscleZeroJoint_JointAngles(a1OW,a2OW,phi1OW,phi2OW);
MuscleLengthOW_time_JA(i,1) = MuscleLengthOW;
MuscleVelocityOW_time_JA(i,1) =  MuscleVelocityOW;
MuscleMomentArmOW_time(i,1) = MuscleMomentArmOW;
end




% Wrist Extensor(W->I)  --------------------------------------------------------------------
alpha1WI = WRTa;
alpha1WI_dot = WRTa_dot;
y1WI = q3;
y2WI = q4;
XC1WI = WRT(:,1);
YC1WI = WRT(:,2);


a1WI = sqrt(Lfarm_Mo2D^2 + a2OW^2 - 2*Lfarm_Mo2D*a2OW*cos(phi2OW));
% a2WI = xopt(5);
phi1WI = -asin(a2OW*sin(phi2OW)/a1WI);
% phi2WI = xopt(6);
% R1WI = xopt(7);
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
MuscleVelocityWI_time_JA(i,1) =  MuscleVelocityWI;
MuscleMomentArmWI_time(i,1) = MuscleMomentArmWI;
end





%% Computation of muscle force and moment
% Wrist Extensor
% Muscle Group Number (G = 1 to 9)

LMT = MuscleLengthOW_time_JA + MuscleLengthWI_time_JA;
VMT = MuscleVelocityOW_time_JA + MuscleVelocityWI_time_JA;


% activation
a = repmat(1, size(LMT,1),1);

% Muscle mechanical parameters 
% LF0 = Data_MuscleMP_9Groups(G,8);         % Optimal fascicle length (mm)
% Vmax_LF0 = -Data_MuscleMP_9Groups(G,14);  % Vamx/LF0 (1/s) is negative for current formulation
% SO = Data_MuscleMP_9Groups(G,11);         % Percentage of slow-twitch fibres
if SO <= 1, SO = 1; end 
% PCSA = Data_MuscleMP_9Groups(G,6);        % Physiological cross sectional area (cm^2)
% PA = Data_MuscleMP_9Groups(G,7);          % Pennation angle (deg)
LF0 = LF0/1000;                           % mm to m
Vmax = Vmax_LF0*LF0;                      % Maximum velocity (m/s)
LT0 = 0.5*LF0;                            % we do not know its value for now
TM = 2.3;                                 % Specific tensions of cat muscles (kg/cm^2)
g = 9.806;                                % Gravity acceleration (m/s^2)
FM_max = TM*PCSA*g;                       % Maximum muscle force (Newton)
aV_FMmax = 0.00915*SO-0.00467;            % aV/Fm_max = 0.00915· S − 0.00467 (normalized by Fm_max)
aV = aV_FMmax*FM_max;                     % aV = aV_FMmax*FM_max
Max_LMT = max(LMT);



[LT, LF, VF, NLF, NVF, FPE, FCE_L, FCE_V, FM, FMT] = MTForce(a, time, LMT, VMT, LT0, LF0, aV, Vmax, FM_max, PA, Max_LMT);
MAMT = [MuscleMomentArmWI_time*0, MuscleMomentArmWI_time*0, MuscleMomentArmWI_time];
% Moment Arm of Musculotendon
MAMT_Shoulder = MAMT(:,1);
MAMT_Elbow = MAMT(:,2);
MAMT_Wrist = MAMT(:,3);

% Moment of Musculotendon
MMT_Shoulder = MAMT(:,1).*FMT;
MMT_Elbow = MAMT(:,2).*FMT;
MMT_Wrist = MAMT(:,3).*FMT;


Mean_MMT_Wrist = mean(MMT_Wrist);

end

