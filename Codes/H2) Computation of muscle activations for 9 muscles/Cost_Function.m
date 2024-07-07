function [ Total_Muscle_Fatigue ] = Cost_Function (xopt)

% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% This M-file minimizes muscle fatigue: (FMT/PCSA)^3 
% xopt is activation for 9 muscles
% MMT is a function of muscle activation (xopt) & time step (TS); TS has
% been loaded
% Act: muscles activations (9 muscles) at time step of TS
% TS: time step (example: 0 to 101)

% Motion results including joint angles and moments
MotionData = load('MotionData.mat');
MotionData = MotionData.MotionData;

% 9 combined Muscles
MuscleLengthVelocityMomentArm_9Groups = load('MuscleLengthVelocityMomentArm_9Groups.mat');
MuscleLengthVelocityMomentArm_9Groups = MuscleLengthVelocityMomentArm_9Groups.MuscleLengthVelocityMomentArm_9Groups;

% 9 combined Muscles
MuscleMP_9Groups = load('MuscleMP_9Groups.mat');
MuscleMP_9Groups = MuscleMP_9Groups.MuscleMP_9Groups;
Data_MuscleMP_9Groups = table2array(MuscleMP_9Groups(1:end, 2:end));



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





%% Computation of muscle force and moment
% Muscle Number (mn = 1 to 9)

% activation for 9 muscle groups
a = xopt;

for mn = 1:size(MuscleLengthVelocityMomentArm_9Groups,1)

% Musculotendon length and velocity
LMT = MuscleLengthVelocityMomentArm_9Groups{mn, 1}.Length;
VMT = MuscleLengthVelocityMomentArm_9Groups{mn, 1}.Velocity;

% Moment Arm of Musculotendon
MAMT = MuscleLengthVelocityMomentArm_9Groups{mn, 1}.MomentArm;


% Muscle mechanical parameters 
LF0 = Data_MuscleMP_9Groups(mn,8);        % Optimal fascicle length (mm)
Vmax_LF0 = -Data_MuscleMP_9Groups(mn,14); % Vamx/LF0 (1/s) is negative for current formulation
SO = Data_MuscleMP_9Groups(mn,11);        % Percentage of sl-twitch fibres
if SO <= 1, SO = 1; end 
PCSA = Data_MuscleMP_9Groups(mn,6);       % Physiological cross sectional area (cm^2)
PA = Data_MuscleMP_9Groups(mn,7);         % Pennation angle (deg)
LF0 = LF0/1000;                           % mm to m
Vmax = Vmax_LF0*LF0;                      % Maximum velocity (m/s)
LT0 = 0.5*LF0;                            % we do not kn its value for n
TM = 2.3;                                 % Specific tensions of cat muscles (kg/cm^2)
g = 9.806;                                % Gravity acceleration (m/s^2)
FM_max = TM*PCSA*g;                       % Maximum muscle force (Newton)
aV_FMmax = 0.00915*SO-0.00467;            % aV/Fm_max = 0.00915· S − 0.00467 (normalized by Fm_max)
aV = aV_FMmax*FM_max;                     % aV = aV_FMmax*FM_max
Max_LMT = max(LMT);                       % Maximum musculotendon length

load('TS.mat');
[LT, LF, VF, NLF, NVF, FPE, FCE_L, FCE_V, FM, FMT] = MTForce(a(mn), time(TS), LMT(TS), VMT(TS), LT0, LF0, aV, Vmax, FM_max, PA, Max_LMT);



% Muscle fatigue
  Muscle_Fatigue(mn,1) = (FMT/PCSA)^3;

  

% Moment of Musculotendon
MMT_Shoulder(mn,:) = MAMT(TS,1).*FMT;
MMT_Elbow(mn,:) = MAMT(TS,2).*FMT;
MMT_Wrist(mn,:) = MAMT(TS,3).*FMT;
end


MMT(1,1) = sum(MMT_Shoulder); % Moment of all musculotendons around shoulder 
MMT(1,2) = sum(MMT_Elbow);    % Moment of all musculotendons around elbow 
MMT(1,3) = sum(MMT_Wrist);    % Moment of all musculotendons around wrist 


% Total muscle fatigue
  Total_Muscle_Fatigue = sum(Muscle_Fatigue);       %(1)


end



