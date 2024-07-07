% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% This M-file calculates the maximum musculotendon forces and moments 
% resulting from maximum activation during locomotion. The results are saved 
% in a structure array 'MaxMuscleForceMoment'. The input data includes 
% cycle time from MotionData for computing time, muscle length, velocity, moment arm data, and muscle 
% mechanical parameters.


clear;
clc;


% Motion results including joint angles and moments
MotionData = load('MotionData.mat');
MotionData = MotionData.MotionData;

MuscleLengthVelocityMomentArm = load('MuscleLengthVelocityMomentArm.mat');
MuscleLengthVelocityMomentArm = MuscleLengthVelocityMomentArm.MuscleLengthVelocityMomentArm;

T = load('MuscleMP.mat');
T2 = T.MuscleMP;
MuscleMP = table2array(T2(1:end, 2:end));



for mn = 1:length(MuscleLengthVelocityMomentArm)

NMT = MuscleLengthVelocityMomentArm{mn, 1}.Name;               % Name of MusculoTendon
LMT = MuscleLengthVelocityMomentArm{mn, 1}.Length;             % Length of MusculoTendon
VMT = MuscleLengthVelocityMomentArm{mn, 1}.Velocity;           % Length of MusculoTendon
MAMT = MuscleLengthVelocityMomentArm{mn, 1}.MomentArm;         % Moment Arm of MusculoTendon
EJMT = MuscleLengthVelocityMomentArm{mn, 1}.EffectiveJoints;   % Effective Joints of MusculoTendon

% time
CT = MotionData.GenericCat.CycleTime.Average;
dt = CT/(size(LMT,1)-1);
time = (0:size(LMT,1)-1)'*dt;

% activation
a = ones(size(LMT,1),1);

% Muscle mechanical parameters 
LF0 = MuscleMP(mn,8);                     % Optimal fascicle length (mm)
Vmax_LF0 = -MuscleMP(mn,14);              % Vamx/LF0 (1/s) is negative for current formulation
SO = MuscleMP(mn,11);                     % Percentage of slow-twitch fibres
if SO == 0, SO = 1; end
PCSA = MuscleMP(mn,6);                    % Physiological cross sectional area (cm^2)
PA = MuscleMP(mn,7);                      % Pennation angle (deg)
LF0 = LF0/1000;                           % mm to m
Vmax = Vmax_LF0*LF0;                      % Maximum velocity (m/s)
LT0 = 0.5*LF0;                            % We do not know its value for now
TM = 2.3;                                 % Specific tensions of cat muscles (kg*cm^2)
g = 9.806;                                % Gravity acceleration (m/s^2)
FM_max = TM*PCSA*g;                       % Maximum muscle force (Newton)
aV_FMmax = 0.00915*SO-0.00467;            % aV/Fm_max = 0.00915· S − 0.00467 (normalized by Fm_max)
aV = aV_FMmax*FM_max;                     % aV = aV_FMmax*FM_max
Max_LMT = max(LMT);                       % Maximum musculotendon length


[LT, LF, VF, NLF, NVF, FPE, FCE_L, FCE_V, FM, FMT] = MTForce(a, time, LMT, VMT, LT0, LF0, aV, Vmax, FM_max, PA, Max_LMT);

% Moment of Musculotendon
MMT_Shoulder = MAMT(:,1).*FMT;
MMT_Elbow = MAMT(:,2).*FMT;
MMT_Wrist = MAMT(:,3).*FMT;



%% Results
MaxMuscleForceMoment{mn,1}.Name = NMT;
MaxMuscleForceMoment{mn,1}.Length.Fascicle = LF;
MaxMuscleForceMoment{mn,1}.Length.OptimalFascicle = LF0;
MaxMuscleForceMoment{mn,1}.Length.Tendon = LT;
MaxMuscleForceMoment{mn,1}.Force.FCE_L = FCE_L; % It has been already normalized
MaxMuscleForceMoment{mn,1}.Force.FCE_V = FCE_V; % It has been already normalized
MaxMuscleForceMoment{mn,1}.Force.FM = FM;
MaxMuscleForceMoment{mn,1}.Force.FMT = FMT;
MaxMuscleForceMoment{mn,1}.Force.MaximumIsometricForce = FM_max;
MaxMuscleForceMoment{mn,1}.Moment.Shoulder = MMT_Shoulder;
MaxMuscleForceMoment{mn,1}.Moment.Elbow = MMT_Elbow;
MaxMuscleForceMoment{mn,1}.Moment.Wrist = MMT_Wrist;

end

%% Write Results
save('MaxMuscleForceMoment.mat', 'MaxMuscleForceMoment');

 