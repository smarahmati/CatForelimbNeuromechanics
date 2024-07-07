% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% This script calculates the biomechanical variables of a selected 
% musculotendon unit (MTU) during locomotion based on input data, 
% including cycle time from MotionData for computing time, muscle length, velocity, moment arms, 
% and muscle mechanical parameters. The results include muscle fascicle 
% and tendon lengths, velocities, forces, and moments. The script also 
% generates plots to visualize these variables over a locomotion cycle.

clear
clc

% Select a muscle from a list of 40 muscles
rowNumber = selectMuscle_from40();

% Load motion data including joint angles and moments
MotionData = load('MotionData.mat');
MotionData = MotionData.MotionData;

% Load muscle length, velocity, and moment arm data
MuscleLengthVelocityMomentArm = load('MuscleLengthVelocityMomentArm.mat');
MuscleLengthVelocityMomentArm = MuscleLengthVelocityMomentArm.MuscleLengthVelocityMomentArm;

% Load muscle mechanical parameters
T = load('MuscleMP.mat');
T2 = T.MuscleMP;
MuscleMP = table2array(T2(1:end, 2:end));

% Muscle number
mn = rowNumber;

% Extract muscle-tendon properties for the selected muscle
NMT = MuscleLengthVelocityMomentArm{mn, 1}.Name;               % Name of MusculoTendon
LMT = MuscleLengthVelocityMomentArm{mn, 1}.Length;             % Length of MusculoTendon
VMT = MuscleLengthVelocityMomentArm{mn, 1}.Velocity;           % Velocity of MusculoTendon
MAMT = MuscleLengthVelocityMomentArm{mn, 1}.MomentArm;         % Moment Arm of MusculoTendon
EJMT = MuscleLengthVelocityMomentArm{mn, 1}.EffectiveJoints;   % Effective Joints of MusculoTendon

% Calculate time vector
CT = MotionData.GenericCat.CycleTime.Average;
dt = CT / (size(LMT, 1) - 1);
time = (0:size(LMT, 1) - 1)' * dt;

% Set muscle activation to maximum (1)
a = ones(size(LMT, 1), 1);

% Extract muscle mechanical parameters
LF0 = MuscleMP(mn, 8);                     % Optimal fascicle length (mm)
Vmax_LF0 = -MuscleMP(mn, 14);              % Vmax/LF0 (1/s) is negative for current formulation
SO = MuscleMP(mn, 11);                     % Percentage of slow-twitch fibres
if SO == 0, SO = 1; end
PCSA = MuscleMP(mn, 6);                    % Physiological cross sectional area (cm^2)
PA = MuscleMP(mn, 7);                      % Pennation angle (deg)
LF0 = LF0 / 1000;                          % Convert mm to m
Vmax = Vmax_LF0 * LF0;                     % Maximum velocity (m/s)
LT0 = 0.5 * LF0;                           % Estimated tendon length
TM = 2.3;                                  % Specific tensions of cat muscles (kg*cm^2)
g = 9.806;                                 % Gravity acceleration (m/s^2)
FM_max = TM * PCSA * g;                    % Maximum muscle force (Newton)
aV_FMmax = 0.00915 * SO - 0.00467;         % aV/Fm_max = 0.00915 · S − 0.00467 (normalized by Fm_max)
aV = aV_FMmax * FM_max;                    % aV = aV_FMmax * FM_max
Max_LMT = max(LMT);                        % Maximum musculotendon length

% Calculate biomechanical variables using the MTForce function
[LT, LF, VF, NLF, NVF, FPE, FCE_L, FCE_V, FM, FMT] = MTForce(a, time, LMT, VMT, LT0, LF0, aV, Vmax, FM_max, PA, Max_LMT);

% Calculate moments of musculotendon
MMT_Shoulder = MAMT(:,1) .* FMT;
MMT_Elbow = MAMT(:,2) .* FMT;
MMT_Wrist = MAMT(:,3) .* FMT;

% Plotting results
figure(1)

% Plot length of musculotendon
subplot(3,3,1)
plot(time, LMT, '-k', 'LineWidth', 1)
hold on
grid on
xlabel('Time (s)');
ylabel('LMT (m)');
title(['Length of Musculotendon (', NMT, ')'])
set(gca, 'XTick', [])
y_min = min(LMT);
y_max = max(LMT);
max_min = y_max - y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');
legend('Swing', 'Stance');

% Plot length of muscle fiber
subplot(3,3,4)
plot(time, LF, '--b', 'LineWidth', 1)
hold on
grid on
xlabel('Time (s)');
ylabel('LF (m)');
title('Length of Muscle Fiber')
set(gca, 'XTick', [])
y_min = min(LF);
y_max = max(LF);
max_min = y_max - y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');

% Plot velocity of muscle fiber
subplot(3,3,7)
plot(time, VF, '-.r', 'LineWidth', 1)
hold on
grid on
xlabel('Time (s)');
ylabel('VF (m/s)');
title('Velocity of Muscle Fiber')
hold on
y_min = min(VF);
y_max = max(VF);
max_min = y_max - y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');

% Plot muscle parallel elastic element force
subplot(3,3,2)
plot(NLF, FPE, '.m', 'LineWidth', 1)
hold on
grid on
xlabel('Normalized length of muscle (LF/LF0)');
ylabel('Normalized Force (FPE/FMmax)');
title(['Muscle parallel elastic element (', NMT, ')'])

% Plot muscle force-length relationship
subplot(3,3,5)
plot(NLF, FCE_L, '.b', 'LineWidth', 1)
hold on
grid on
xlabel('Normalized length of muscle (LF/LF0)');
ylabel('Normalized Force (FCEl/FMmax)');
title('Muscle force-length relationship')

% Plot muscle force-velocity relationship
subplot(3,3,8)
plot(NVF, FCE_V, '.r', 'LineWidth', 1)
hold on
grid on
xlabel('Normalized velocity of muscle (VF/Vmax)');
ylabel('Normalized Force (FCEv/FMmax)');
title('Muscle force-velocity relationship')

% Plot moment arm and force/moment for specific joints
if (MAMT(1,1) ~= 0) && (MAMT(1,2) == 0) && (MAMT(1,3) == 0)
    % Shoulder
    subplot(3,3,3)
    plot(time, MAMT(:,1), '-m', 'LineWidth', 1)
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('Moment arm of MT (m)');
    title(['Shoulder (', NMT, ')'])
    set(gca, 'XTick', [])
    y_min = min(MAMT(:,1));
    y_max = max(MAMT(:,1));
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');

    subplot(3,3,6)
    plot(time, FMT, '--b', 'LineWidth', 1)
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('MT force (N)');
    set(gca, 'XTick', [])
    y_min = min(FMT);
    y_max = max(FMT);
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');

    subplot(3,3,9)
    plot(time, MMT_Shoulder, '-k', 'LineWidth', 1)
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('MT moment (Nm)');
    y_min = min(MMT_Shoulder);
    y_max = max(MMT_Shoulder);
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');
    
elseif (MAMT(1,1) == 0) && (MAMT(1,2) ~= 0) && (MAMT(1,3) == 0)
    % Elbow
    subplot(3,3,3)
    plot(time, MAMT(:,2), '-m', 'LineWidth', 1)
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('Moment arm of MT (m)');
    title(['Elbow (', NMT, ')'])
    set(gca, 'XTick', [])
    y_min = min(MAMT(:,2));
    y_max = max(MAMT(:,2));
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');

    subplot(3,3,6)
    plot(time, FMT, '--b', 'LineWidth', 1)
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('MT force (N)');
    set(gca, 'XTick', [])
    y_min = min(FMT);
    y_max = max(FMT);
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');

    subplot(3,3,9)
    plot(time, MMT_Elbow, '-k', 'LineWidth', 1)
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('MT moment (Nm)');
    y_min = min(MMT_Elbow);
    y_max = max(MMT_Elbow);
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');
    
elseif (MAMT(1,1) == 0) && (MAMT(1,2) == 0) && (MAMT(1,3) ~= 0)
    % Wrist
    subplot(3,3,3)
    plot(time, MAMT(:,3), '-m', 'LineWidth', 1)
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('Moment arm of MT (m)');
    title(['Wrist (', NMT, ')'])
    set(gca, 'XTick', [])
    y_min = min(MAMT(:,3));
    y_max = max(MAMT(:,3));
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');

    subplot(3,3,6)
    plot(time, FMT, '--b', 'LineWidth', 1)
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('MT force (N)');
    set(gca, 'XTick', [])
    y_min = min(FMT);
    y_max = max(FMT);
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');

    subplot(3,3,9)
    plot(time, MMT_Wrist, '-k', 'LineWidth', 1)
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('MT moment (Nm)');
    y_min = min(MMT_Wrist);
    y_max = max(MMT_Wrist);
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');
    
elseif (MAMT(1,1) ~= 0) && (MAMT(1,2) ~= 0) && (MAMT(1,3) == 0)
    % Shoulder and Elbow
    subplot(3,3,3)
    p1 = plot(time, MAMT(:,1), '-m', 'LineWidth', 1);
    hold on
    p2 = plot(time, MAMT(:,2), '-.m', 'LineWidth', 1);
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('Moment arm of MT (m)');
    title(['Shoulder & Elbow (', NMT, ')'])
    set(gca, 'XTick', [])
    y_min = min([MAMT(:,1); MAMT(:,2)]);
    y_max = max([MAMT(:,1); MAMT(:,2)]);
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');
    legend([p1, p2], 'Shoulder', 'Elbow');

    subplot(3,3,6)
    plot(time, FMT, '--b', 'LineWidth', 1)
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('MT force (N)');
    set(gca, 'XTick', [])
    y_min = min(FMT);
    y_max = max(FMT);
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');

    subplot(3,3,9)
    p1 = plot(time, MMT_Shoulder, '-k', 'LineWidth', 1);
    hold on
    p2 = plot(time, MMT_Elbow, '-.k', 'LineWidth', 1);
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('MT moment (Nm)');
    y_min = min([MMT_Shoulder; MMT_Elbow]);
    y_max = max([MMT_Shoulder; MMT_Elbow]);
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');
    legend([p1, p2], 'Shoulder', 'Elbow');

elseif (MAMT(1,1) == 0) && (MAMT(1,2) ~= 0) && (MAMT(1,3) ~= 0)
    % Elbow and Wrist
    subplot(3,3,3)
    p1 = plot(time, MAMT(:,2), '-m', 'LineWidth', 1);
    hold on
    p2 = plot(time, MAMT(:,3), '-.m', 'LineWidth', 1);
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('Moment arm of MT (m)');
    title(['Elbow & Wrist (', NMT, ')'])
    set(gca, 'XTick', [])
    y_min = min([MAMT(:,2); MAMT(:,3)]);
    y_max = max([MAMT(:,2); MAMT(:,3)]);
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');
    legend([p1, p2], 'Elbow', 'Wrist');

    subplot(3,3,6)
    plot(time, FMT, '--b', 'LineWidth', 1)
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('MT force (N)');
    set(gca, 'XTick', [])
    y_min = min(FMT);
    y_max = max(FMT);
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');

    subplot(3,3,9)
    p1 = plot(time, MMT_Elbow, '-k', 'LineWidth', 1);
    hold on
    p2 = plot(time, MMT_Wrist, '-.k', 'LineWidth', 1);
    hold on
    grid on
    xlabel('Time (s)');
    ylabel('MT moment (Nm)');
    y_min = min([MMT_Elbow; MMT_Wrist]);
    y_max = max([MMT_Elbow; MMT_Wrist]);
    max_min = y_max - y_min;
    x_min = 0;
    x_max = time(end);
    x_mid = x_max * (1 - MotionData.GenericCat.DutyFactor.Average);
    line([x_min, x_mid], [y_min - 0.05 * max_min, y_min - 0.05 * max_min], 'LineWidth', 2, 'LineStyle', '-.', 'Color', 'g');
    line([x_mid, x_max], [y_min - 0.1 * max_min, y_min - 0.1 * max_min], 'LineWidth', 2, 'LineStyle', '-', 'Color', 'g');
    legend([p1, p2], 'Elbow', 'Wrist');
end
