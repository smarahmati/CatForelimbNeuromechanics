% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% This script processes and analyzes sensory feedbacks in cat locomotion.
% In order to compute sensory feedback (Ia, II, and Ib afferent activities), 
% muscle activations, muscle-tendon lengths, velocities, and forces for both individual muscles and equivalent muscle groups were used.
% The results are normalized and saved for further analysis.

clear;  % Clear workspace
clc;    % Clear command window

% Load necessary data
load('MotionData.mat');
load('MuscleLengthVelocityMomentArm.mat');
load('Activations.mat');
load('MuscleMP.mat');
load('OptimizedMuscleForceMoment.mat');
load('MuscleLengthVelocityMomentArm_9Groups.mat');
load('Activations_9Groups.mat');
load('MuscleMP_9Groups.mat');
load('OptimizedMuscleForceMoment_9Groups.mat');
load('MuscleCluster.mat');
load('Color_40.mat');

% Time calculation
CT = MotionData.GenericCat.CycleTime.Average;
dt = CT / (size(MuscleLengthVelocityMomentArm{1, 1}.Length, 1) - 1);
time = (0:size(MuscleLengthVelocityMomentArm{1, 1}.Length, 1) - 1)' * dt;

%% MT length, velocity, force calculations

% ---------- 40 muscles ---------------------------------------------------
for i = 1:size(MuscleLengthVelocityMomentArm, 1)
    % Muscle length and velocity (converted to mm and mm/s)
    LMT{i, 1} = MuscleLengthVelocityMomentArm{i, 1}.Length * 1000;     % mm
    VMT{i, 1} = MuscleLengthVelocityMomentArm{i, 1}.Velocity * 1000;   % mm/s

    % Pennation angle and muscle parameters
    PA = MuscleMP{i, 8} * (pi / 180);   % rad
    LF0 = MuscleMP{i, 9};               % mm
    Max_LMT = max(LMT{i, 1});           % mm
    LT{i, 1} = Max_LMT - 1.05 * LF0 * cos(PA);  % mm
    LMT0{i, 1} = LT{i, 1} + LF0 * cos(PA);      % mm

    % Muscle force calculations
    PCSA = MuscleMP{i, 7};                      % cm^2
    TM = 2.3;                                   % kg/cm^2
    g = 9.806;                                  % m/s^2
    FM_max{i, 1} = TM * PCSA * g;               % N
    FT_max{i, 1} = FM_max{i, 1} * cos(PA);      % N
end

% ---------- 9 combined muscles -------------------------------------------
for i = 1:size(MuscleLengthVelocityMomentArm_9Groups, 1)
    % Muscle length and velocity (converted to mm and mm/s)
    LMT_9Groups{i, 1} = MuscleLengthVelocityMomentArm_9Groups{i, 1}.Length * 1000;      % mm
    VMT_9Groups{i, 1} = MuscleLengthVelocityMomentArm_9Groups{i, 1}.Velocity * 1000;    % mm/s

    % Pennation angle and muscle parameters
    PA = MuscleMP_9Groups{i, 8} * (pi / 180);           % rad
    LF0 = MuscleMP_9Groups{i, 9};                       % mm
    Max_LMT = max(LMT_9Groups{i, 1});                   % mm
    LT_9Groups{i, 1} = Max_LMT - 1.05 * LF0 * cos(PA);  % mm
    LMT0_9Groups{i, 1} = LT_9Groups{i, 1} + LF0 * cos(PA);  % mm

    % Muscle force calculations
    PCSA = MuscleMP_9Groups{i, 7};                      % cm^2
    TM = 2.3;                                           % kg/cm^2
    g = 9.806;                                          % m/s^2
    FM_max_9Groups{i, 1} = TM * PCSA * g;               % N
    FT_max_9Groups{i, 1} = FM_max_9Groups{i, 1} * cos(PA); % N
end

%% Computation of sensory feedbacks

ratio = 1;

% ---------- 40 muscles ---------------------------------------------------
for i = 1:size(MuscleLengthVelocityMomentArm, 1)
    % Ia afferent activity
    RIa{i, 1}.Name = OptimizedMuscleForceMoment{i, 1}.Name;
    RIa{i, 1}.SensoryFeedback = 4.3 * sign(VMT{i, 1}) .* abs(VMT{i, 1}).^0.6 + ...
                               2 * (LMT{i, 1} - ratio * LMT0{i, 1}) + ...
                               100 * Activations{i, 1}.a;

    % II afferent activity
    RII{i, 1}.Name = OptimizedMuscleForceMoment{i, 1}.Name;
    RII{i, 1}.SensoryFeedback = 13.5 * (LMT{i, 1} - ratio * LMT0{i, 1}) + ...
                               20 * Activations{i, 1}.a;

    % Ib afferent activity
    kIb = 333;
    RIb{i, 1}.Name = OptimizedMuscleForceMoment{i, 1}.Name;
    RIb{i, 1}.SensoryFeedback = kIb * (OptimizedMuscleForceMoment{i, 1}.Force / FT_max{i, 1});

    % Store feedback in matrices
    RIa_Matrix(:, i) = RIa{i, 1}.SensoryFeedback;
    RII_Matrix(:, i) = RII{i, 1}.SensoryFeedback;
    RIb_Matrix(:, i) = RIb{i, 1}.SensoryFeedback;
end

% Minimum feedback values for normalization
MinRIa = min(RIa_Matrix);
MinRII = min(RII_Matrix);
MinRIb = min(RIb_Matrix);

% ---------- 40 muscles (normalized) --------------------------------------
for i = 1:size(MuscleLengthVelocityMomentArm, 1)
    RIa{i, 1}.Name = OptimizedMuscleForceMoment{i, 1}.Name;
    RIa{i, 1}.SensoryFeedback = 4.3 * sign(VMT{i, 1}) .* abs(VMT{i, 1}).^0.6 + ...
                               2 * (LMT{i, 1} - ratio * LMT0{i, 1}) + ...
                               100 * Activations{i, 1}.a - MinRIa(i);

    RII{i, 1}.Name = OptimizedMuscleForceMoment{i, 1}.Name;
    RII{i, 1}.SensoryFeedback = 13.5 * (LMT{i, 1} - ratio * LMT0{i, 1}) + ...
                               20 * Activations{i, 1}.a - MinRII(i);

    kIb = 333;
    RIb{i, 1}.Name = OptimizedMuscleForceMoment{i, 1}.Name;
    RIb{i, 1}.SensoryFeedback = kIb * (OptimizedMuscleForceMoment{i, 1}.Force / FT_max{i, 1}) - MinRIb(i);

    % Store normalized feedback in matrices
    RIa_Matrix(:, i) = RIa{i, 1}.SensoryFeedback;
    RII_Matrix(:, i) = RII{i, 1}.SensoryFeedback;
    RIb_Matrix(:, i) = RIb{i, 1}.SensoryFeedback;
end

% Save results
save('RIa.mat', 'RIa');
save('RII.mat', 'RII');
save('RIb.mat', 'RIb');

% ---------- 9 combined muscles -------------------------------------------
for i = 1:size(MuscleLengthVelocityMomentArm_9Groups, 1)
    RIa_9Groups{i, 1}.Name = OptimizedMuscleForceMoment_9Groups{i, 1}.Name;
    RIa_9Groups{i, 1}.SensoryFeedback = 4.3 * sign(VMT_9Groups{i, 1}) .* abs(VMT_9Groups{i, 1}).^0.6 + ...
                                        2 * (LMT_9Groups{i, 1} - ratio * LMT0_9Groups{i, 1}) + ...
                                        100 * Activations_9Groups{i, 1}.a;

    RII_9Groups{i, 1}.Name = OptimizedMuscleForceMoment_9Groups{i, 1}.Name;
    RII_9Groups{i, 1}.SensoryFeedback = 13.5 * (LMT_9Groups{i, 1} - ratio * LMT0_9Groups{i, 1}) + ...
                                        20 * Activations_9Groups{i, 1}.a;

    kIb = 333;
    RIb_9Groups{i, 1}.Name = OptimizedMuscleForceMoment_9Groups{i, 1}.Name;
    RIb_9Groups{i, 1}.SensoryFeedback = kIb * (OptimizedMuscleForceMoment_9Groups{i, 1}.Force / FT_max_9Groups{i, 1});

    % Store feedback in matrices
    RIa_9Groups_Matrix(:, i) = RIa_9Groups{i, 1}.SensoryFeedback;
    RII_9Groups_Matrix(:, i) = RII_9Groups{i, 1}.SensoryFeedback;
    RIb_9Groups_Matrix(:, i) = RIb_9Groups{i, 1}.SensoryFeedback;
end

% Minimum feedback values for normalization
MinRIa_9Groups = min(RIa_9Groups_Matrix);
MinRII_9Groups = min(RII_9Groups_Matrix);
MinRIb_9Groups = min(RIb_9Groups_Matrix);

% ---------- 9 combined muscles (normalized) ------------------------------
for i = 1:size(MuscleLengthVelocityMomentArm_9Groups, 1)
    RIa_9Groups{i, 1}.Name = OptimizedMuscleForceMoment_9Groups{i, 1}.Name;
    RIa_9Groups{i, 1}.SensoryFeedback = 4.3 * sign(VMT_9Groups{i, 1}) .* abs(VMT_9Groups{i, 1}).^0.6 + ...
                                        2 * (LMT_9Groups{i, 1} - ratio * LMT0_9Groups{i, 1}) + ...
                                        100 * Activations_9Groups{i, 1}.a - MinRIa_9Groups(i);

    RII_9Groups{i, 1}.Name = OptimizedMuscleForceMoment_9Groups{i, 1}.Name;
    RII_9Groups{i, 1}.SensoryFeedback = 13.5 * (LMT_9Groups{i, 1} - ratio * LMT0_9Groups{i, 1}) + ...
                                        20 * Activations_9Groups{i, 1}.a - MinRII_9Groups(i);

    kIb = 333;
    RIb_9Groups{i, 1}.Name = OptimizedMuscleForceMoment_9Groups{i, 1}.Name;
    RIb_9Groups{i, 1}.SensoryFeedback = kIb * (OptimizedMuscleForceMoment_9Groups{i, 1}.Force / FT_max_9Groups{i, 1}) - MinRIb_9Groups(i);

    % Store normalized feedback in matrices
    RIa_9Groups_Matrix(:, i) = RIa_9Groups{i, 1}.SensoryFeedback;
    RII_9Groups_Matrix(:, i) = RII_9Groups{i, 1}.SensoryFeedback;
    RIb_9Groups_Matrix(:, i) = RIb_9Groups{i, 1}.SensoryFeedback;
end

% Save results
save('RIa_9Groups.mat', 'RIa_9Groups');
save('RII_9Groups.mat', 'RII_9Groups');
save('RIb_9Groups.mat', 'RIb_9Groups');

% Plotting the results
figure(1)

%% Ia afferent activity plots
plotAffActivity(1, 1, 'Shoulder protractors', 'Ia afferent activity', MuscleCluster, time, Color_40, RIa, RIa_9Groups);
plotAffActivity(4, 2, 'Elbow extensors', 'Ia afferent activity', MuscleCluster, time, Color_40, RIa, RIa_9Groups);
plotAffActivity(7, 3, 'Shoulder protractor-Elbow flexor (Biceps Brachii)', 'Ia afferent activity', MuscleCluster, time, Color_40, RIa, RIa_9Groups);
plotAffActivity(10, 4, 'Elbow flexors', 'Ia afferent activity', MuscleCluster, time, Color_40, RIa, RIa_9Groups);
plotAffActivity(13, 5, 'Elbow flexors-Wrist dorsiflexors', 'Ia afferent activity', MuscleCluster, time, Color_40, RIa, RIa_9Groups);
plotAffActivity(16, 6, 'Wrist dorsiflexors', 'Ia afferent activity', MuscleCluster, time, Color_40, RIa, RIa_9Groups);
plotAffActivity(19, 7, 'Wrist plantarflexors', 'Ia afferent activity', MuscleCluster, time, Color_40, RIa, RIa_9Groups);
plotAffActivity(22, 8, 'Shoulder retractors', 'Ia afferent activity', MuscleCluster, time, Color_40, RIa, RIa_9Groups);
plotAffActivity(25, 9, 'Shoulder retractor-Elbow extensor (Triceps Brachii Long)', 'Ia afferent activity', MuscleCluster, time, Color_40, RIa, RIa_9Groups);

%% II afferent activity plots
plotAffActivity(2, 1, 'Shoulder protractors', 'II afferent activity', MuscleCluster, time, Color_40, RII, RII_9Groups);
plotAffActivity(5, 2, 'Elbow extensors', 'II afferent activity', MuscleCluster, time, Color_40, RII, RII_9Groups);
plotAffActivity(8, 3, 'Shoulder protractor-Elbow flexor (Biceps Brachii)', 'II afferent activity', MuscleCluster, time, Color_40, RII, RII_9Groups);
plotAffActivity(11, 4, 'Elbow flexors', 'II afferent activity', MuscleCluster, time, Color_40, RII, RII_9Groups);
plotAffActivity(14, 5, 'Elbow flexors-Wrist dorsiflexors', 'II afferent activity', MuscleCluster, time, Color_40, RII, RII_9Groups);
plotAffActivity(17, 6, 'Wrist dorsiflexors', 'II afferent activity', MuscleCluster, time, Color_40, RII, RII_9Groups);
plotAffActivity(20, 7, 'Wrist plantarflexors', 'II afferent activity', MuscleCluster, time, Color_40, RII, RII_9Groups);
plotAffActivity(23, 8, 'Shoulder retractors', 'II afferent activity', MuscleCluster, time, Color_40, RII, RII_9Groups);
plotAffActivity(26, 9, 'Shoulder retractor-Elbow extensor (Triceps Brachii Long)', 'II afferent activity', MuscleCluster, time, Color_40, RII, RII_9Groups);

%% Ib afferent activity plots
plotAffActivity(3, 1, 'Shoulder protractors', 'Ib afferent activity', MuscleCluster, time, Color_40, RIb, RIb_9Groups);
plotAffActivity(6, 2, 'Elbow extensors', 'Ib afferent activity', MuscleCluster, time, Color_40, RIb, RIb_9Groups);
plotAffActivity(9, 3, 'Shoulder protractor-Elbow flexor (Biceps Brachii)', 'Ib afferent activity', MuscleCluster, time, Color_40, RIb, RIb_9Groups);
plotAffActivity(12, 4, 'Elbow flexors', 'Ib afferent activity', MuscleCluster, time, Color_40, RIb, RIb_9Groups);
plotAffActivity(15, 5, 'Elbow flexors-Wrist dorsiflexors', 'Ib afferent activity', MuscleCluster, time, Color_40, RIb, RIb_9Groups);
plotAffActivity(18, 6, 'Wrist dorsiflexors', 'Ib afferent activity', MuscleCluster, time, Color_40, RIb, RIb_9Groups);
plotAffActivity(21, 7, 'Wrist plantarflexors', 'Ib afferent activity', MuscleCluster, time, Color_40, RIb, RIb_9Groups);
plotAffActivity(24, 8, 'Shoulder retractors', 'Ib afferent activity', MuscleCluster, time, Color_40, RIb, RIb_9Groups);
plotAffActivity(27, 9, 'Shoulder retractor-Elbow extensor (Triceps Brachii Long)', 'Ib afferent activity', MuscleCluster, time, Color_40, RIb, RIb_9Groups);

% Function to plot afferent activity
function plotAffActivity(subplotIndex, muscleGroup, subtitleText, titleText, MuscleCluster, time, Color_40, feedbackData, feedbackData_9Groups)
    subplot(9, 3, subplotIndex);
    MinY = [];
    MaxY = [];
    for i = 1:size(MuscleCluster{muscleGroup, 1}, 1)
        j = MuscleCluster{muscleGroup, 1}(i);
        plot(time * 100 / max(time), feedbackData{j, 1}.SensoryFeedback, '-', 'LineWidth', 1, 'Color', Color_40(j, :), 'DisplayName', feedbackData{j, 1}.Name);
        hold on;
        set(gca, 'XTick', []);
        ylabel('FR, imp/s');
        title(titleText);
        subtitle(subtitleText);
        MinY = [MinY; min(feedbackData{j, 1}.SensoryFeedback)];
        MaxY = [MaxY; max(feedbackData{j, 1}.SensoryFeedback)];
    end
    plot(time * 100 / max(time), feedbackData_9Groups{muscleGroup, 1}.SensoryFeedback, '-', 'LineWidth', 2, 'Color', [0 0 0], 'DisplayName', feedbackData_9Groups{muscleGroup, 1}.Name);
    MinY = min([MinY; feedbackData_9Groups{muscleGroup, 1}.SensoryFeedback]);
    MaxY = max([MaxY; feedbackData_9Groups{muscleGroup, 1}.SensoryFeedback]);
    MinY = MinY - 0.1 * abs(MinY);
    MaxY = MaxY + 0.1 * abs(MaxY);
    ylim([MinY MaxY]);
    n = 15;
    y_tran = linspace(MinY, MaxY, n);
    x_tran = repelem((1 - evalin('base', 'MotionData.GenericCat.DutyFactor.Average')) * 100, n);
    plot(x_tran, y_tran, '.k');
end
