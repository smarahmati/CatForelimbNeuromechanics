% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% This M-file creates MuscleMP corresponding to the muscle mechanical
% properties and musculotendon lengths in 3D and 2D

clear;
clc;


% Load musculoskeletal data
MusculoskeletalData = load('MusculoskeletalData.mat');
MusculoskeletalData = MusculoskeletalData.MusculoskeletalData;

% Load muscle names and mechanical properties
[numMN, txtMN, rawMN] = xlsread('Muscles Considered For Modelling.xlsx');   % MN: Muscle name
[numMMP, txtMMP, rawMMP] = xlsread('ForelimbArchitecture.xlsx');           % MMP: Muscle Mechanical Properties

% Remove digits and quotes from muscle names
for i = 1:size(txtMN, 1)
    txtMN_NN{i, 1} = regexprep(txtMN{i, 1}, '[\d"]', ''); % txtMN_NN: txtMN No Number
end

% Match muscle names between the two files
idx = zeros(size(txtMN_NN, 1) - 1, size(txtMMP, 1));
for i = 2:size(txtMN_NN, 1)
    s = 0;
    for j = 2:size(txtMMP, 1)
        str = strtrim(lower(txtMMP{j, 1}));
        k = strcmp(str, strtrim(lower(txtMN_NN{i, 1})));
        if k == 1
            s = s + 1;
            % Each row of idx corresponds to the muscle in 'Muscles Considered For Modelling.xlsx'
            % Columns of idx show the row of repeated muscle name in 'ForelimbArchitecture.xlsx'
            idx(i - 1, s) = j; 
        end
    end
end

% Adjust indices to account for header rows
for i = 1:size(idx, 1)
    for j = 1:size(idx, 2)
        if idx(i, j) ~= 0
            idx(i, j) = idx(i, j) - 1;
        end
    end
end

% Define columns for different properties
Mass_Column = 3;
ML_Column = 4;
FL_Column = 5;
SL_Column = 6;
Sarc_Column = 7;
PCSA_Column = 8;
PA_Column = 9;
LM0_Column = 10;
FL_ratio_Column = 11;
OptimalML_Column = 12;
SO_Column = 13;
FOG_Column = 14;
FG_Column = 15;
Vmax_LM0_Column = 16;

% Function to format mean and std deviation as strings
formatStats = @(meanVal, stdVal, dec) [num2str(round(meanVal, dec)) char(177) num2str(round(stdVal, dec))];

% Calculate stats for each property and store results in tables
for i = 1:size(idx, 1)
    % Mass
    data = numMMP(nonzeros(idx(i, :)), Mass_Column);
    data = data(~isnan(data));
    [Mass_Ave(i, 1), Mass_std(i, 1)] = calculateStats(data);
    Mass_str{i, 1} = formatStats(Mass_Ave(i, 1), Mass_std(i, 1), 2);

    % Muscle Length (ML)
    data = numMMP(nonzeros(idx(i, :)), ML_Column);
    data = data(~isnan(data));
    [ML_Ave(i, 1), ML_std(i, 1)] = calculateStats(data);
    ML_str{i, 1} = formatStats(ML_Ave(i, 1), ML_std(i, 1), 1);

    % Fiber Length (FL)
    data = numMMP(nonzeros(idx(i, :)), FL_Column);
    data = data(~isnan(data));
    [FL_Ave(i, 1), FL_std(i, 1)] = calculateStats(data);
    FL_str{i, 1} = formatStats(FL_Ave(i, 1), FL_std(i, 1), 1);

    % Sarcomere Length (SL)
    data = numMMP(nonzeros(idx(i, :)), SL_Column);
    data = data(~isnan(data));
    [SL_Ave(i, 1), SL_std(i, 1)] = calculateStats(data);
    SL_str{i, 1} = formatStats(SL_Ave(i, 1), SL_std(i, 1), 1);

    % Sarcomere Number (Sarc)
    data = numMMP(nonzeros(idx(i, :)), Sarc_Column);
    data = data(~isnan(data));
    [Sarc_Ave(i, 1), Sarc_std(i, 1)] = calculateStats(data);
    Sarc_str{i, 1} = formatStats(Sarc_Ave(i, 1), Sarc_std(i, 1), 0);

    % Physiological Cross-Sectional Area (PCSA)
    data = numMMP(nonzeros(idx(i, :)), PCSA_Column);
    data = data(~isnan(data));
    [PCSA_Ave(i, 1), PCSA_std(i, 1)] = calculateStats(data);
    PCSA_str{i, 1} = formatStats(PCSA_Ave(i, 1), PCSA_std(i, 1), 2);

    % Pennation Angle (PA)
    data = numMMP(nonzeros(idx(i, :)), PA_Column);
    data = data(~isnan(data));
    [PA_Ave(i, 1), PA_std(i, 1)] = calculateStats(data);
    PA_str{i, 1} = formatStats(PA_Ave(i, 1), PA_std(i, 1), 0);

    % Optimal Muscle Length (LM0)
    data = numMMP(nonzeros(idx(i, :)), LM0_Column);
    data = data(~isnan(data));
    [LM0_Ave(i, 1), LM0_std(i, 1)] = calculateStats(data);
    LM0_str{i, 1} = formatStats(LM0_Ave(i, 1), LM0_std(i, 1), 1);

    % Fiber Length Ratio (FL_ratio)
    data = numMMP(nonzeros(idx(i, :)), FL_ratio_Column);
    data = data(~isnan(data));
    [FL_ratio_Ave(i, 1), FL_ratio_std(i, 1)] = calculateStats(data);
    FL_ratio_str{i, 1} = formatStats(FL_ratio_Ave(i, 1), FL_ratio_std(i, 1), 2);

    % Optimal Muscle Length (OptimalML)
    data = numMMP(nonzeros(idx(i, :)), OptimalML_Column);
    data = data(~isnan(data));
    [OptimalML_Ave(i, 1), OptimalML_std(i, 1)] = calculateStats(data);
    OptimalML_str{i, 1} = formatStats(OptimalML_Ave(i, 1), OptimalML_std(i, 1), 1);

    % Slow Oxidative (SO) Fiber Type
    data = numMMP(nonzeros(idx(i, :)), SO_Column);
    data = data(~isnan(data));
    [SO_Ave(i, 1), SO_std(i, 1)] = calculateStats(data);
    SO_str{i, 1} = num2str(round(SO_Ave(i, 1), 1));

    % Fast Oxidative Glycolytic (FOG) Fiber Type
    data = numMMP(nonzeros(idx(i, :)), FOG_Column);
    data = data(~isnan(data));
    [FOG_Ave(i, 1), FOG_std(i, 1)] = calculateStats(data);
    FOG_str{i, 1} = num2str(round(FOG_Ave(i, 1), 1));

    % Fast Glycolytic (FG) Fiber Type
    data = numMMP(nonzeros(idx(i, :)), FG_Column);
    data = data(~isnan(data));
    [FG_Ave(i, 1), FG_std(i, 1)] = calculateStats(data);
    FG_str{i, 1} = num2str(round(FG_Ave(i, 1), 1));

    % Combine Fiber Type Composition
    Fiber_Type{i, 1} = [SO_str{i, 1}, ',', FOG_str{i, 1}, ',', FG_str{i, 1}];

    % Maximal Muscle Velocity (Vmax_LM0)
    data = numMMP(nonzeros(idx(i, :)), Vmax_LM0_Column);
    data = data(~isnan(data));
    [Vmax_LM0_Ave(i, 1), Vmax_LM0_std(i, 1)] = calculateStats(data);
    Vmax_LM0_str{i, 1} = num2str(round(Vmax_LM0_Ave(i, 1), 1));
end

% Rows related to total muscle length
RTM = [1, 1; 2, 2; 5, 5; 6, 7; 8, 8; 9, 9; 10, 11; 14, 15; 16, 17; 18, 19; 20, 21; ...
       30, 31; 32, 33; 34, 34; 35, 36; 37, 38; 39, 40; 41, 42; 43, 44; 45, 46; 47, 48; ...
       49, 50; 51, 52; 53, 54; 55, 56; 57, 58; 59, 60; 61, 62; 63, 64; 65, 66; 67, 68; ...
       69, 69; 70, 70; 71, 71; 72, 72; 73, 73; 74, 74; 75, 75; 76, 76; 77, 77];

% Calculate musculotendon lengths in 3D and 2D
Musculotendon_Length3D = zeros(size(RTM, 1), 1);
Musculotendon_Length2D = zeros(size(RTM, 1), 1);
for i = 1:size(RTM, 1)
    if RTM(i, 1) == RTM(i, 2)
        Musculotendon_Length3D(i, 1) = (MusculoskeletalData.Three_Dimension.Musculotendon_Length(RTM(i, 1))) * 1000; % mm
        Musculotendon_Length2D(i, 1) = (MusculoskeletalData.Two_Dimension.Musculotendon_Length(RTM(i, 1))) * 1000;   % mm
    else
        Musculotendon_Length3D(i, 1) = (MusculoskeletalData.Three_Dimension.Musculotendon_Length(RTM(i, 1)) + MusculoskeletalData.Three_Dimension.Musculotendon_Length(RTM(i, 2))) * 1000; % mm  
        Musculotendon_Length2D(i, 1) = (MusculoskeletalData.Two_Dimension.Musculotendon_Length(RTM(i, 1)) + MusculoskeletalData.Two_Dimension.Musculotendon_Length(RTM(i, 2))) * 1000;     % mm 
    end
end

% Get last names
LastName = cell(size(txtMN, 1) - 1, 1);
for i = 2:size(txtMN, 1)
    LastName{i - 1, 1} = txtMN{i, 1};
end

% Abbreviations for last names
LastName_abb = {'AD'; 'ANC'; 'BB'; 'BCD'; 'BR'; 'CB'; 'ECR'; ...
                'EDC2'; 'EDC3'; 'EDC4'; 'EDC5'; 'EPL1'; 'EPL2'; ...
                'EPT'; 'FCR'; 'FCU'; 'FDP1'; 'FDP2'; 'FDP3'; 'FDP4'; 'FDP5'; ...
                'FDS2'; 'FDS3'; 'FDS4'; 'FDS5'; 'IF'; 'PL1'; 'PL2'; 'PL3'; 'PL4'; 'PL5'; ...
                'PT'; 'SD'; 'SPS'; 'SSC'; 'TJ'; 'TLAT'; 'TLONG'; 'TMED'; 'TN'};

% Create tables for muscle mechanical properties and save the results
MuscleMP = table(LastName, Mass_Ave, ML_Ave, FL_Ave, SL_Ave, Sarc_Ave, ...
                 PCSA_Ave, PA_Ave, LM0_Ave, FL_ratio_Ave, OptimalML_Ave, ...
                 SO_Ave, FOG_Ave, FG_Ave, Vmax_LM0_Ave, Musculotendon_Length3D, Musculotendon_Length2D);

MuscleMP_STD = table(LastName, LastName_abb, Mass_str, LM0_str, ...
                     PA_str, PCSA_str, Fiber_Type, Vmax_LM0_str);

% Save the results
save('MuscleMP.mat', 'MuscleMP');

disp('Muscle mechanical properties have been processed and saved to MuscleMP.mat.');

% Function to calculate mean and std deviation
function [meanVal, stdVal] = calculateStats(data)
    meanVal = mean(data, 'omitnan');
    stdVal = std(data, 'omitnan');
end
