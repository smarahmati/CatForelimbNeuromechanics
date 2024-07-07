% Description:
% This script processes and compares muscle activations from individual muscles and their equivalent muscles with EMG data. 

clear;
clc;

% Load necessary data files
load('Activations.mat');
load('Activations_9Groups.mat');
load('MotionData.mat');

% Define muscle names from EMG data
MuscleNames_EMGs = {'BB'; 'ECR'; 'PT'; 'SPS'; 'TLONG'; 'TLAT'; 'BR'}; % From EMGs

% Extract muscle names from Activations and Activations_9Groups
MuscleNames_Acts = cell(length(Activations), 1);
for i = 1:length(Activations)
    MuscleNames_Acts{i, 1} = Activations{i, 1}.Name;
end

MuscleNames_Acts_9Groups = cell(length(Activations_9Groups), 1);
for i = 1:length(Activations_9Groups)
    MuscleNames_Acts_9Groups{i, 1} = Activations_9Groups{i, 1}.Name;
end

% Load color data
load('Color_40.mat');

% Calculate normalized time
CT = MotionData.GenericCat.CycleTime.Average;
dt = CT / (size(MotionData.GenericCat.Angles.AngShou.Total, 1) - 1);
time = (0:size(MotionData.GenericCat.Angles.AngShou.Total, 1) - 1)' * dt;
Ntime = time / max(time);
Ndt = Ntime(2) - Ntime(1);

% Read data from ForeLimbMuscles_EMG_Vertical.xlsx
[~, sheet_name] = xlsfinfo('ForeLimbMuscles_EMG_Vertical.xlsx');
for k = 1:numel(sheet_name)
    data{k} = xlsread('ForeLimbMuscles_EMG_Vertical.xlsx', sheet_name{k});
end
data = data';

% Calculate duty factors
DF_ReSt = round(MotionData.GenericCat.DutyFactor.Average, 2); 
DF_ReSw = 1 - DF_ReSt;

% Define indices
E_S = 101;  % End of step
DF_S = 103; % Duty factor step

% Adjust EMG data based on duty factors
for i = 1:size(data, 1)
    for j = 1:size(data{i, 1}, 2)
        DF_Sw = round(data{i, 1}(DF_S, j) / 100, 2);
        DF_St = 1 - DF_Sw;
        Ndt_Sw = Ndt * (DF_ReSw / DF_Sw);
        Ndt_St = Ndt * (DF_ReSt / DF_St);
        Ntime_Sw = (0:Ndt_Sw:DF_ReSw)';
        Ntime_St = (DF_ReSw:Ndt_St:1)';
        Ntime_adj{i, 1}(:, j) = [Ntime_Sw; Ntime_St(2:end, 1)];

        % Spline fit
        p = 1;
        options = fitoptions('Method', 'Smooth', 'SmoothingParam', p);
        fitobject = fit(Ntime_adj{i, 1}(:, j), data{i, 1}(1:E_S, j), 'smoothingspline', options);
        EMG_adj{i, 1}(:, j) = fitobject(Ntime);
    end
end

% Calculate mean values of adjusted EMG data
for i = 1:size(EMG_adj, 1)
    EMG_adj_mean{i, 1} = mean(EMG_adj{i, 1}, 2);
end

% Indices for muscles similar to MuscleNames_EMGs from MuscleNames_Acts & MuscleNames_Acts_9Groups
Idx_EMGs = (1:length(MuscleNames_EMGs))';
Idx_Acts = [3; 7; 32; 34; 38; 37; 5];
Idx_Acts_9Groups = [3; 5; 4; 1; 9; 2; 4];

%% Plotting the data

figure(1)

grayColor = [.7 .7 .71];

% Plot for each muscle based on EMGs
subplot(7, 1, 1)
MN = 1; % based on EMGs
y1 = EMG_adj_mean{Idx_EMGs(MN), 1};
y2 = Activations{Idx_Acts(MN)}.a;
y3 = Activations_9Groups{Idx_Acts_9Groups(MN)}.a;
y4 = EMG_adj{Idx_EMGs(MN), 1};
hold on;

% Plot EMG data
yyaxis right
pp = plot(Ntime*100, y4, '-', 'Color', grayColor, 'LineWidth', 0.2);
p1 = plot(Ntime*100, y1, '-k', 'LineWidth', 1.5);
ylim([0 1])
ax = gca;
ax.YAxis(2).Color = [0 0 0];
ylabel('EMGs')

% Plot activation data
yyaxis left
p2 = plot(Ntime*100, y2, 'LineWidth', 1.5, 'Color', Color_40(Idx_Acts(MN), :));
hold on
p3 = plot(Ntime*100, y3, '-.', 'LineWidth', 1.5, 'Color', 'b');
mm = max([max(y2)/max(y1) max(y3)/max(y1)]);
ylim([0 0.4])
yticks([0 0.2 0.4])
ax.YAxis(1).Color = [0 0 0];
ylabel('Act')

% Mark the duty factor
xl = xline((1 - MotionData.GenericCat.DutyFactor.Average) * 100, ':');
xl.LineWidth = 2;

legend([p1 pp(1, 1) p2 p3], 'Mean EMG', 'All EMGs', 'Act from 40', 'Act from 9');
title(MuscleNames_EMGs(MN))
ytickformat('%.1f')
box on
R4 = corrcoef(EMG_adj_mean{Idx_EMGs(MN), 1}, Activations{Idx_Acts(MN)}.a);

% Repeat the plotting for each muscle
for MN = 2:7
    subplot(7, 1, MN)
    y1 = EMG_adj_mean{Idx_EMGs(MN), 1};
    y2 = Activations{Idx_Acts(MN)}.a;
    y3 = Activations_9Groups{Idx_Acts_9Groups(MN)}.a;
    y4 = EMG_adj{Idx_EMGs(MN), 1};
    hold on;

    % Plot EMG data
    yyaxis right
    pp = plot(Ntime*100, y4, '-', 'Color', grayColor, 'LineWidth', 0.2);
    p1 = plot(Ntime*100, y1, '-k', 'LineWidth', 1.5);
    ylim([0 1])
    ax = gca;
    ax.YAxis(2).Color = [0 0 0];
    ylabel('EMGs')

    % Plot activation data
    yyaxis left
    p2 = plot(Ntime*100, y2, 'LineWidth', 1.5, 'Color', Color_40(Idx_Acts(MN), :));
    hold on
    p3 = plot(Ntime*100, y3, '-.', 'LineWidth', 1.5, 'Color', 'b');
    mm = max([max(y2)/max(y1) max(y3)/max(y1)]);
    ylim([0 mm])
    yticks([0 mm*1/2 mm*2/2])
    ax.YAxis(1).Color = [0 0 0];
    ylabel('Act')

    % Mark the duty factor
    xl = xline((1 - MotionData.GenericCat.DutyFactor.Average) * 100, ':');
    xl.LineWidth = 2;

    title(MuscleNames_EMGs(MN))
    ytickformat('%.1f')
    box on
    R4 = corrcoef(EMG_adj_mean{Idx_EMGs(MN), 1}, Activations{Idx_Acts(MN)}.a);
end

% Adjust figure size and export
x0 = 50;
y0 = 50;
width = 400;
height = 1000;
set(gcf, 'position', [x0, y0, width, height])
exportgraphics(gcf, 'FigEMGs_IndividualMuscles_Vertical.pdf', 'ContentType', 'vector')
