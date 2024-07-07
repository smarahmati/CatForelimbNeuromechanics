% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, School of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% This script computes a spinal map of the activities of motor neurons and sensory neurons (Ia, Ib, II) of forelimb muscles during the walking cycle,
% based on the proportional distribution of motor pools from the literature.


clear;  % Clear workspace
clc;    % Clear command window

% Load necessary data
load('Activations.mat');
load('RIa.mat');
load('RIb.mat');
load('RII.mat');

% Load motor pool distribution table
T = readtable('Proportional distribution of the motor pools.xlsx');

% Initialize distribution matrices for different motor pools
C5 = T.Pro_C5;
C6 = T.Pro_C6;
C7 = T.Pro_C7;
C8 = T.Pro_C8;
T1 = T.Pro_T1;

% Get the number of time points from Activations
numTimePoints = length(Activations{1, 1}.a);

% Initialize matrices to store results
Mat_C5_Act = zeros(numTimePoints, 40);
Mat_C6_Act = zeros(numTimePoints, 40);
Mat_C7_Act = zeros(numTimePoints, 40);
Mat_C8_Act = zeros(numTimePoints, 40);
Mat_T1_Act = zeros(numTimePoints, 40);

Mat_C5_RIa = zeros(numTimePoints, 40);
Mat_C6_RIa = zeros(numTimePoints, 40);
Mat_C7_RIa = zeros(numTimePoints, 40);
Mat_C8_RIa = zeros(numTimePoints, 40);
Mat_T1_RIa = zeros(numTimePoints, 40);

Mat_C5_RIb = zeros(numTimePoints, 40);
Mat_C6_RIb = zeros(numTimePoints, 40);
Mat_C7_RIb = zeros(numTimePoints, 40);
Mat_C8_RIb = zeros(numTimePoints, 40);
Mat_T1_RIb = zeros(numTimePoints, 40);

Mat_C5_RII = zeros(numTimePoints, 40);
Mat_C6_RII = zeros(numTimePoints, 40);
Mat_C7_RII = zeros(numTimePoints, 40);
Mat_C8_RII = zeros(numTimePoints, 40);
Mat_T1_RII = zeros(numTimePoints, 40);

% Process data and compute sensory feedback
for i = 1:40
    % Ensure compatibility of dimensions
    if size(Activations{i, 1}.a, 1) == numTimePoints
        % Activation data
        Mat_C5_Act(:, i) = C5(i) * Activations{i, 1}.a;
        Mat_C6_Act(:, i) = C6(i) * Activations{i, 1}.a;
        Mat_C7_Act(:, i) = C7(i) * Activations{i, 1}.a;
        Mat_C8_Act(:, i) = C8(i) * Activations{i, 1}.a;
        Mat_T1_Act(:, i) = T1(i) * Activations{i, 1}.a;

        % Ia afferent activity
        Mat_C5_RIa(:, i) = C5(i) * RIa{i, 1}.SensoryFeedback;
        Mat_C6_RIa(:, i) = C6(i) * RIa{i, 1}.SensoryFeedback;
        Mat_C7_RIa(:, i) = C7(i) * RIa{i, 1}.SensoryFeedback;
        Mat_C8_RIa(:, i) = C8(i) * RIa{i, 1}.SensoryFeedback;
        Mat_T1_RIa(:, i) = T1(i) * RIa{i, 1}.SensoryFeedback;

        % Ib afferent activity
        Mat_C5_RIb(:, i) = C5(i) * RIb{i, 1}.SensoryFeedback;
        Mat_C6_RIb(:, i) = C6(i) * RIb{i, 1}.SensoryFeedback;
        Mat_C7_RIb(:, i) = C7(i) * RIb{i, 1}.SensoryFeedback;
        Mat_C8_RIb(:, i) = C8(i) * RIb{i, 1}.SensoryFeedback;
        Mat_T1_RIb(:, i) = T1(i) * RIb{i, 1}.SensoryFeedback;

        % II afferent activity
        Mat_C5_RII(:, i) = C5(i) * RII{i, 1}.SensoryFeedback;
        Mat_C6_RII(:, i) = C6(i) * RII{i, 1}.SensoryFeedback;
        Mat_C7_RII(:, i) = C7(i) * RII{i, 1}.SensoryFeedback;
        Mat_C8_RII(:, i) = C8(i) * RII{i, 1}.SensoryFeedback;
        Mat_T1_RII(:, i) = T1(i) * RII{i, 1}.SensoryFeedback;
    else
        error('Mismatch in the number of time points for muscle %d', i);
    end
end

% Normalize and aggregate activation data
Map_C5_Act = sum(Mat_C5_Act, 2) / sum(C5);
Map_C6_Act = sum(Mat_C6_Act, 2) / sum(C6);
Map_C7_Act = sum(Mat_C7_Act, 2) / sum(C7);
Map_C8_Act = sum(Mat_C8_Act, 2) / sum(C8);
Map_T1_Act = sum(Mat_T1_Act, 2) / sum(T1);
Map_Act = [Map_C5_Act; Map_C6_Act; Map_C7_Act; Map_C8_Act; Map_T1_Act];
Min = min(Map_Act);
Map_Act = Map_Act - Min;

% Normalize and aggregate Ia afferent activity data
Map_C5_RIa = sum(Mat_C5_RIa, 2) / sum(C5);
Map_C6_RIa = sum(Mat_C6_RIa, 2) / sum(C6);
Map_C7_RIa = sum(Mat_C7_RIa, 2) / sum(C7);
Map_C8_RIa = sum(Mat_C8_RIa, 2) / sum(C8);
Map_T1_RIa = sum(Mat_T1_RIa, 2) / sum(T1);
Map_RIa = [Map_C5_RIa; Map_C6_RIa; Map_C7_RIa; Map_C8_RIa; Map_T1_RIa];
Min = min(Map_RIa);
Map_RIa = Map_RIa - Min;

% Normalize and aggregate Ib afferent activity data
Map_C5_RIb = sum(Mat_C5_RIb, 2) / sum(C5);
Map_C6_RIb = sum(Mat_C6_RIb, 2) / sum(C6);
Map_C7_RIb = sum(Mat_C7_RIb, 2) / sum(C7);
Map_C8_RIb = sum(Mat_C8_RIb, 2) / sum(C8);
Map_T1_RIb = sum(Mat_T1_RIb, 2) / sum(T1);
Map_RIb = [Map_C5_RIb; Map_C6_RIb; Map_C7_RIb; Map_C8_RIb; Map_T1_RIb];
Min = min(Map_RIb);
Map_RIb = Map_RIb - Min;

% Normalize and aggregate II afferent activity data
Map_C5_RII = sum(Mat_C5_RII, 2) / sum(C5);
Map_C6_RII = sum(Mat_C6_RII, 2) / sum(C6);
Map_C7_RII = sum(Mat_C7_RII, 2) / sum(C7);
Map_C8_RII = sum(Mat_C8_RII, 2) / sum(C8);
Map_T1_RII = sum(Mat_T1_RII, 2) / sum(T1);
Map_RII = [Map_C5_RII; Map_C6_RII; Map_C7_RII; Map_C8_RII; Map_T1_RII];
Min = min(Map_RII);
Map_RII = Map_RII - Min;

% Generate color maps for plotting
n = length(Map_Act);
color_Act = jet(n);
[~, Idx_Act] = sort(Map_Act);
[~, Idx2_Act] = sort(Idx_Act);
color_Act = color_Act(Idx2_Act, :);

color_RIa = jet(n);
[~, Idx_RIa] = sort(Map_RIa);
[~, Idx2_RIa] = sort(Idx_RIa);
color_RIa = color_RIa(Idx2_RIa, :);

color_RIb = jet(n);
[~, Idx_RIb] = sort(Map_RIb);
[~, Idx2_RIb] = sort(Idx_RIb);
color_RIb = color_RIb(Idx2_RIb, :);

color_RII = jet(n);
[~, Idx_RII] = sort(Map_RII);
[~, Idx2_RII] = sort(Idx_RII);
color_RII = color_RII(Idx2_RII, :);

% Plot the results
figure(1)

% Plot activation data
subplot(4,1,1)
plotFeedback(Map_Act, color_Act, 'Act', true);

% Plot Ia afferent activity data
subplot(4,1,2)
plotFeedback(Map_RIa, color_RIa, 'RIa', false);

% Plot II afferent activity data
subplot(4,1,3)
plotFeedback(Map_RII, color_RII, 'RII', false);

% Plot Ib afferent activity data
subplot(4,1,4)
plotFeedback(Map_RIb, color_RIb, 'RIb', false);


% Adjust figure size and save
x0 = 50;
y0 = 50;
width = 400;
height = 800;
set(gcf, 'position', [x0, y0, width, height]);
exportgraphics(gcf, 'Fig_MapSensoryFeedback.pdf', 'ContentType', 'vector');


% Function to plot feedback data
function plotFeedback(Map, colorMap, titleText, useTwoDecimalPlaces)
    y2 = [20; 25; 25; 20];
    for k = 0:4
        for i = 1:101
            x2 = [i-1; i-1; i; i];
            v = [x2, y2 - k*5];
            patch('Faces',1:4,'Vertices',v,'FaceColor',colorMap(i + k*101, :),'EdgeColor', 'none')
            hold on
        end
    end
    xlim([0 100])
    ylim([0 25])
    set(gca, 'YTick', [])
    colormap('jet');
    cb = colorbar;
    if useTwoDecimalPlaces
        set(cb, 'ticks', linspace(0, 1, 5), 'ticklabels', num2str(linspace(min(Map), max(Map), 5)', '%.2f'));
    else
        set(cb, 'ticks', linspace(0, 1, 5), 'ticklabels', num2str(round(linspace(min(Map), max(Map), 5))'));
    end
    set(gca, 'XTick', [0 20 40 60 80 100]);
    y_labels = {'T1', 'C8', 'C7', 'C6', 'C5'};
    set(gca, 'YTick', [2.5 7.5 12.5 17.5 22.5], 'YTickLabel', y_labels);
    xlabel('Normalized cycle time, %');
    title(titleText);
end

