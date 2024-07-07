% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% This script loads data from specified Excel sheets including musculoskeletal data, 
% extracting muscle origin and insertion points, joint centers, and other relevant anatomical data.
%
% The script fits a plane to the muscle attachment points using Principal 
% Component Analysis (PCA) and projects various anatomical landmarks onto this plane for accurate 
% spatial representation.
%
% The final part of the script involves plotting the muscles and segments in a 3D figure. 
% This includes visualizing the fitted plane, annotating the muscles and their attachment points, 
% and highlighting key anatomical features. The result is a detailed and informative 3D representation 
% of the musculoskeletal system, providing valuable insights for further analysis and interpretation.



clear; % Clear workspace variables
clc;   % Clear command window

% Load data from Excel sheets
[numMS, txtMS, allMS] = xlsread("Original Musculoskeletal Data.xlsx", "Muscle Segments");
[numJC, ~, ~] = xlsread("Original Musculoskeletal Data.xlsx", "Joint Centers");
[numOD, txtOD, ~] = xlsread("Original Musculoskeletal Data.xlsx", "Original Data");

% Extract muscle origin and insertion points
Origins = numMS(:, 1:3);
Insertions = numMS(:, 5:7);

% Joint Centers
JC = numJC;

% Process joint axes data and comply with the right-hand rule
JAxes = numOD(4:11, 4:end); 
JAxes([4, 6], :) = -JAxes([4, 6], :);


% Find the column numbers by searching the headers
[~, origin_col] = find(cellfun(@(x) strcmp(x, 'Muscle Origin|Via-Point Number'), allMS));
[~, insertion_col] = find(cellfun(@(x) strcmp(x, 'Muscle Via-Point|Insertion Number'), allMS));

% Extract data from the corresponding columns
Origin_nums = cell2mat(allMS(2:end, origin_col));
Insertion_nums = cell2mat(allMS(2:end, insertion_col));

% Find rows with similar numbers in both columns
[common_values, origin_rows, insertion_rows] = intersect(Origin_nums, Insertion_nums);

% The rows where muscle points are repeated (muscles with via(retinacula)-points)
RBM = [insertion_rows, origin_rows];



% Fit a plane to the muscle attachment points using PCA
%---------- Muscle attachment points ------------------------------------------------------
MAP = [Origins; Insertions];

% Remove duplicate rows from MAP
[MAP, ~, ic] = unique(MAP, 'rows');

% Perform PCA on the unique muscle attachment points
[coeff_MAP, score_MAP] = pca(MAP);

% Extract the normal vector to the fitted plane
normal_MAP = coeff_MAP(:, 3);

% Map original points to their corresponding PCA scores
score_MAP2 = score_MAP(ic, :);

% Calculate the mean of the unique points
meanMAP = mean(MAP, 1);

% Project original points onto the fitted plane
MAPfit = repmat(meanMAP, size(score_MAP2, 1), 1) + score_MAP2(:, 1:2) * coeff_MAP(:, 1:2)';

% Separate the projected origins and insertions
SO = size(Origins, 1);
Origins_2D = MAPfit(1:SO, :);
Insertions_2D = MAPfit(SO + 1:end, :);


% Extract bone landmark positions
boneLandmarks = {'flmcp', 'fmmcp', 'ftb', 'hgt', 'hle', 'hme', 'notrusp', 'rh', 'rsp', 'rus', 'sa', 'sca', 'svs', 'uop', 'usp'};
landmarkIndices = cellfun(@(x) find(contains(txtOD, x)), boneLandmarks, 'UniformOutput', false);

% Apply corrections for specific landmark indices
landmarkIndices{10} = 133; % 'rus'
landmarkIndices{11} = landmarkIndices{11}(end-1); % 'sa'
landmarkIndices{12} = landmarkIndices{12}(end-1); % 'sca'
landmarkIndices{15} = landmarkIndices{15}(end); % 'usp'

% Store landmarks in variables
landmarks = cellfun(@(x) numOD(x - 1, 1:3), landmarkIndices, 'UniformOutput', false);
[flmcp, fmmcp, ftb, hgt, hle, hme, notrusp, rh, rsp, rus, sa, sca, svs, uop, usp] = deal(landmarks{:});

% Calculate mean positions for specific landmarks (proximal scapula & distal foot)
Mean_svs_sca = mean([sca; svs]);
Mean_flmcp_fmmcp = mean([flmcp; fmmcp]);

% Project mean positions onto the fitted plane
projectOntoPlane = @(C, P, n) C - dot(C - P, n) * n;
PMs = projectOntoPlane(Mean_svs_sca, MAPfit(1, :), normal_MAP');
PMf = projectOntoPlane(Mean_flmcp_fmmcp, MAPfit(1, :), normal_MAP');

% Find intersections of flexion-extension axes with the sagittal plane
findIntersection = @(P0, P1, V0, n) P0 + (dot(n, (V0 - P0)) / dot(n, (P1 - P0))) * (P1 - P0);

SIP = findIntersection(JC(1,:) + JAxes(1,:), JC(1,:) - JAxes(1,:), MAPfit(1,:), normal_MAP);
EIP = findIntersection(JC(2,:) + JAxes(4,:), JC(2,:) - JAxes(4,:), MAPfit(1,:), normal_MAP);
WIP = findIntersection(JC(3,:) + JAxes(6,:), JC(3,:) - JAxes(6,:), MAPfit(1,:), normal_MAP);



%% Plot muscles and segments
MaS = 4;
textsize = 12;

figure(1);
hold on;

% Plot joints and segments
jointColors = {[1, 0, 0], [0, 1, 0], [0, 0, 1]};
h = gobjects(0);

for i = 1:min(length(JC), 3)
    color = jointColors{i};
    s = scatter3(JC(i, 1), JC(i, 2), JC(i, 3), 'MarkerFaceColor', color, 'MarkerEdgeColor', color);
    h(i) = s;
    alpha(s, 0.7);
    hold on;
end

h(i+1) = plot3(JC(1:2, 1), JC(1:2, 2), JC(1:2, 3), 'LineWidth', 4, 'Color', [0 0 0 0.6]);
h(i+2) = plot3(JC(2:3, 1), JC(2:3, 2), JC(2:3, 3), 'LineWidth', 4, 'Color', [0 0 0 0.6]);
hold on;

% Plot scapula and paw segments
Scapula_Seg = plot3([Mean_svs_sca(1,1) JC(1,1)], [Mean_svs_sca(1,2) JC(1,2)], [Mean_svs_sca(1,3) JC(1,3)], 'LineWidth', 4);
Scapula_Seg.Color = [[0 0 0] 0.6];
Paw_Seg = plot3([Mean_flmcp_fmmcp(1,1) JC(3,1)], [Mean_flmcp_fmmcp(1,2) JC(3,2)], [Mean_flmcp_fmmcp(1,3) JC(3,3)], 'LineWidth', 4);
Paw_Seg.Color = [[0 0 0] 0.6];
hold on;

% Plot muscles
colors = [
    0.8147, 0.9058, 0.1270; 0.9134, 0.6324, 0.0975; 0.2785, 0.5469, 0.9575;
    0, 0, 0; 0.9649, 0.1576, 0.9706; 0.9572, 0.4854, 0.8003; 0, 0, 0;
    0.1419, 0.4218, 0.9157; 0.7922, 0.9595, 0.6557; 0.0357, 0.8491, 0.9340;
    0, 0, 0; 0.6787, 0.7577, 0.7431; 0, 0, 0; 0.3922, 0.6555, 0.1712;
    0, 0, 0; 0.7060, 0.0318, 0.2769; 0, 0, 0; 0.0462, 0.0971, 0.8235;
    0, 0, 0; 0.6948, 0.3171, 0.9502; 0, 0, 0; 0.0344, 0.4387, 0.3816;
    0, 0, 0; 0.7655, 0.7952, 0.1869; 0, 0, 0; 0.4898, 0.4456, 0.6463;
    0, 0, 0; 0.7094, 0.7547, 0.2760; 0, 0, 0; 0.6797, 0.6551, 0.1626;
    0, 0, 0; 0.1190, 0.4984, 0.9597; 0, 0, 0; 0.3404, 0.5853, 0.2238;
    0.7513, 0.2551, 0.5060; 0, 0, 0; 0.6991, 0.8909, 0.9593; 0, 0, 0;
    0.5472, 0.1386, 0.1493; 0, 0, 0; 0.2575, 0.8407, 0.2543; 0, 0, 0;
    0.8143, 0.2435, 0.9293; 0, 0, 0; 0.3500, 0.1966, 0.2511; 0, 0, 0;
    0.6160, 0.4733, 0.3517; 0, 0, 0; 0.8308, 0.5853, 0.5497; 0, 0, 0;
    0.9172, 0.2858, 0.7572; 0, 0, 0; 0.7537, 0.3804, 0.5678; 0, 0, 0;
    0.0759, 0.0539, 0.5308; 0, 0, 0; 0.7792, 0.9340, 0.1299; 0, 0, 0;
    0.5688, 0.4694, 0.0119; 0, 0, 0; 0.3371, 0.1622, 0.7943; 0, 0, 0;
    0.3112, 0.5285, 0.1656; 0, 0, 0; 0.6020, 0.2630, 0.6541; 0, 0, 0;
    0.6892, 0.7482, 0.4505; 0, 0, 0; 0.0838, 0.2290, 0.9133; 0.1524, 0.8258, 0.5383;
    0.9961, 0.0782, 0.4427; 0.1067, 0.9619, 0.0046; 0.7749, 0.8173, 0.8687;
    0.0844, 0.3998, 0.2599; 0.8001, 0.4314, 0.9106; 0.1818, 0.2638, 0.1455;
    0.1361, 0.8693, 0.5797
];

k = 0; % Initialize row index for muscles
nn = i + 2;

for i = 1:length(Insertions)
    k = k + 1;
    % Plot muscle as a line between origin and insertion points
    h(nn + k) = line([Origins(k, 1), Insertions(k, 1)], [Origins(k, 2), Insertions(k, 2)], [Origins(k, 3), Insertions(k, 3)], 'Color', colors(k, :), 'LineWidth', 1.5);
    hold on;

    % Plot muscle labels
    text(Origins(k, 1), Origins(k, 2), Origins(k, 3), num2str(Origin_nums(k,1)), 'FontSize', textsize, 'FontWeight', 'bold');
    text(Insertions(k, 1), Insertions(k, 2), Insertions(k, 3), num2str(Insertion_nums(k,1)), 'FontSize', textsize, 'FontWeight', 'bold');
    hold on;

    % Plot origin and insertion points
    plot3(Origins(k, 1), Origins(k, 2), Origins(k, 3), 'o', 'LineWidth', 1, 'MarkerEdgeColor', colors(k, :), 'MarkerFaceColor', 'none', 'MarkerSize', MaS);
    plot3(Insertions(k, 1), Insertions(k, 2), Insertions(k, 3), 'o', 'LineWidth', 1, 'MarkerEdgeColor', colors(k, :), 'MarkerFaceColor', 'none', 'MarkerSize', MaS);
    hold on;

    % Check for redundant muscle-bone connections and plot them
    if ismember(k, RBM(:, 1))
        k = k + 1;
        line([Origins(k, 1), Insertions(k, 1)], [Origins(k, 2), Insertions(k, 2)], [Origins(k, 3), Insertions(k, 3)], 'Color', colors(k - 1, :), 'LineWidth', 1.5);
        hold on;

        % Plot labels for redundant connections
        text(Origins(k, 1), Origins(k, 2), Origins(k, 3), num2str(Origin_nums(k,1)), 'FontSize', textsize, 'FontWeight', 'bold');
        text(Insertions(k, 1), Insertions(k, 2), Insertions(k, 3), num2str(Insertion_nums(k,1)), 'FontSize', textsize, 'FontWeight', 'bold');
        hold on;

        % Plot origin and insertion points for redundant connections
        plot3(Origins(k, 1), Origins(k, 2), Origins(k, 3), 'o', 'LineWidth', 1, 'MarkerEdgeColor', colors(k - 1, :), 'MarkerFaceColor', 'none', 'MarkerSize', MaS);
        plot3(Insertions(k, 1), Insertions(k, 2), Insertions(k, 3), 'o', 'LineWidth', 1, 'MarkerEdgeColor', colors(k - 1, :), 'MarkerFaceColor', 'none', 'MarkerSize', MaS);
        hold on;
    end

    % Break the loop if all insertions are processed
    if k == length(Insertions)
        break;
    end
end

% Create mesh grid for the plane fit to muscle attachment points
[xgrid_MAP, ygrid_MAP] = meshgrid(linspace(min(MAP(:, 1)), max(MAP(:, 1)), 20), linspace(min(MAP(:, 2)), max(MAP(:, 2)), 20));
zgrid_MAP = (1 / normal_MAP(3)) * (meanMAP * normal_MAP - (xgrid_MAP .* normal_MAP(1) + ygrid_MAP .* normal_MAP(2)));
h(end + 1) = mesh(xgrid_MAP, ygrid_MAP, zgrid_MAP, 'EdgeColor', 'none', 'FaceAlpha', .2, 'FaceColor', [0, 0, 1]);

% Add legend
legend([h(1:3), h(6:8), h(10), h(11), h(13), h(14), h(15), h(17), h(19), h(21), h(23), h(25), h(27), h(29), h(31), h(33), h(35), h(37), ...
        h(39), h(40), h(42), h(44), h(46), h(48), h(50), h(52), h(54), h(56), h(58), h(60), h(62), h(64), h(66), h(68), h(70), h(72), h(74), ...
        h(75), h(76), h(77), h(78), h(79), h(80), h(81), h(82), h(83)], ...
       'Shoulder', 'Elbow', 'Wrist', ...
       'Acromiodeltoideus (1->2)', 'Anconeus (3->4)', 'Abductor pollicis longus (5->6->7)', ...
       'Biceps brachii (8->9)', 'Brachioradialis (10->11->12)', 'Brachialis (13->14)', ...
       'Coracobrachialis (15->16)', 'Extensor carpi radialis (17->18->19)', 'Extensor Carpi Ulnaris (20->21->22)', ...
       'Extensor digitorum communis 2 (23->24->25)', 'Extensor digitorum communis 3 (26->27->28)', 'Extensor digitorum communis 4 (29->30->31)', ...
       'Extensor digitorum communis 5 (32->33->34)', 'Extensor digitorum lateralis 2 (35->36->37)', 'Extensor digitorum lateralis 3 (38->39->40)', ...
       'Extensor digitorum lateralis 4 (41->42->43)', 'Extensor digitorum lateralis 5 (44->45->46)', 'Extensor pollicis longus 1 (47->48->49)', ...
       'Extensor pollicis longus 2 (50->51->52)', 'Epitrochlearis (53->54)', 'Flexor Carpi radialis (55->56->57)', 'Flexor Carpi Ulnaris (58->59->60)', ...
       'Flexor digitorum profundus 1 (61->62->63)', 'Flexor digitorum profundus 2 (64->65->66)', 'Flexor digitorum profundus 3 (67->68->69)', ...
       'Flexor digitorum profundus 4 (70->71->72)', 'Flexor digitorum profundus 5 (73->74->75)', 'Flexor Digitorum Superficialis 2 (76->77->78)', ...
       'Flexor Digitorum Superficialis 3 (79->80->81)', 'Flexor Digitorum Superficialis 4 (82->83->84)', 'Flexor Digitorum Superficialis 5 (85->86->87)', ...
       'Infraspinatus (88->89->90)', 'Palmaris Longus 1 (91->92->93)', 'Palmaris Longus 2 (94->95->96)', 'Palmaris Longus 3 (97->98->99)', ...
       'Palmaris Longus 4 (100->101->102)', 'Palmaris Longus 5 (103->104->105)', 'Pronator Teres (106->107)', 'Spinodeltoideus (108->109)', ...
       'Supraspinatus (110->111)', 'Subscapularis (112->113)', 'Teres Major (114->115)', 'Triceps Brachii Lateralis (116->117)', ...
       'Triceps Brachii Long (118->119)', 'Triceps brachii medial (120->121)', 'Teres Minor (122->123)', ...
       'Plane fitted to muscle attachment points');

% Set plot properties
box on;
grid on;
axis equal;
xlabel('x, m');
ylabel('y, m');
zlabel('z, m');
view(30, 20); % Perspective view
title('Perspective View of Cat Forelimb 3D Musculoskeletal System');

% Adjust axis limits
xlim([min(MAP(:, 1)), max(MAP(:, 1))]);
ylim([min(MAP(:, 2)), max(MAP(:, 2))]);
zlim([min(MAP(:, 3)), max(MAP(:, 3)) + 0.01]);
