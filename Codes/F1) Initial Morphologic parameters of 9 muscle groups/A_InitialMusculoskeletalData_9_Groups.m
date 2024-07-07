% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% This m-file generates initial morphological parameters for 9 muscle
% groups, incluing a and phi
% It includes ranges for joint radius based on individual muscles in each group.
% The ranges for the rest of the morphological parameters, including 'a' and 'phi', are computed from SolidWorks drawings.
% The generated data is saved in 'NineMuscleMorphParaInitial.mat'.

clear;
clc;

% Load the scaled muscle morphological parameters
ScaledMuscleMorphologicalParameters = load('ScaledMuscleMorphologicalParameters.mat');
ScaledMuscleMorphologicalParameters = ScaledMuscleMorphologicalParameters.ScaledMuscleMorphologicalParameters;

% Load the muscle clusters
MuscleCluster = load('MuscleCluster.mat');
MuscleCluster = MuscleCluster.MuscleCluster;

% Rows related to considered muscles
RTM = [1,1 ; 2,2; 5,5 ; 6,7; 8,8; 9,9; 10,11; 14,15; 16,17; 18,19; 20,21; ...
       30,31 ; 32,33; 34,34; 35,36; 37,38; 39,40; 41,42; 43,44; 45,46; 47,48; ...
       49,50; 51,52; 53,54; 55,56; 57,58; 59,60; 61,62; 63,64; 65,66; 67,68; ...
       69,69; 70,70; 71,71; 72,72; 73,73; 74,74; 75,75; 76,76; 77,77];

% Initialize muscle groups
NineMuscleMorphParaInitial = cell(9, 1);

%-----------Shoulder Protractors (Group 1)-------------------------------------------------
G = 1;
MM = RTM(MuscleCluster{G},:);  % MM i.e. muscle morphologies
MN = unique(sort(MM(:)));      % MN i.e. muscle number

for i = 1:size(MN, 1)
    Muscle_Group{G, 1}(i, :) = ScaledMuscleMorphologicalParameters{MN(i, 1), 1}.Parameters;
end

A = Muscle_Group{G, 1}(:, 5).R1; % Joint radius
B = A(~isnan(A));

R1 = mean(B);
R1min = min(B);
R1max = max(B);

NineMuscleMorphParaInitial{G, 1}.Name = 'Shoulder Protractors';
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters = ScaledMuscleMorphologicalParameters{72, 1}.Parameters;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.R1 = R1;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.MinR1 = R1min;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.MaxR1 = R1max;

%-----------Elbow Extensors (Group 2)-----------------------------------------------------
G = 2;
MM = RTM(MuscleCluster{G}, :);  % MM i.e. muscle morphologies
MN = unique(sort(MM(:)));       % MN i.e. muscle number

for i = 1:size(MN, 1)
    Muscle_Group{G, 1}(i, :) = ScaledMuscleMorphologicalParameters{MN(i, 1), 1}.Parameters;
end

A = Muscle_Group{G, 1}(:, 5).R1;
B = A(~isnan(A));

R1 = mean(B);
R1min = min(B);
R1max = max(B);

NineMuscleMorphParaInitial{G, 1}.Name = 'Elbow Extensors';
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters = ScaledMuscleMorphologicalParameters{74, 1}.Parameters;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.R1 = R1;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.MinR1 = R1min;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.MaxR1 = R1max;

%-----------Shoulder Protr-Elbow Flex (Group 3)--------------------------------------------
G = 3;
MM = RTM(MuscleCluster{G}, :);  % MM i.e. muscle morphologies
MN = unique(sort(MM(:)));       % MN i.e. muscle number

for i = 1:size(MN, 1)
    Muscle_Group{G, 1}(i, :) = ScaledMuscleMorphologicalParameters{MN(i, 1), 1}.Parameters;
end

NineMuscleMorphParaInitial{G, 1}.Name = 'Shoulder Protr-Elbow Flex';
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters = ScaledMuscleMorphologicalParameters{5, 1}.Parameters;

%-----------Elbow Flexors (Group 4)-------------------------------------------------------
G = 4;
MM = RTM(MuscleCluster{G}, :);  % MM i.e. muscle morphologies
MN = unique(sort(MM(:)));       % MN i.e. muscle number

for i = 1:size(MN, 1)
    Muscle_Group{G, 1}(i, :) = ScaledMuscleMorphologicalParameters{MN(i, 1), 1}.Parameters;
end

A = Muscle_Group{G, 1}(:, 5).R1OW;
B = A(~isnan(A));

R1 = mean(B);
R1min = min(B);
R1max = max(B);

NineMuscleMorphParaInitial{G, 1}.Name = 'Elbow Flexors';
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters = ScaledMuscleMorphologicalParameters{8, 1}.Parameters;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.R1 = R1;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.MinR1 = R1min;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.MaxR1 = R1max;

%-----------Elbow Flexor-Wrist Dorsiflexors (Group 5)-------------------------------------
G = 5;
MM = RTM(MuscleCluster{G}, :);  % MM i.e. muscle morphologies
MN = unique(sort(MM(:)));       % MN i.e. muscle number

for i = 1:size(MN, 1)
    Muscle_Group{G, 1}(i, :) = ScaledMuscleMorphologicalParameters{MN(i, 1), 1}.Parameters;
end

R1OW = (Muscle_Group{G, 1}(:, 5).R1OW(1) + Muscle_Group{G, 1}(:, 5).R1OW(3) + Muscle_Group{G, 1}(:, 5).R1OW(5) + ...
        Muscle_Group{G, 1}(:, 5).R1OW(7) + Muscle_Group{G, 1}(:, 5).R1OW(9)) / 5;

R1WI = (Muscle_Group{G, 1}(:, 6).R2OW(1) + Muscle_Group{G, 1}(:, 5).R1OW(4) + Muscle_Group{G, 1}(:, 5).R1OW(6) + ...
        Muscle_Group{G, 1}(:, 5).R1OW(8) + Muscle_Group{G, 1}(:, 5).R1OW(10)) / 5;

R1OWmin = min([Muscle_Group{G, 1}(:, 5).R1OW(1), Muscle_Group{G, 1}(:, 5).R1OW(3), Muscle_Group{G, 1}(:, 5).R1OW(5), ...
               Muscle_Group{G, 1}(:, 5).R1OW(7), Muscle_Group{G, 1}(:, 5).R1OW(9)]);

R1OWmax = max([Muscle_Group{G, 1}(:, 5).R1OW(1), Muscle_Group{G, 1}(:, 5).R1OW(3), Muscle_Group{G, 1}(:, 5).R1OW(5), ...
               Muscle_Group{G, 1}(:, 5).R1OW(7), Muscle_Group{G, 1}(:, 5).R1OW(9)]);

R1WImin = min([Muscle_Group{G, 1}(:, 6).R2OW(1), Muscle_Group{G, 1}(:, 5).R1OW(4), Muscle_Group{G, 1}(:, 5).R1OW(6), ...
               Muscle_Group{G, 1}(:, 5).R1OW(8), Muscle_Group{G, 1}(:, 5).R1OW(10)]);

R1WImax = max([Muscle_Group{G, 1}(:, 6).R2OW(1), Muscle_Group{G, 1}(:, 5).R1OW(4), Muscle_Group{G, 1}(:, 5).R1OW(6), ...
               Muscle_Group{G, 1}(:, 5).R1OW(8), Muscle_Group{G, 1}(:, 5).R1OW(10)]);

NineMuscleMorphParaInitial{G, 1}.Name = 'Elbow Flexor-Wrist Dorsiflexors';
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW = ScaledMuscleMorphologicalParameters{14, 1}.Parameters;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI = ScaledMuscleMorphologicalParameters{15, 1}.Parameters;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.R1OW = R1OW;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.R1WI = R1WI;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.MinR1OW = R1OWmin;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.MaxR1OW = R1OWmax;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.MinR1WI = R1WImin;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.MaxR1WI = R1WImax;

%-----------Wrist Dorsiflexors (Group 6)--------------------------------------------------
G = 6;
MM = RTM(MuscleCluster{G}, :);  % MM i.e. muscle morphologies
MN = unique(sort(MM(:)));       % MN i.e. muscle number

for i = 1:size(MN, 1)
    Muscle_Group{G, 1}(i, :) = ScaledMuscleMorphologicalParameters{MN(i, 1), 1}.Parameters;
end

A = Muscle_Group{G, 1}(:, 5).R1OW;
B = A(~isnan(A));

R1WI = mean(B);
R1WImin = min(B);
R1WImax = max(B);

NineMuscleMorphParaInitial{G, 1}.Name = 'Wrist Dorsiflexors';
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW = ScaledMuscleMorphologicalParameters{30, 1}.Parameters;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI = ScaledMuscleMorphologicalParameters{31, 1}.Parameters;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.R1WI = R1WI;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.MinR1WI = R1WImin;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.MaxR1WI = R1WImax;

%-----------Wrist Plantarflexors (Group 7)------------------------------------------------
G = 7;
MM = RTM(MuscleCluster{G}, :);  % MM i.e. muscle morphologies
MN = unique(sort(MM(:)));       % MN i.e. muscle number

for i = 1:size(MN, 1)
    Muscle_Group{G, 1}(i, :) = ScaledMuscleMorphologicalParameters{MN(i, 1), 1}.Parameters;
end

A = Muscle_Group{G, 1}(:, 5).R1OW;
B = A(~isnan(A));

R1WI = mean(B);
R1WImin = min(B);
R1WImax = max(B);

NineMuscleMorphParaInitial{G, 1}.Name = 'Wrist Plantarflexors';
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW = ScaledMuscleMorphologicalParameters{39, 1}.Parameters;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI = ScaledMuscleMorphologicalParameters{40, 1}.Parameters;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.R1WI = R1WI;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.MinR1WI = R1WImin;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.MaxR1WI = R1WImax;

%-----------Shoulder Retractors (Group 8)-------------------------------------------------
G = 8;
MM = RTM(MuscleCluster{G}, :);  % MM i.e. muscle morphologies
MN = unique(sort(MM(:)));       % MN i.e. muscle number

for i = 1:size(MN, 1)
    Muscle_Group{G, 1}(i, :) = ScaledMuscleMorphologicalParameters{MN(i, 1), 1}.Parameters;
end

A = Muscle_Group{G, 1}(:, 5).R1;
B = A(~isnan(A));

R1 = mean(B);
R1min = min(B);
R1max = max(B);

NineMuscleMorphParaInitial{G, 1}.Name = 'Shoulder Retractors';
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters = ScaledMuscleMorphologicalParameters{73, 1}.Parameters;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.R1 = R1;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.MinR1 = R1min;
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.MaxR1 = R1max;

%-----------Shoulder Retr-Elbow Ext (Group 9)---------------------------------------------
G = 9;
MM = RTM(MuscleCluster{G}, :);  % MM i.e. muscle morphologies
MN = unique(sort(MM(:)));       % MN i.e. muscle number

for i = 1:size(MN, 1)
    Muscle_Group{G, 1}(i, :) = ScaledMuscleMorphologicalParameters{MN(i, 1), 1}.Parameters;
end

NineMuscleMorphParaInitial{G, 1}.Name = 'Shoulder Retr-Elbow Ext';
NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters = ScaledMuscleMorphologicalParameters{75, 1}.Parameters;

% Save the generated muscle morphological parameters to a .mat file
save('NineMuscleMorphParaInitial.mat', 'NineMuscleMorphParaInitial');
