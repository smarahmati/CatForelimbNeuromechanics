% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% This script performs k-means clustering on maximum muscle moments and compares it with function-based grouping. 
% It consolidates individual muscles into a single group, except for specified separate muscles, and visualizes the clustering results. 
% The script generates the 'MuscleCluster' cell array, which shows the cluster of muscles in each cell.


clear;
clc;

% Load the MaxMuscleForceMoment data
MaxMuscleForceMoment = load('MaxMuscleForceMoment.mat');
MaxMuscleForceMoment = MaxMuscleForceMoment.MaxMuscleForceMoment;

% Initialize variables to store maximum, minimum, and average moments
for i = 1:length(MaxMuscleForceMoment)
    signShoulder = sign(MaxMuscleForceMoment{i, 1}.Moment.Shoulder);
    signElbow = sign(MaxMuscleForceMoment{i, 1}.Moment.Elbow);
    signWrist = sign(MaxMuscleForceMoment{i, 1}.Moment.Wrist);

    Ave_Muscle_Moment(i, 1) = mean(MaxMuscleForceMoment{i, 1}.Moment.Shoulder);
    Ave_Muscle_Moment(i, 2) = mean(MaxMuscleForceMoment{i, 1}.Moment.Elbow);
    Ave_Muscle_Moment(i, 3) = mean(MaxMuscleForceMoment{i, 1}.Moment.Wrist);
end

X = Ave_Muscle_Moment;

% The following function call creates 5 to 25 clusters using
% the k-means clustering, and calculates the silhouette value for each
% clustering scheme.
clustev = evalclusters(X, 'kmeans', 'silhouette', 'Distance', 'correlation', 'KList', 5:25);

% The output variable, clustev, contains detailed information about the evaluation including the optimum number of clusters.
kbest = clustev.OptimalK;

% Perform k-means clustering using the initial centroids
[idx, C, sumd, D] = kmeans(X, kbest, 'Distance', 'correlation');

% Grouping muscles based on the clustering results
eval = idx;
for i = 1:max(eval)
    k = find(eval == i);
    Cluster_Kmean{i, 1} = k;
end

for i = 1:size(Cluster_Kmean, 1)
    Min_Cluster_Kmean(i, 1) = min(Cluster_Kmean{i, 1});
end

[B, I] = sort(Min_Cluster_Kmean);
for i = 1:size(I, 1)
    Cluster_Kmean_Sort{i, 1} = Cluster_Kmean{I(i), 1};
end

% The muscles can be clustered based on their function
Cluster_Function{1, 1} = [1; 6; 26; 34; 35; 40];
Cluster_Function{2, 1} = [2; 14; 37; 39];
Cluster_Function{3, 1} = [3];
Cluster_Function{4, 1} = [4; 5; 32];
Cluster_Function{5, 1} = [7; 8; 9; 10; 11];
Cluster_Function{6, 1} = [12; 13];
Cluster_Function{7, 1} = [15; 16; 17; 18; 19; 20; 21; 22; 23; 24; 25; 27; 28; 29; 30; 31];
Cluster_Function{8, 1} = [33; 36];
Cluster_Function{9, 1} = [38];

% Identify individual muscles in k-means clustering
individualMuscles_kmeans = [];
for i = 1:length(Cluster_Kmean_Sort)
    if length(Cluster_Kmean_Sort{i}) == 1
        individualMuscles_kmeans = [individualMuscles_kmeans; Cluster_Kmean_Sort{i}];
    end
end

% Identify individual muscles in function-based clustering
individualMuscles_function = [];
for i = 1:length(Cluster_Function)
    if length(Cluster_Function{i}) == 1
        individualMuscles_function = [individualMuscles_function; Cluster_Function{i}];
    end
end

% Find common individual muscles between the two methods
separateMuscles = intersect(individualMuscles_kmeans, individualMuscles_function);

% Create a new grouping based on the consolidation
newCluster_Kmean_Sort = Cluster_Kmean_Sort;

% Consolidate individual muscles into one group, except for the specified separate muscles
consolidatedGroup = [];
for i = 1:length(individualMuscles_kmeans)
    if ~ismember(individualMuscles_kmeans(i), separateMuscles)
        consolidatedGroup = [consolidatedGroup; individualMuscles_kmeans(i)];
    end
end

% Remove the individual muscles that are being consolidated
for i = length(newCluster_Kmean_Sort):-1:1
    if length(newCluster_Kmean_Sort{i}) == 1 && ~ismember(newCluster_Kmean_Sort{i}, separateMuscles)
        newCluster_Kmean_Sort(i) = [];
    end
end

% Add the consolidated group to the clustering result
if ~isempty(consolidatedGroup)
    newCluster_Kmean_Sort{end+1} = consolidatedGroup;
end

% Sort Cluster_Kmean based on similarity to Cluster_Function
function_similarities = zeros(length(newCluster_Kmean_Sort), length(Cluster_Function));

for i = 1:length(newCluster_Kmean_Sort)
    for j = 1:length(Cluster_Function)
        % Compute the Jaccard similarity coefficient
        function_similarities(i, j) = numel(intersect(newCluster_Kmean_Sort{i}, Cluster_Function{j})) / ...
                                      numel(union(newCluster_Kmean_Sort{i}, Cluster_Function{j}));
    end
end

[~, max_similarity_indices] = max(function_similarities, [], 2);
sorted_indices = [];
for j = 1:length(Cluster_Function)
    sorted_indices = [sorted_indices; find(max_similarity_indices == j)];
end

Cluster_Kmean_SortedByFunction = newCluster_Kmean_Sort(sorted_indices);

MuscleCluster = Cluster_Kmean_SortedByFunction;

% Plot the clustering result
figure(1)
color = eval;
scatter3(X(:, 1), X(:, 2), X(:, 3), 50, color, 'filled')
hold on

% Add labels to specific clusters
for i = 1:length(Cluster_Function)
    text(X(Cluster_Function{i, 1}, 1), X(Cluster_Function{i, 1}, 2), X(Cluster_Function{i, 1}, 3), ['  ', num2str(i)])
end

axis equal
xlabel('Mean of Maximum Moment at Shoulder')
ylabel('Mean of Maximum Moment at Elbow')
zlabel('Mean of Maximum Moment at Wrist')
title('Result of Clustering')

% Plot group of moments
figure(2)
hold on
legend_entries = cell(size(Cluster_Function, 1), 1);
for k = 1:size(Cluster_Function, 1)
    GroupedMoment{k} = MaxMuscleForceMoment{1, 1}.Moment.Shoulder * 0;  % Initialize size of grouped moment

    for i = 1:size(Cluster_Function{k, 1}, 1)
        GroupedMoment_Shoulder = MaxMuscleForceMoment{Cluster_Function{k, 1}(i), 1}.Moment.Shoulder;
        GroupedMoment_Elbow = MaxMuscleForceMoment{Cluster_Function{k, 1}(i), 1}.Moment.Elbow;
        GroupedMoment_Wrist = MaxMuscleForceMoment{Cluster_Function{k, 1}(i), 1}.Moment.Wrist;

        GroupedMoment{k} = GroupedMoment_Shoulder + GroupedMoment_Elbow + GroupedMoment_Wrist + GroupedMoment{k};
    end
    plot(GroupedMoment{k})
    legend_entries{k} = sprintf('Group %d', k); % Create legend entries
end

xlabel('Time Step')
ylabel('Maximum moment, Nm')
legend(legend_entries, 'Location', 'best') % Add legend to the plot
title('Maximum moment generated by each group of muscles')

% Save the clustering result
save('MuscleCluster.mat', 'MuscleCluster');

% Save muscle groups as a table
for i = 1:size(Cluster_Function{1}, 1)
    Name_G1{i, 1} = MaxMuscleForceMoment{Cluster_Function{1}(i), 1}.Name;
end

for i = 1size(Cluster_Function{2}, 1)
    Name_G2{i, 1} = MaxMuscleForceMoment{Cluster_Function{2}(i), 1}.Name;
end

for i = 1size(Cluster_Function{3}, 1)
    Name_G3{i, 1} = MaxMuscleForceMoment{Cluster_Function{3}(i), 1}.Name;
end

for i = 1size(Cluster_Function{4}, 1)
    Name_G4{i, 1} = MaxMuscleForceMoment{Cluster_Function{4}(i), 1}.Name;
end

for i = 1size(Cluster_Function{5}, 1)
    Name_G5{i, 1} = MaxMuscleForceMoment{Cluster_Function{5}(i), 1}.Name;
end

for i = 1size(Cluster_Function{6}, 1)
    Name_G6{i, 1} = MaxMuscleForceMoment{Cluster_Function{6}(i), 1}.Name;
end

for i = 1size(Cluster_Function{7}, 1)
    Name_G7{i, 1} = MaxMuscleForceMoment{Cluster_Function{7}(i), 1}.Name;
end

for i = 1size(Cluster_Function{8}, 1)
    Name_G8{i, 1} = MaxMuscleForceMoment{Cluster_Function{8}(i), 1}.Name;
end

for i = 1size(Cluster_Function{9}, 1)
    Name_G9{i, 1} = MaxMuscleForceMoment{Cluster_Function{9}(i), 1}.Name;
end
