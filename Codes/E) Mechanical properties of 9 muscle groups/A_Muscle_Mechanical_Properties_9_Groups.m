% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% This script calculates and summarizes the mechanical properties of 9 muscle groups based on clustering results. It also prepares the data for publication by calculating various properties such as mass, PCSA, and fiber type composition, and saves the results in a table.

clear;
clc;

% Load the MuscleMP and MuscleCluster data
MuscleMP = load('MuscleMP.mat');
MuscleMP = MuscleMP.MuscleMP;

MuscleCluster = load('MuscleCluster.mat');
MuscleCluster = MuscleCluster.MuscleCluster;

%% Mechanical properties of 9 muscle groups

%-----------Shoulder Extensors (Group 1)------------------------------------------------------
G = 1;
Mass_SE = sum(MuscleMP.Mass_Ave(MuscleCluster{G})); 
PCSA_SE = sum(MuscleMP.PCSA_Ave(MuscleCluster{G}));
ML_SE = mean(MuscleMP.ML_Ave(MuscleCluster{G}));
FL_SE = mean(MuscleMP.FL_Ave(MuscleCluster{G}));
SL_SE = mean(MuscleMP.SL_Ave(MuscleCluster{G}));
Sarc_SE = mean(MuscleMP.Sarc_Ave(MuscleCluster{G}));
PA_SE = mean(MuscleMP.PA_Ave(MuscleCluster{G}));
LM0_SE = mean(MuscleMP.LM0_Ave(MuscleCluster{G}));
FL_ratio_SE = mean(MuscleMP.FL_ratio_Ave(MuscleCluster{G}));
OptimalML_SE = mean(MuscleMP.OptimalML_Ave(MuscleCluster{G}));
SO_SE = mean(MuscleMP.SO_Ave(MuscleCluster{G}));
FOG_SE = mean(MuscleMP.FOG_Ave(MuscleCluster{G}));
FG_SE = mean(MuscleMP.FG_Ave(MuscleCluster{G}));
Vmax_LM0_SE = mean(MuscleMP.Vmax_LM0_Ave(MuscleCluster{G}));
Musculotendon_Length3D_SE = mean(MuscleMP.Musculotendon_Length3D(MuscleCluster{G}));
Musculotendon_Length2D_SE = mean(MuscleMP.Musculotendon_Length2D(MuscleCluster{G}));

%-----------Elbow Extensors (Group 2)---------------------------------------------------------
G = 2;
Mass_EE = sum(MuscleMP.Mass_Ave(MuscleCluster{G})); 
PCSA_EE = sum(MuscleMP.PCSA_Ave(MuscleCluster{G}));
ML_EE = mean(MuscleMP.ML_Ave(MuscleCluster{G}));
FL_EE = mean(MuscleMP.FL_Ave(MuscleCluster{G}));
SL_EE = mean(MuscleMP.SL_Ave(MuscleCluster{G}));
Sarc_EE = mean(MuscleMP.Sarc_Ave(MuscleCluster{G}));
PA_EE = mean(MuscleMP.PA_Ave(MuscleCluster{G}));
LM0_EE = mean(MuscleMP.LM0_Ave(MuscleCluster{G}));
FL_ratio_EE = mean(MuscleMP.FL_ratio_Ave(MuscleCluster{G}));
OptimalML_EE = mean(MuscleMP.OptimalML_Ave(MuscleCluster{G}));
SO_EE = mean(MuscleMP.SO_Ave(MuscleCluster{G}));
FOG_EE = mean(MuscleMP.FOG_Ave(MuscleCluster{G}));
FG_EE = mean(MuscleMP.FG_Ave(MuscleCluster{G}));
Vmax_LM0_EE = mean(MuscleMP.Vmax_LM0_Ave(MuscleCluster{G}));
Musculotendon_Length3D_EE = mean(MuscleMP.Musculotendon_Length3D(MuscleCluster{G}));
Musculotendon_Length2D_EE = mean(MuscleMP.Musculotendon_Length2D(MuscleCluster{G}));

%-----------Biceps Brachii (Group 3)----------------------------------------------------------
G = 3;
Mass_BB = sum(MuscleMP.Mass_Ave(MuscleCluster{G})); 
PCSA_BB = sum(MuscleMP.PCSA_Ave(MuscleCluster{G}));
ML_BB = mean(MuscleMP.ML_Ave(MuscleCluster{G}));
FL_BB = mean(MuscleMP.FL_Ave(MuscleCluster{G}));
SL_BB = mean(MuscleMP.SL_Ave(MuscleCluster{G}));
Sarc_BB = mean(MuscleMP.Sarc_Ave(MuscleCluster{G}));
PA_BB = mean(MuscleMP.PA_Ave(MuscleCluster{G}));
LM0_BB = mean(MuscleMP.LM0_Ave(MuscleCluster{G}));
FL_ratio_BB = mean(MuscleMP.FL_ratio_Ave(MuscleCluster{G}));
OptimalML_BB = mean(MuscleMP.OptimalML_Ave(MuscleCluster{G}));
SO_BB = mean(MuscleMP.SO_Ave(MuscleCluster{G}));
FOG_BB = mean(MuscleMP.FOG_Ave(MuscleCluster{G}));
FG_BB = mean(MuscleMP.FG_Ave(MuscleCluster{G}));
Vmax_LM0_BB = mean(MuscleMP.Vmax_LM0_Ave(MuscleCluster{G}));
Musculotendon_Length3D_BB = mean(MuscleMP.Musculotendon_Length3D(MuscleCluster{G}));
Musculotendon_Length2D_BB = mean(MuscleMP.Musculotendon_Length2D(MuscleCluster{G}));

%-----------Elbow Flexors (Group 4)-----------------------------------------------------------
G = 4;
Mass_EF = sum(MuscleMP.Mass_Ave(MuscleCluster{G})); 
PCSA_EF = sum(MuscleMP.PCSA_Ave(MuscleCluster{G}));
ML_EF = mean(MuscleMP.ML_Ave(MuscleCluster{G}));
FL_EF = mean(MuscleMP.FL_Ave(MuscleCluster{G}));
SL_EF = mean(MuscleMP.SL_Ave(MuscleCluster{G}));
Sarc_EF = mean(MuscleMP.Sarc_Ave(MuscleCluster{G}));
PA_EF = mean(MuscleMP.PA_Ave(MuscleCluster{G}));
LM0_EF = mean(MuscleMP.LM0_Ave(MuscleCluster{G}));
FL_ratio_EF = mean(MuscleMP.FL_ratio_Ave(MuscleCluster{G}));
OptimalML_EF = mean(MuscleMP.OptimalML_Ave(MuscleCluster{G}));
SO_EF = mean(MuscleMP.SO_Ave(MuscleCluster{G}));
FOG_EF = mean(MuscleMP.FOG_Ave(MuscleCluster{G}));
FG_EF = mean(MuscleMP.FG_Ave(MuscleCluster{G}));
Vmax_LM0_EF = mean(MuscleMP.Vmax_LM0_Ave(MuscleCluster{G}));
Musculotendon_Length3D_EF = mean(MuscleMP.Musculotendon_Length3D(MuscleCluster{G}));
Musculotendon_Length2D_EF = mean(MuscleMP.Musculotendon_Length2D(MuscleCluster{G}));

%-----------Elbow Flexors Wrist Flexors (Group 5)--------------------------------------------
G = 5;
Mass_EF_WF = sum(MuscleMP.Mass_Ave(MuscleCluster{G})); 
PCSA_EF_WF = sum(MuscleMP.PCSA_Ave(MuscleCluster{G}));
ML_EF_WF = mean(MuscleMP.ML_Ave(MuscleCluster{G}));
FL_EF_WF = mean(MuscleMP.FL_Ave(MuscleCluster{G}));
SL_EF_WF = mean(MuscleMP.SL_Ave(MuscleCluster{G}));
Sarc_EF_WF = mean(MuscleMP.Sarc_Ave(MuscleCluster{G}));
PA_EF_WF = mean(MuscleMP.PA_Ave(MuscleCluster{G}));
LM0_EF_WF = mean(MuscleMP.LM0_Ave(MuscleCluster{G}));
FL_ratio_EF_WF = mean(MuscleMP.FL_ratio_Ave(MuscleCluster{G}));
OptimalML_EF_WF = mean(MuscleMP.OptimalML_Ave(MuscleCluster{G}));
SO_EF_WF = mean(MuscleMP.SO_Ave(MuscleCluster{G}));
FOG_EF_WF = mean(MuscleMP.FOG_Ave(MuscleCluster{G}));
FG_EF_WF = mean(MuscleMP.FG_Ave(MuscleCluster{G}));
Vmax_LM0_EF_WF = mean(MuscleMP.Vmax_LM0_Ave(MuscleCluster{G}));
Musculotendon_Length3D_EF_WF = mean(MuscleMP.Musculotendon_Length3D(MuscleCluster{G}));
Musculotendon_Length2D_EF_WF = mean(MuscleMP.Musculotendon_Length2D(MuscleCluster{G}));

%-----------Wrist Flexors (Group 6)----------------------------------------------------------
G = 6;
Mass_WF = sum(MuscleMP.Mass_Ave(MuscleCluster{G})); 
PCSA_WF = sum(MuscleMP.PCSA_Ave(MuscleCluster{G}));
ML_WF = mean(MuscleMP.ML_Ave(MuscleCluster{G}));
FL_WF = mean(MuscleMP.FL_Ave(MuscleCluster{G}));
SL_WF = mean(MuscleMP.SL_Ave(MuscleCluster{G}));
Sarc_WF = mean(MuscleMP.Sarc_Ave(MuscleCluster{G}));
PA_WF = mean(MuscleMP.PA_Ave(MuscleCluster{G}));
LM0_WF = mean(MuscleMP.LM0_Ave(MuscleCluster{G}));
FL_ratio_WF = mean(MuscleMP.FL_ratio_Ave(MuscleCluster{G}));
OptimalML_WF = mean(MuscleMP.OptimalML_Ave(MuscleCluster{G}));
SO_WF = mean(MuscleMP.SO_Ave(MuscleCluster{G}));
FOG_WF = mean(MuscleMP.FOG_Ave(MuscleCluster{G}));
FG_WF = mean(MuscleMP.FG_Ave(MuscleCluster{G}));
Vmax_LM0_WF = mean(MuscleMP.Vmax_LM0_Ave(MuscleCluster{G}));
Musculotendon_Length3D_WF = mean(MuscleMP.Musculotendon_Length3D(MuscleCluster{G}));
Musculotendon_Length2D_WF = mean(MuscleMP.Musculotendon_Length2D(MuscleCluster{G}));

%-----------Wrist Extensors (Group 7)--------------------------------------------------------
G = 7;
Mass_WE = sum(MuscleMP.Mass_Ave(MuscleCluster{G})); 
PCSA_WE = sum(MuscleMP.PCSA_Ave(MuscleCluster{G}));
ML_WE = mean(MuscleMP.ML_Ave(MuscleCluster{G}));
FL_WE = mean(MuscleMP.FL_Ave(MuscleCluster{G}));
SL_WE = mean(MuscleMP.SL_Ave(MuscleCluster{G}));
Sarc_WE = mean(MuscleMP.Sarc_Ave(MuscleCluster{G}));
PA_WE = mean(MuscleMP.PA_Ave(MuscleCluster{G}));
LM0_WE = mean(MuscleMP.LM0_Ave(MuscleCluster{G}));
FL_ratio_WE = mean(MuscleMP.FL_ratio_Ave(MuscleCluster{G}));
OptimalML_WE = mean(MuscleMP.OptimalML_Ave(MuscleCluster{G}));
SO_WE = mean(MuscleMP.SO_Ave(MuscleCluster{G}));
FOG_WE = mean(MuscleMP.FOG_Ave(MuscleCluster{G}));
FG_WE = mean(MuscleMP.FG_Ave(MuscleCluster{G}));
Vmax_LM0_WE = mean(MuscleMP.Vmax_LM0_Ave(MuscleCluster{G}));
Musculotendon_Length3D_WE = mean(MuscleMP.Musculotendon_Length3D(MuscleCluster{G}));
Musculotendon_Length2D_WE = mean(MuscleMP.Musculotendon_Length2D(MuscleCluster{G}));

%-----------Shoulder Flexors (Group 8)-------------------------------------------------------
G = 8;
Mass_SF = sum(MuscleMP.Mass_Ave(MuscleCluster{G})); 
PCSA_SF = sum(MuscleMP.PCSA_Ave(MuscleCluster{G}));
ML_SF = mean(MuscleMP.ML_Ave(MuscleCluster{G}));
FL_SF = mean(MuscleMP.FL_Ave(MuscleCluster{G}));
SL_SF = mean(MuscleMP.SL_Ave(MuscleCluster{G}));
Sarc_SF = mean(MuscleMP.Sarc_Ave(MuscleCluster{G}));
PA_SF = mean(MuscleMP.PA_Ave(MuscleCluster{G}));
LM0_SF = mean(MuscleMP.LM0_Ave(MuscleCluster{G}));
FL_ratio_SF = mean(MuscleMP.FL_ratio_Ave(MuscleCluster{G}));
OptimalML_SF = mean(MuscleMP.OptimalML_Ave(MuscleCluster{G}));
SO_SF = mean(MuscleMP.SO_Ave(MuscleCluster{G}));
FOG_SF = mean(MuscleMP.FOG_Ave(MuscleCluster{G}));
FG_SF = mean(MuscleMP.FG_Ave(MuscleCluster{G}));
Vmax_LM0_SF = mean(MuscleMP.Vmax_LM0_Ave(MuscleCluster{G}));
Musculotendon_Length3D_SF = mean(MuscleMP.Musculotendon_Length3D(MuscleCluster{G}));
Musculotendon_Length2D_SF = mean(MuscleMP.Musculotendon_Length2D(MuscleCluster{G}));

%-----------Triceps Brachii Long (Group 9)---------------------------------------------------
G = 9;
Mass_TBL = sum(MuscleMP.Mass_Ave(MuscleCluster{G})); 
PCSA_TBL = sum(MuscleMP.PCSA_Ave(MuscleCluster{G}));
ML_TBL = mean(MuscleMP.ML_Ave(MuscleCluster{G}));
FL_TBL = mean(MuscleMP.FL_Ave(MuscleCluster{G}));
SL_TBL = mean(MuscleMP.SL_Ave(MuscleCluster{G}));
Sarc_TBL = mean(MuscleMP.Sarc_Ave(MuscleCluster{G}));
PA_TBL = mean(MuscleMP.PA_Ave(MuscleCluster{G}));
LM0_TBL = mean(MuscleMP.LM0_Ave(MuscleCluster{G}));
FL_ratio_TBL = mean(MuscleMP.FL_ratio_Ave(MuscleCluster{G}));
OptimalML_TBL = mean(MuscleMP.OptimalML_Ave(MuscleCluster{G}));
SO_TBL = mean(MuscleMP.SO_Ave(MuscleCluster{G}));
FOG_TBL = mean(MuscleMP.FOG_Ave(MuscleCluster{G}));
FG_TBL = mean(MuscleMP.FG_Ave(MuscleCluster{G}));
Vmax_LM0_TBL = mean(MuscleMP.Vmax_LM0_Ave(MuscleCluster{G}));
Musculotendon_Length3D_TBL = mean(MuscleMP.Musculotendon_Length3D(MuscleCluster{G}));
Musculotendon_Length2D_TBL = mean(MuscleMP.Musculotendon_Length2D(MuscleCluster{G}));

% Group names and mechanical properties
LastName = {'Shoulder Protractor'; 'Elbow Extensor'; 'Biceps Brachii'; ...
            'Elbow Flexor'; 'Elbow Flexor-Wrist Dorsiflexor'; 'Wrist Dorsiflexor'; ...
            'Wrist Plantarflexor'; 'Shoulder Retractor'; 'Triceps Brachii Long'};
Mass = [Mass_SE; Mass_EE; Mass_BB; Mass_EF; Mass_EF_WF; Mass_WF; Mass_WE; Mass_SF; Mass_TBL];
ML = [ML_SE; ML_EE; ML_BB; ML_EF; ML_EF_WF; ML_WF; ML_WE; ML_SF; ML_TBL];
FL = [FL_SE; FL_EE; FL_BB; FL_EF; FL_EF_WF; FL_WF; FL_WE; FL_SF; FL_TBL];
SL = [SL_SE; SL_EE; SL_BB; SL_EF; SL_EF_WF; SL_WF; SL_WE; SL_SF; SL_TBL];
Sarc = [Sarc_SE; Sarc_EE; Sarc_BB; Sarc_EF; Sarc_EF_WF; Sarc_WF; Sarc_WE; Sarc_SF; Sarc_TBL];
PCSA = [PCSA_SE; PCSA_EE; PCSA_BB; PCSA_EF; PCSA_EF_WF; PCSA_WF; PCSA_WE; PCSA_SF; PCSA_TBL];
PA = [PA_SE; PA_EE; PA_BB; PA_EF; PA_EF_WF; PA_WF; PA_WE; PA_SF; PA_TBL];
LM0 = [LM0_SE; LM0_EE; LM0_BB; LM0_EF; LM0_EF_WF; LM0_WF; LM0_WE; LM0_SF; LM0_TBL];
FL_ratio = [FL_ratio_SE; FL_ratio_EE; FL_ratio_BB; FL_ratio_EF; FL_ratio_EF_WF; FL_ratio_WF; FL_ratio_WE; FL_ratio_SF; FL_ratio_TBL];
OptimalML = [OptimalML_SE; OptimalML_EE; OptimalML_BB; OptimalML_EF; OptimalML_EF_WF; OptimalML_WF; OptimalML_WE; OptimalML_SF; OptimalML_TBL];
SO = [SO_SE; SO_EE; SO_BB; SO_EF; SO_EF_WF; SO_WF; SO_WE; SO_SF; SO_TBL];
FOG = [FOG_SE; FOG_EE; FOG_BB; FOG_EF; FOG_EF_WF; FOG_WF; FOG_WE; FOG_SF; FOG_TBL];
FG = [FG_SE; FG_EE; FG_BB; FG_EF; FG_EF_WF; FG_WF; FG_WE; FG_SF; FG_TBL];
Vmax_LM0 = [Vmax_LM0_SE; Vmax_LM0_EE; Vmax_LM0_BB; Vmax_LM0_EF; Vmax_LM0_EF_WF; Vmax_LM0_WF; Vmax_LM0_WE; Vmax_LM0_SF; Vmax_LM0_TBL];
Musculotendon_Length3D = [Musculotendon_Length3D_SE; Musculotendon_Length3D_EE; Musculotendon_Length3D_BB; ...
                          Musculotendon_Length3D_EF; Musculotendon_Length3D_EF_WF; Musculotendon_Length3D_WF; ...
                          Musculotendon_Length3D_WE; Musculotendon_Length3D_SF; Musculotendon_Length3D_TBL];                                        
Musculotendon_Length2D = [Musculotendon_Length2D_SE; Musculotendon_Length2D_EE; Musculotendon_Length2D_BB; ...
                          Musculotendon_Length2D_EF; Musculotendon_Length2D_EF_WF; Musculotendon_Length2D_WF; ...
                          Musculotendon_Length2D_WE; Musculotendon_Length2D_SF; Musculotendon_Length2D_TBL];

% Create a table for the mechanical properties of the 9 muscle groups
MuscleMP_9Groups = table(LastName, Mass, ML, FL, SL, Sarc, ...
                         PCSA, PA, LM0, FL_ratio, OptimalML, ...
                         SO, FOG, FG, Vmax_LM0, Musculotendon_Length3D, Musculotendon_Length2D);

% Save the table to a .mat file
save('MuscleMP_9Groups.mat', 'MuscleMP_9Groups')

% Prepare data for publication
LastName_abb = {'Sh Protr'; 'El Exts'; 'Sh protr-El fl (BB)'; ...
                'El Fl'; 'El fl-Wr DFl'; 'Wr DFl'; ...
                'Wr PFl'; 'Sh Retr'; 'Sh Retr-El Ex (TL)'};

for i = 1:9
    SO_str{i, 1} = round(SO(i, 1), 1);
    FOG_str{i, 1} = round(FOG(i, 1), 1);
    FG_str{i, 1} = round(FG(i, 1), 1);
    Fiber_Type{i, 1} = [num2str(SO_str{i, 1}), ',', num2str(FOG_str{i, 1}), ',', num2str(FG_str{i, 1})];
end

Mass = round(Mass, 2);
LM0 = round(LM0, 1);
PA = round(PA, 0);
PCSA = round(PCSA, 2);
Vmax_LM0 = round(Vmax_LM0, 1);

% Create a table for the publication
MuscleMP_9Groups_paper = table(LastName, LastName_abb, Mass, LM0, ...
                               PA, PCSA, Fiber_Type, Vmax_LM0);

% Reorder the groups for publication
MuscleMP_9Groups_paper_Pro_Dis(1, :) = MuscleMP_9Groups_paper(1, :);
MuscleMP_9Groups_paper_Pro_Dis(2, :) = MuscleMP_9Groups_paper(8, :);
MuscleMP_9Groups_paper_Pro_Dis(3, :) = MuscleMP_9Groups_paper(3, :);
MuscleMP_9Groups_paper_Pro_Dis(4, :) = MuscleMP_9Groups_paper(9, :);
MuscleMP_9Groups_paper_Pro_Dis(5, :) = MuscleMP_9Groups_paper(2, :);
MuscleMP_9Groups_paper_Pro_Dis(6, :) = MuscleMP_9Groups_paper(4, :);
MuscleMP_9Groups_paper_Pro_Dis(7, :) = MuscleMP_9Groups_paper(5, :);
MuscleMP_9Groups_paper_Pro_Dis(8, :) = MuscleMP_9Groups_paper(7, :);
MuscleMP_9Groups_paper_Pro_Dis(9, :) = MuscleMP_9Groups_paper(6, :);

% Save the table to an Excel file
writetable(MuscleMP_9Groups_paper_Pro_Dis, 'MuscleArchitecturalProperties_9Groups.xlsx')
