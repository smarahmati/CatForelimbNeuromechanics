% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
% Description:
% This script computes a sensitivity analysis of the mean maximum muscle-tendon (MT) moment to changes in 
% muscle parameters for the equivalent muscle in group 7, i.e., Wrist Plantarflexors
% The first-order global sensitivity index for each parameter was computed using the 
% Sobol method (Sobol, 2001), implemented in the SobolGSA software (Kucherenko and Zaccheus, 2016).
% 
% For implementation, please first download and install the corresponding toolbox.
%
% Toolbox: Flax (2024). Global Sensitivity Analysis Toolbox (https://www.mathworks.com/matlabcentral/fileexchange/40759-global-sensitivity-analysis-toolbox), 
% MATLAB Central File Exchange. Retrieved July 5, 2024.

clear
clc

G = 7; % Muscle Group Number (G = 1 to 9)

xopt = [0.00637612706495929,0.0962613055647396,-0.0465736783811647,-0.0516758634738425,0.0288587788186216,-0.163175752538083,0.00271044296770131];

a1OW = xopt(1);
a2OW = xopt(2);
phi1OW = xopt(3);
phi2OW = xopt(4);
a2WI = xopt(5);
phi2WI = xopt(6);
R1WI = xopt(7);

% 9 Muscles
MuscleMP_9Groups = load('MuscleMP_9Groups.mat');
MuscleMP_9Groups = MuscleMP_9Groups.MuscleMP_9Groups;
Data_MuscleMP_9Groups = table2array(MuscleMP_9Groups(1:end, 2:end));


LF0 = Data_MuscleMP_9Groups(G,8);         % Optimal fascicle length (mm)
Vmax_LF0 = -Data_MuscleMP_9Groups(G,14);  % Vamx/LF0 (1/s) is negative for current formulation
SO = Data_MuscleMP_9Groups(G,11);         % Percentage of sl-twitch fibres
PCSA = Data_MuscleMP_9Groups(G,6);        % Physiological cross sectional area (cm^2)
PA = Data_MuscleMP_9Groups(G,7);          % Pennation angle (deg)

% x = [a1,a2,phi1,phi2,R1,LF0,Vmax_LF0,SO,PCSA,PA];
% [Mean_MMT_Elbow] = Muscle_Moment_G7(x);



%% Sobol sensitivity analysis
% create a new project 
pro = pro_Create();

% add 3 input variables with a pdf uniformely distributed in [-pi pi]
% pro = pro_AddInput(pro, @(N)pdf_Uniform(N, [-pi pi]), 'X1');
% pro = pro_AddInput(pro, @(N)pdf_Uniform(N, [-pi pi]), 'X2');
% pro = pro_AddInput(pro, @(N)pdf_Uniform(N, [-pi pi]), 'X3');

% add to the project 3 input variables, named X*, distributed in the 
% range [-pi pi] and indicate that the variables will be sampled following a 
% Sobol quasi-random set 
dev = 0.1;
pro = pro_AddInput(pro, @()pdf_Sobol([a1OW-dev*abs(a1OW) a1OW+dev*abs(a1OW)]), 'a1OW');
pro = pro_AddInput(pro, @()pdf_Sobol([a2OW-dev*abs(a2OW) a2OW+dev*abs(a2OW)]), 'a2OW');
pro = pro_AddInput(pro, @()pdf_Sobol([phi1OW-dev*abs(phi1OW) phi1OW+dev*abs(phi1OW)]), 'phi1OW');
pro = pro_AddInput(pro, @()pdf_Sobol([phi2OW-dev*abs(phi2OW) phi2OW+dev*abs(phi2OW)]), 'phi2OW');
pro = pro_AddInput(pro, @()pdf_Sobol([a2WI-dev*abs(a2WI) a2WI+dev*abs(a2WI)]), 'a2WI');
pro = pro_AddInput(pro, @()pdf_Sobol([phi2WI-dev*abs(phi2WI) phi2WI+dev*abs(phi2WI)]), 'phi2WI');
pro = pro_AddInput(pro, @()pdf_Sobol([R1WI-dev*abs(R1WI) R1WI+dev*abs(R1WI)]), 'R1WI');

pro = pro_AddInput(pro, @()pdf_Sobol([LF0-dev*abs(LF0) LF0+dev*abs(LF0)]), 'LF0');
pro = pro_AddInput(pro, @()pdf_Sobol([Vmax_LF0-dev*abs(Vmax_LF0) Vmax_LF0+dev*abs(Vmax_LF0)]), 'Vmax_LFO');
pro = pro_AddInput(pro, @()pdf_Sobol([SO-dev*abs(SO) SO+dev*abs(SO)]), 'SO');
pro = pro_AddInput(pro, @()pdf_Sobol([PCSA-dev*abs(PCSA) PCSA+dev*abs(PCSA)]), 'PCSA');
pro = pro_AddInput(pro, @()pdf_Sobol([PA-dev*abs(PA) PA+dev*abs(PA)]), 'PA');



% set the model, and name it as 'model', to the project 
% the model is well-known as "Ishigami function" (see 3.0.1)
pro = pro_SetModel(pro, @(x)Muscle_Moment_G7(x), 'model');

% set the number of samples for the quasi-random Monte Carlo simulation
pro.N = 100000;

% initialize the project by calculating the model at the sample points
pro = GSA_Init(pro);

% calculate the first order global sensitivity coefficients by using FAST
% algorithm
Sfast_G7 = GSA_FAST_GetSi(pro);


save('Sfast_G7.mat','Sfast_G7');

%% plot
bar(Sfast_G7)
% a1,a2,phi1,phi2,R1,LF0,Vmax_LF0,SO,PCSA,PA
set(gca,'xticklabel',{'a1OW','a2OW','phi1OW','phi2OW','a2WI','phi2WI','R1WI','LF0','Vmax/LF0','SO','PCSA','PA'})
title('Sensitivity analysis of MT momemt to changes in muscle morphological and mechanical parameters')



