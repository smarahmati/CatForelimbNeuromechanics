% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
% Description:
% This script computes a sensitivity analysis of the mean maximum muscle-tendon (MT) moment to changes in 
% muscle parameters for the equivalent muscle in group 2, i.e., Elbow Extensors
% The first-order global sensitivity index for each parameter was computed using the 
% Sobol method (Sobol, 2001), implemented in the SobolGSA software (Kucherenko and Zaccheus, 2016).
% 
% For implementation, please first download and install the corresponding toolbox.
%
% Toolbox: Flax (2024). Global Sensitivity Analysis Toolbox (https://www.mathworks.com/matlabcentral/fileexchange/40759-global-sensitivity-analysis-toolbox), 
% MATLAB Central File Exchange. Retrieved July 5, 2024.

clear
clc

xopt = [0.0600016465411511,0.00558020397995713,0.155275208613396,-2.69130152425526,0.00211767658246807];

a1 = xopt(1);
a2 = xopt(2);
phi1 = xopt(3);
phi2 = xopt(4);
R1 = xopt(5);

% 9 Muscles
MuscleMP_9Groups = load('MuscleMP_9Groups.mat');
MuscleMP_9Groups = MuscleMP_9Groups.MuscleMP_9Groups;
Data_MuscleMP_9Groups = table2array(MuscleMP_9Groups(1:end, 2:end));

G = 2; % Muscle Group Number (G = 1 to 9)
LF0 = Data_MuscleMP_9Groups(G,8);         % Optimal fascicle length (mm)
Vmax_LF0 = -Data_MuscleMP_9Groups(G,14);  % Vamx/LF0 (1/s) is negative for current formulation
SO = Data_MuscleMP_9Groups(G,11);         % Percentage of sl-twitch fibres
PCSA = Data_MuscleMP_9Groups(G,6);        % Physiological cross sectional area (cm^2)
PA = Data_MuscleMP_9Groups(G,7);          % Pennation angle (deg)

% x = [a1,a2,phi1,phi2,R1,LF0,Vmax_LF0,SO,PCSA,PA];
% [Mean_MMT_Shoulder] = Muscle_Moment_G2(x);



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
pro = pro_AddInput(pro, @()pdf_Sobol([a1-dev*abs(a1) a1+dev*abs(a1)]), 'X1');
pro = pro_AddInput(pro, @()pdf_Sobol([a2-dev*abs(a2) a2+dev*abs(a2)]), 'X2');
pro = pro_AddInput(pro, @()pdf_Sobol([phi1-dev*abs(phi1) phi1+dev*abs(phi1)]), 'X3');
pro = pro_AddInput(pro, @()pdf_Sobol([phi2-dev*abs(phi2) phi2+dev*abs(phi2)]), 'X4');
pro = pro_AddInput(pro, @()pdf_Sobol([R1-dev*abs(R1) R1+dev*abs(R1)]), 'X5');

pro = pro_AddInput(pro, @()pdf_Sobol([LF0-dev*abs(LF0) LF0+dev*abs(LF0)]), 'X6');
pro = pro_AddInput(pro, @()pdf_Sobol([Vmax_LF0-dev*abs(Vmax_LF0) Vmax_LF0+dev*abs(Vmax_LF0)]), 'X7');
pro = pro_AddInput(pro, @()pdf_Sobol([SO-dev*abs(SO) SO+dev*abs(SO)]), 'X8');
pro = pro_AddInput(pro, @()pdf_Sobol([PCSA-dev*abs(PCSA) PCSA+dev*abs(PCSA)]), 'X9');
pro = pro_AddInput(pro, @()pdf_Sobol([PA-dev*abs(PA) PA+dev*abs(PA)]), 'X10');



% set the model, and name it as 'model', to the project 
% the model is well-known as "Ishigami function" (see 3.0.1)
pro = pro_SetModel(pro, @(x)Muscle_Moment_G2(x), 'model');

% set the number of samples for the quasi-random Monte Carlo simulation
pro.N = 100000;

% initialize the project by calculating the model at the sample points
pro = GSA_Init(pro);

% calculate the first order global sensitivity coefficients by using FAST
% algorithm
Sfast_G2 = GSA_FAST_GetSi(pro);




save('Sfast.mat','Sfast_G2');
%% plot
bar(Sfast_G2)
% a1,a2,phi1,phi2,R1,LF0,Vmax_LF0,SO,PCSA,PA
set(gca,'xticklabel',{'a1','a2','phi1','phi2','R','LF0','Vmax/LF0','SO','PCSA','PA'})
title('Sensitivity analysis of MT momemt to changes in muscle morphological and mechanical parameters')



