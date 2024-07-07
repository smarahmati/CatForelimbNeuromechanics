% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% This script performs an optimization to minimize muscle fatigue for 9 combined muscles by applying using the Cost_Function.
% The optimization parameters are muscle activations.
% A static optimization process is employed, with random initial conditions for the 9 combined muscles at each time step 
% to ensure stable convergence of the results.
% The optimization is constrained by the Constraint_Function, which ensures that the joint moments 
% are equal to the sum of the corresponding muscle moments at the joint.

clear
clc

%
delete(gcp('nocreate'))
myCluster.NumWorkers = 8;
parpool('local',8)
%}

% 9 combined Muscles
MuscleLengthVelocityMomentArm_9Groups = load('MuscleLengthVelocityMomentArm_9Groups.mat');
MuscleLengthVelocityMomentArm_9Groups = MuscleLengthVelocityMomentArm_9Groups.MuscleLengthVelocityMomentArm_9Groups;


MaxFunctionEvaluations_Data = 10^30;
MaxIterations_Data = 500;
OptimalityTolerance_Data = 10^-30;
StepTolerance_Data = 10^-30;
ObjectiveLimit_Data = 10^-30;
ConstraintTolerance_Data = 10^-30;


nti = size(MuscleLengthVelocityMomentArm_9Groups{1, 1}.Length,1); % size of time iterations  
nm =  size(MuscleLengthVelocityMomentArm_9Groups,1);              % number of muscles 


lb =  rand(1,nm)*0;
ub =  rand(1,nm)*0 + 1; 


tic;
for TS = 1:1:nti
TS 
save('TS.mat','TS');

x0 = rand(1,nm);

[x,fval,exitflag,output,lambda,grad,hessian] = IPGenerateCode(x0,lb,ub,MaxFunctionEvaluations_Data,MaxIterations_Data,OptimalityTolerance_Data,StepTolerance_Data,ObjectiveLimit_Data,ConstraintTolerance_Data);


% Activations(TS,:) = x; 
Total_Muscle_Fatigue(TS,1) = fval; 

%----- Activations -------------
for j=1:nm
Activations{j,1}.Name = MuscleLengthVelocityMomentArm_9Groups{j, 1}.NameOfMuscle;
Activations{j,1}.a(TS,1) = x(j);
end


end

Activations_9Groups = Activations;
Total_Muscle_Fatigue_9Groups = Total_Muscle_Fatigue;
save('Activations_9Groups.mat','Activations_9Groups');
save('Total_Muscle_Fatigue_9Groups.mat','Total_Muscle_Fatigue_9Groups');

elapsed = toc