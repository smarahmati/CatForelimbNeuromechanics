clear
clc

%{
delete(gcp('nocreate'))
myCluster.NumWorkers = 8;
parpool('local',8)
%}

NineMuscleMorphParaInitial = load('NineMuscleMorphParaInitial.mat');
NineMuscleMorphParaInitial = NineMuscleMorphParaInitial.NineMuscleMorphParaInitial;


MaxFunctionEvaluations_Data = 10^300;
MaxIterations_Data = 100;
OptimalityTolerance_Data = 10^-300;
StepTolerance_Data = 10^-300;
ObjectiveLimit_Data = 10^-300;
ConstraintTolerance_Data = 10^-300;

G = 1; % Muscle Group Number (G = 1 to 9)
x0(1) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.a1;
x0(2) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.a2;
x0(3) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.phi1;
x0(4) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.phi2;
x0(5) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.R1;



lb =  [0.0371, 0.0167, 0*(pi/180), 23.32*(pi/180), 0.0018];  % Minimum of phi1 was changed from -8.04*(pi/180) to 0*(pi/180)
ub =  [0.0564, 0.0191, 17.63*(pi/180), 43.06*(pi/180), 0.0035];


x0 = lb + (ub-lb).*rand(1,1);


[x,fval,exitflag,output,lambda,grad,hessian] = IPGenerateCode(x0,lb,ub,MaxFunctionEvaluations_Data,MaxIterations_Data,OptimalityTolerance_Data,StepTolerance_Data,ObjectiveLimit_Data,ConstraintTolerance_Data);
    


