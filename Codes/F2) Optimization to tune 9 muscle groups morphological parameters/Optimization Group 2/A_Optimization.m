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
MaxIterations_Data = 60;
OptimalityTolerance_Data = 10^-300;
StepTolerance_Data = 10^-300;
ObjectiveLimit_Data = 10^-300;
ConstraintTolerance_Data = 10^-300;

G = 4; % Muscle Group Number (G = 1 to 9)
x0(1) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.a1;
x0(2) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.a2;
x0(3) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.phi1;
x0(4) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.phi2;
x0(5) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.R1;


lb =  [0.0363, 0.0028, -4.12*(pi/180),  -157.13*(pi/180), 0.0021];
ub =  [0.0899, 0.0094,  11.24*(pi/180), -87.1*(pi/180), 0.0084];


x0 = lb + (ub-lb).*rand(1,1);


[x,fval,exitflag,output,lambda,grad,hessian] = IPGenerateCode(x0,lb,ub,MaxFunctionEvaluations_Data,MaxIterations_Data,OptimalityTolerance_Data,StepTolerance_Data,ObjectiveLimit_Data,ConstraintTolerance_Data);
    


