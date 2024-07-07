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



lb =  [0.0579, 0.0058, -5.64*(pi/180), 29.47*(pi/180), 0.0032];
ub =  [0.0801, 0.0116, -2.42*(pi/180), 104.85*(pi/180), 0.0099];


x0 = lb + (ub-lb).*rand(1,1);



[x,fval,exitflag,output,lambda,grad,hessian] = IPGenerateCode(x0,lb,ub,MaxFunctionEvaluations_Data,MaxIterations_Data,OptimalityTolerance_Data,StepTolerance_Data,ObjectiveLimit_Data,ConstraintTolerance_Data);
    


