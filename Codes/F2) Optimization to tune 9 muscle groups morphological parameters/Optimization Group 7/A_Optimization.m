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
MaxIterations_Data = 70;
OptimalityTolerance_Data = 10^-300;
StepTolerance_Data = 10^-300;
ObjectiveLimit_Data = 10^-300;
ConstraintTolerance_Data = 10^-300;

G = 7; % Muscle Group Number (G = 1 to 9)
x0(1) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.a1OW;
x0(2) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.a2OW;
x0(3) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.phi1OW;
x0(4) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.phi2OW;
x0(5) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.a2WI;
x0(6) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.phi2WI;
x0(7) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.R1WI;

lb = [0.0011, 0.0936, -35*(pi/180), -4.79*(pi/180), 0.0288, -18.4*(pi/180), 0.0027];
ub = [0.0064, 0.0983, 179.5*(pi/180), -2.55*(pi/180), 0.0329, -9.08*(pi/180), 0.0061];

x0 = lb + (ub-lb).*rand(1,1);


[x,fval,exitflag,output,lambda,grad,hessian] = IPGenerateCode(x0,lb,ub,MaxFunctionEvaluations_Data,MaxIterations_Data,OptimalityTolerance_Data,StepTolerance_Data,ObjectiveLimit_Data,ConstraintTolerance_Data);
    


