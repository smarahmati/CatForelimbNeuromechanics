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

G = 5; % Muscle Group Number (G = 1 to 9)
x0(1) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.a1OW;
x0(2) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.a2OW;
x0(3) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.phi1OW;
x0(4) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.phi2OW;
x0(5) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.a2WI;
x0(6) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.phi2WI;
x0(7) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.OW.R1OW;
x0(8) = NineMuscleMorphParaInitial{G, 1}.MorphologicalParameters.WI.R1WI;


lb = [0.0114, 0.0951, 9.58*(pi/180), 2.92*(pi/180), 0.0279, -1.48*(pi/180), 0.0053, 0.0035];
ub = [0.0207, 0.0965, 21.87*(pi/180), 4.48*(pi/180), 0.0333, 7.21*(pi/180), 0.0055, 0.0061];

x0 = lb + (ub-lb).*rand(1,1);



[x,fval,exitflag,output,lambda,grad,hessian] = IPGenerateCode(x0,lb,ub,MaxFunctionEvaluations_Data,MaxIterations_Data,OptimalityTolerance_Data,StepTolerance_Data,ObjectiveLimit_Data,ConstraintTolerance_Data);
    


