function [x,fval,exitflag,output,lambda,grad,hessian] = IPGenerateCode(x0,lb,ub,MaxFunctionEvaluations_Data,MaxIterations_Data,OptimalityTolerance_Data,StepTolerance_Data,ObjectiveLimit_Data,ConstraintTolerance_Data)
%% This is an auto generated MATLAB file from Optimization Tool.

%% Start with the default options
options = optimoptions('fmincon');
%% Modify options setting
options = optimoptions(options,'Display', 'off');
options = optimoptions(options,'MaxFunctionEvaluations', MaxFunctionEvaluations_Data);
options = optimoptions(options,'MaxIterations', MaxIterations_Data);
options = optimoptions(options,'OptimalityTolerance', OptimalityTolerance_Data);
options = optimoptions(options,'FunctionTolerance', OptimalityTolerance_Data);
options = optimoptions(options,'StepTolerance', StepTolerance_Data);
options = optimoptions(options,'PlotFcn', {  @optimplotx @optimplotfval @optimplotconstrviolation });
options = optimoptions(options,'ObjectiveLimit', ObjectiveLimit_Data);
options = optimoptions(options,'ConstraintTolerance', ConstraintTolerance_Data);
options = optimoptions(options,'UseParallel', true);
[x,fval,exitflag,output,lambda,grad,hessian] = ...
fmincon(@Cost_Function,x0,[],[],[],[],lb,ub,@Constraint_Function,options);
