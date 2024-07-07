% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% After running the optimization process (A_Optimization), the optimal
% muscle activations is created and saved in Activations.mat. This script loads
% the results from Activations.mat, computes muscle forces and moments, and calculates
% the sum of muscle moments at each joint using Results_Function.
% This script compares the joint moments with the sum of the corresponding muscle moments at the joint 
% to ensure equality. Additionally, it plots the activation for a muscle selected by the user.


clear
clc

% Motion results including joint angles and moments
MotionData = load('MotionData.mat');
MotionData = MotionData.MotionData;

% 40 Muscles
MuscleLengthVelocityMomentArm = load('MuscleLengthVelocityMomentArm.mat');
MuscleLengthVelocityMomentArm = MuscleLengthVelocityMomentArm.MuscleLengthVelocityMomentArm;

% 40 Muscles
MaxMuscleForceMoment = load('MaxMuscleForceMoment.mat');
MaxMuscleForceMoment = MaxMuscleForceMoment.MaxMuscleForceMoment;

% 9 Muscles
T = load('MuscleMP.mat');
T2 = T.MuscleMP;
MuscleMP = table2array(T2(1:end, 2:end));



% time
CT = MotionData.GenericCat.CycleTime.Average;
timesize = MuscleLengthVelocityMomentArm{1, 1}.Length;
dt = CT/(size(timesize,1)-1);
time = (0:size(timesize,1)-1)'*dt;


% number of muscles 
nm = size(MuscleLengthVelocityMomentArm,1);       % number of muscles 

% xop: muscles activations (40 muscles) at time step of TS
% TS: time step (example: 0 to 101)



load('Activations');


Activations_Nan = Activations;
Activations_filled = Activations;
% fillmissing
for i=1:size(Activations,1)
if sum(Activations{i,1}.a==0) ~= 0
    Act = Activations{i,1}.a;
    Act(Act==0) = NaN;  
    Activations_Nan{i,1}.a = Act;

  Activations_filled{i,1}.a = fillmissing(Activations_Nan{i,1}.a,'linear');
  
else    
  Activations_filled{i,1}.a = Activations{i,1}.a;
end

  Activations_Matrix(:,i) = Activations_filled{i,1}.a;
end



for TS = 1:size(Activations_Matrix,1)
TS
save('TS.mat','TS');
xopt = Activations_Matrix(TS,:);
[MMT_Shoulder(TS,:), MMT_Elbow(TS,:), MMT_Wrist(TS,:), MMT(TS,:), FMT(TS,:)] = Results_Function (xopt);
end



%----- optimized MT force and moment during motion -------------
for i=1:size(MaxMuscleForceMoment,1)
OptimizedMuscleForceMoment{i,1}.Name = MaxMuscleForceMoment{i, 1}.Name;
OptimizedMuscleForceMoment{i,1}.Force = FMT(:,i);
OptimizedMuscleForceMoment{i,1}.Moment.Shoulder = MMT_Shoulder(:,i);
OptimizedMuscleForceMoment{i,1}.Moment.Elbow = MMT_Elbow(:,i);
OptimizedMuscleForceMoment{i,1}.Moment.Wrist = MMT_Wrist(:,i);
end
save('OptimizedMuscleForceMoment.mat','OptimizedMuscleForceMoment');


%% Plot
% Muscle moment vs joint moment

figure(1)
plot(MMT(:,1),'Ob','LineWidth',2)
hold on
plot(MotionData.GenericCat.Moments.MomShou.Total,'-k','LineWidth',2)
hold on
plot(MMT(:,1)*0,'.k','LineWidth',1)
legend('Muscle moment','Joint moment')
xlabel('Time Step')
ylabel('Shoulder Moment (Nm)')
title('Shoulder')


figure(2)
plot(MMT(:,2),'Ob','LineWidth',2)
hold on
plot(MotionData.GenericCat.Moments.MomElbow.Total,'-k','LineWidth',2)
hold on
plot(MMT(:,1)*0,'.k','LineWidth',1)
legend('Muscle moment','Joint moment')
xlabel('Time Step')
ylabel('Elbow Moment (Nm)')
title('Elbow')

figure(3)
plot(MMT(:,3),'Ob','LineWidth',2)
hold on
plot(MotionData.GenericCat.Moments.MomWrist.Total,'-k','LineWidth',2)
hold on
plot(MMT(:,1)*0,'.k','LineWidth',1)
legend('Muscle moment','Joint moment')
xlabel('Time Step')
ylabel('Wrist Moment (Nm)')
title('Wrist')



%% Muscle activation
rowNumber = selectMuscle_from40();
mn = rowNumber;
figure (4)
p1 = plot(time, Activations_filled{mn,1}.a,'-k','LineWidth',1);
hold on
grid on
xlabel('Time(s)');
ylabel('a');
title(['Activation ', '(', Activations_filled{mn,1}.Name, ')'])
set(gca,'XTick',[])
hold on
y_min = min(Activations_filled{mn,1}.a);
y_max = max(Activations_filled{mn,1}.a);
max_min = y_max-y_min;
x_min = 0;
x_max = time(end);
x_mid = x_max*(1-MotionData.GenericCat.DutyFactor.Average);
width = max_min*0.2;
l1 = line([x_min, x_mid],[y_min-0.05*max_min, y_min-0.05*max_min], 'LineWidth',2,'LineStyle','-.','Color','b');
hold on
l2 = line([x_mid, x_max],[y_min-0.1*max_min, y_min-0.1*max_min], 'LineWidth',2,'LineStyle','-','Color','g');
legend([l1, l2], 'Swing', 'Stance'); 
ylim([y_min-0.1*max_min 1])


