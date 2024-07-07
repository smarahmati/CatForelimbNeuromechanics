clear
clc

% This M-file creates MuscleLengthVelocityMomentArm_9Groups corresponding to the musculotendon length, velocity and moment arm during locomotion
% This M-file creates MusculoskeletalData_9Groups corresponding to the joint center, muscle origin, insertion and geometric parameters  during locomotion
% This M-file creates MaxMuscleForceMoment_9Groups corresponding to the musculotendon force, moment, fascicle length, tendon length and etc during locomotion with maximum activation

f = 0;

figure(5)
pause(2)
for ii = 1
    % 1:101

% 1
Results_1
hold on
pause (0)

% 2
Results_2
hold on
pause (0)

% 3
Results_3
hold on
pause (0)


% 4
Results_4
hold on
pause (0)

% 5
Results_5
hold on
pause (0)

% 6
Results_6
hold on
pause (0)

% 7
Results_7
hold on
pause (0)

% 8
Results_8
hold on
pause (0)

% 9
Results_9
hold on
pause (0)



hold off

%save figure to create gif file
filename = ['Musculoskeletal Simulation (9 groups)', '.gif'];
% h= figure(4);
drawnow
% Capture the plot as an image
frame = getframe(h);
im = frame2im(frame);
[imind,cm] = rgb2ind(im,250);
% Write to the GIF File
if f == 0
imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
else
imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.1);
end
f=f+1;
F(ii) = getframe(gcf); 

end

% savefig('Figure 3.fig');





























