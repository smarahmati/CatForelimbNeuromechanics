function circular_arrow(figHandle, radius, center, arrow_angle, angle, colour, linewidth, arrowhead_angle)


arrow_angle = arrow_angle*(pi/180);
angle = angle*(pi/180);
arrowhead_angle = arrowhead_angle*(pi/180);


th = arrow_angle:angle/100:arrow_angle+angle;
x = center(1);
y = center(2);
r = radius;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
plot(xunit, yunit,'color',colour,'LineWidth',linewidth);
hold on


xp1 = xunit(end-1);
yp1 = yunit(end-1);

xp2 = xunit(end);
yp2 = yunit(end);

% arrow head coordinates
xp3 = -(xp1*xp2 + yp1*yp2 - xp1^2 - yp1^2 + (yp1*(yp1*cos(arrowhead_angle) + xp1*(-(cos(arrowhead_angle) - 1)*(cos(arrowhead_angle) + 1))^(1/2) - xp2*(-(cos(arrowhead_angle) - 1)*(cos(arrowhead_angle) + 1))^(1/2)))/cos(arrowhead_angle) - (yp2*(yp1*cos(arrowhead_angle) + xp1*(-(cos(arrowhead_angle) - 1)*(cos(arrowhead_angle) + 1))^(1/2) - xp2*(-(cos(arrowhead_angle) - 1)*(cos(arrowhead_angle) + 1))^(1/2)))/cos(arrowhead_angle))/(xp1 - xp2);
yp3 = (yp1*cos(arrowhead_angle) + xp1*(-(cos(arrowhead_angle) - 1)*(cos(arrowhead_angle) + 1))^(1/2) - xp2*(-(cos(arrowhead_angle) - 1)*(cos(arrowhead_angle) + 1))^(1/2))/cos(arrowhead_angle);

xp4 = -(xp1*xp2 + yp1*yp2 - xp1^2 - yp1^2 + (yp1*(yp1*cos(arrowhead_angle) - xp1*(-(cos(arrowhead_angle) - 1)*(cos(arrowhead_angle) + 1))^(1/2) + xp2*(-(cos(arrowhead_angle) - 1)*(cos(arrowhead_angle) + 1))^(1/2)))/cos(arrowhead_angle) - (yp2*(yp1*cos(arrowhead_angle) - xp1*(-(cos(arrowhead_angle) - 1)*(cos(arrowhead_angle) + 1))^(1/2) + xp2*(-(cos(arrowhead_angle) - 1)*(cos(arrowhead_angle) + 1))^(1/2)))/cos(arrowhead_angle))/(xp1 - xp2);
yp4 = (yp1*cos(arrowhead_angle) - xp1*(-(cos(arrowhead_angle) - 1)*(cos(arrowhead_angle) + 1))^(1/2) + xp2*(-(cos(arrowhead_angle) - 1)*(cos(arrowhead_angle) + 1))^(1/2))/cos(arrowhead_angle);

head_size = 10*abs(angle);
if head_size> 8
    head_size = 8;
end
xp3 = head_size*(xp3-xp2) + xp2;
yp3 = head_size*(yp3-yp2) + yp2;

xp4 = head_size*(xp4-xp2) + xp2;
yp4 = head_size*(yp4-yp2) + yp2;



plot([xp3 xp2], [yp3 yp2],'color',colour,'LineWidth',linewidth);
hold on
plot([xp4 xp2], [yp4 yp2],'color',colour,'LineWidth',linewidth);


end
