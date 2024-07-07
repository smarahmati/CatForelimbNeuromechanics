
clear
clc

syms xp yp xp1 yp1 xp2 yp2 arrowhead_angle
eqns = [(xp2-xp1)*(xp-xp1) + (yp2-yp1)*(yp-yp1) == 0, sqrt((xp-xp2)^2 + (yp-yp2)^2)*cos(arrowhead_angle) - sqrt((xp1-xp2)^2 + (yp1-yp2)^2) == 0];
S = solve(eqns,[xp yp]);
