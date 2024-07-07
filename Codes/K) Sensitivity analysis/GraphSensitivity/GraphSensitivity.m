clear
clc

load('Sfast_G1.mat');
load('Sfast_G2.mat');
load('Sfast_G3.mat');
load('Sfast_G4.mat');
load('Sfast_G5.mat');
load('Sfast_G6.mat');
load('Sfast_G7.mat');
load('Sfast_G8.mat');
load('Sfast_G9.mat');


G1_corrected = Sfast_G1(:, 6:10);
G1_corrected(:,6:10) = Sfast_G1(:,1:5);
Sfast_G1 = G1_corrected;

G2_corrected = Sfast_G2(:, 6:10);
G2_corrected(:,6:10) = Sfast_G2(:,1:5);
Sfast_G2 = G2_corrected;

G3_corrected = Sfast_G3(:, 7:11);
G3_corrected(:,6:11) = Sfast_G3(:,1:6);
Sfast_G3 = G3_corrected;

G4_corrected = Sfast_G4(:, 6:10);
G4_corrected(:,6:10) = Sfast_G4(:,1:5);
Sfast_G4 = G4_corrected;

G5_corrected = Sfast_G5(:, 9:13);
G5_corrected(:,6:13) = Sfast_G5(:,1:8);
Sfast_G5 = G5_corrected;

G6_corrected = Sfast_G6(:, 8:12);
G6_corrected(:,6:12) = Sfast_G6(:,1:7);
G6_corrected(10) = Sfast_G6(7);
G6_corrected(11) = Sfast_G6(5);
G6_corrected(12) = Sfast_G6(6);
Sfast_G6 = G6_corrected;

G7_corrected = Sfast_G7(:, 8:12);
G7_corrected(:,6:12) = Sfast_G7(:,1:7);
G7_corrected(10) = Sfast_G7(7);
G7_corrected(11) = Sfast_G7(5);
G7_corrected(12) = Sfast_G7(6);
Sfast_G7 = G7_corrected;

G8_corrected = Sfast_G8(:, 6:10);
G8_corrected(:,6:10) = Sfast_G8(:,1:5);
Sfast_G8 = G8_corrected;

G9_corrected = Sfast_G9(:, 7:11);
G9_corrected(:,6:11) = Sfast_G9(:,1:6);
Sfast_G9 = G9_corrected;
x1 = [0 0 1 1; 2 2 3 3; 4 4 5 5; 6 6 7 7; 8 8 9 9; 10 10 11 11; 12 12 13 13; 14 14 15 15; 16 16 17 17; 18 18 19 19;];
y1 = repmat([0 1 1 0], 10);
y1 = y1(:,1:4);
x1 = x1.';
y1 = y1.';

x2 = x1;
y2 = y1 + 2;

x3 = x1;
x3(:,11) = (x3(:,10) + 2);
y3 = y1;
y3(:,11) = [0 1 1 0];
y3 = y3 + 4;

x4 = x1;
y4 = y1 + 6;

x5 = x1;
x5(:,11) = [20 20 21 21];
x5(:,12) = [22 22 23 23];
x5(:,13) = [24 24 25 25];
y5 = y1;
y5(:,11:13) = y5(:,1:3);
y5 = y5 + 8;

x6 = x1;
x6(:,10) = [20 20 21 21];
x6(:,11) = [22 22 23 23];
x6(:,12) = [24 24 25 25];
y6 = y1;
y6(:,11:12) = y6(:,1:2);
y6 = y6 + 10;

x7 = x6;
y7 = y6 + 2;

x8 = x1;
y8 = y1 + 14;

x9 = x3;
y9 = y3 + 12;




tiledlayout(9,1)

ax1 = nexttile;
patch(x1, y1, Sfast_G1/max(Sfast_G1))
colormap(ax1,jet)
%colorbar
yticks([0.5])
yticklabels({'Sh Protr'})
xlim([0 25])
xticks([])
xticklabels([])

ax8 = nexttile; 
patch(x8, y8, Sfast_G8/max(Sfast_G8))
colormap(ax8,jet)
%colorbar
yticks([14.5])
yticklabels({'Sh Retr'})
xlim([0 25])
xticks([])
xticklabels([])

ax3 = nexttile; 
patch(x3, y3, Sfast_G3/max(Sfast_G3))
colormap(ax3,jet)
%colorbar
yticks([4.5])
yticklabels({'Sh Protr-El Fl'})
xlim([0 25])
xticks([])
xticklabels([])

ax9 = nexttile; 
patch(x9, y9, Sfast_G9/max(Sfast_G9))
colormap(ax9,jet)
%colorbar
yticks([16.5])
yticklabels({'Sh Retr-El Ex'})
xlim([0 25])
xticks([])
xticklabels([])

ax2 = nexttile; 
patch(x2, y2, Sfast_G2/max(Sfast_G2))
colormap(ax2,jet)
%colorbar
yticks([2.5])
yticklabels({'El Ex'})
xlim([0 25])
xticks([])
xticklabels([])

ax4 = nexttile; 
patch(x4, y4, Sfast_G4/max(Sfast_G4))
colormap(ax4,jet)
%colorbar
yticks([6.5])
yticklabels({'El Fl'})
xlim([0 25])
xticks([])
xticklabels([])

ax5 = nexttile; 
patch(x5, y5, Sfast_G5/max(Sfast_G5))
colormap(ax5,jet)
yticks([8.5])
yticklabels({'El Fl-Wr DFl'})
%colorbar
xlim([0 25])
xticks([])
xticklabels([])

ax7 = nexttile; 
patch(x7, y7, Sfast_G7/max(Sfast_G7))
colormap(ax7,jet)
%colorbar
yticks([12.5])
yticklabels({'Wr PFl'})
xlim([0 25])
xticks([])
xticklabels([])

ax6 = nexttile; 
patch(x6, y6, Sfast_G6/max(Sfast_G6))
colormap(ax6,jet)
%colorbar
yticks([10.5])
yticklabels({'Wr DFl'})
xlim([0 25])


xticks([0.5 2.5 4.5 6.5 8.5 10.5 12.5 14.5 16.5 18.5 20.5 22.5 24.5])
xticklabels({'LF0', 'Vmax/LF0', 'SO', 'PCSA', 'PA', 'a1/a1OV', 'a2/a2OV', 'phi1/phi1OV', 'phi2/phi2OV', 'R1', 'R2', 'a2VI', 'phi2VI'})

cbh = colorbar(ax6);
cbh.Layout.Tile = 'east';
% patch(x1, y1, Sfast_G1/max(Sfast_G1))
% patch(x2, y2, Sfast_G2/max(Sfast_G2))
% patch(x3, y3, Sfast_G3/max(Sfast_G3))
% patch(x4, y4, Sfast_G4/max(Sfast_G4))
% patch(x5, y5, Sfast_G5/max(Sfast_G5))
% patch(x6, y6, Sfast_G6/max(Sfast_G6))
% patch(x7, y7, Sfast_G7/max(Sfast_G7))
% patch(x8, y8, Sfast_G8/max(Sfast_G8))
% patch(x9, y9, Sfast_G9/max(Sfast_G9))
x0=50;
y0=50;
width=400;
height=500;
set(gcf,'position',[x0,y0,width,height])
exportgraphics(gcf,'Fig_SensitivityAnalysis.pdf','ContentType','vector')



