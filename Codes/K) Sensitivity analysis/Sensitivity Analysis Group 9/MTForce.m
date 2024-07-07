function [LT, LF, VF, NLF, NVF, FPE, FCE_L, FCE_V, FM, FMT] = ...
         MTForce(a, time, LMT, VMT, LT0, LF0, aV, Vmax, FM_max, PA, Max_LMT)
% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% MTForce calculates various biomechanical variables of a musculotendon 
% unit (MTU) during locomotion based on input parameters such as muscle 
% activation, musculotendon length, velocity, and muscle mechanical 
% parameters.
%
% Inputs:
% - a: Muscle activation level (dimensionless).
% - time: Time vector corresponding to the locomotion cycle.
% - LMT: Length of the musculotendon unit (meters).
% - VMT: Velocity of the musculotendon unit (meters/second).
% - LT0: Tendon slack length (meters).
% - LF0: Optimal muscle fascicle length (meters).
% - aV: Muscle viscosity coefficient.
% - Vmax: Maximum shortening velocity of the muscle (meters/second).
% - FM_max: Maximum isometric muscle force (Newtons).
% - PA: Pennation angle of the muscle (radians).
% - Max_LMT: Maximum length of the musculotendon unit during the cycle (meters).
%
% Outputs:
% - LT: Length of the tendon (meters).
% - LF: Length of the muscle fascicle (meters).
% - VF: Velocity of the muscle fascicle (meters/second).
% - NLF: Normalized length of the muscle fascicle (dimensionless).
% - NVF: Normalized velocity of the muscle fascicle (dimensionless).
% - FPE: Force of the muscle parallel elastic element (Newtons).
% - FCE_L: Muscle force-length relationship (dimensionless).
% - FCE_V: Muscle force-velocity relationship (dimensionless).
% - FM: Muscle fascicle force (Newtons).
% - FMT: Force of the musculotendon unit (Newtons).
%
% Note: the tendon slack length has been computed here

% Muscle maximal activation
Ku_max = 1;

% Tendon parameters (we do not apply tendon mechanics here)
KT1 = 0.1;
KT2 = 90;

% Muscle parallel elastic element parameters
KPE1 = 0.0075;
KPE2 = 11.6;

% Muscle and tendon viscosity coefficients
bT = 0.02; % N.s/m
bM = 0.02; % N.s/m

% Computation of tendon and muscle length/normalized length 
PA = PA .* (pi / 180); % Convert degrees to radians
LT = Max_LMT - 1.05 * LF0 * cos(PA);
LF = (LMT - LT) ./ cos(PA);
NLF = LF ./ LF0;

% Note: Only here since the tendon length is constant we consider LT0 as LT
LT0 = mean(LT);
NLT = LT ./ LT0;

% Muscle force-length contractile element parameters 
if (LT0 / LF0) <= 1
    rho = 6;
else
    rho = 3;
end
betha = 1.55;
omega = 0.55;

% Computation of muscle (fascicle) velocity/normalized velocity
VF = VMT / cos(PA);
NVF = VF ./ abs(Vmax);

% FPE: force of muscle parallel elastic element
if a==1
    FPE = 0;
else
    FPE = (KPE1 / KPE2) * (exp(KPE2 .* ((LF ./ LF0) - 1)) - 1);
end 

% FCE_L: Muscle force-length relationship
FCE_L = exp(-abs((NLF .^ betha - 1) ./ omega) .^ rho);

% FCE_V: Muscle force-velocity relationship
b0 = Vmax * 0.8 / (aV + 1);
b1 = (1.8 * (Vmax + b0) - b0) / Vmax;
FCE_V = zeros(size(NVF)); % Initialize FCE_V
for i = 1:size(NVF, 1)
    if NVF(i, 1) >= 0  
        FCE_V(i, 1) = ((b0 / Vmax) + b1 * NVF(i, 1)) / ((b0 / Vmax) + NVF(i, 1));
    else
        b = (aV / FM_max) * Vmax;
        FCE_V(i, 1) = ((FM_max * b - aV * VF(i, 1)) / (VF(i, 1) + b)) / FM_max;
    end
end

% FM: Muscle (fascicle) force
FM = FM_max .* (FCE_L .* FCE_V .* Ku_max .* a + FPE + bM .* VF);

% FMT: force of Musculotendon unit
FMT = FM .* cos(PA);

end
