% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% A_MusculoskeletalSystem_SagittalPlane.m should be run first
% This script performs 2D and 3D visualization and analysis of selected muscle.
% It includes rotations to align muscle attachment points, joint centers, and other anatomical points on the computed sagittal plane with the XY plane.
% The script computes muscle lengths and geometrical parameters, and estimates segment lengths and angles.
% The script creates a structure called MusculoskeletalData for further analysis and interpretation.

h = gobjects(0); 

% Select the muscle name to be shown on 2D and 3D
selectedRow = selectMuscle();
MuscleNumber = selectedRow; 


%% Final desired values after rotating sagital plane to XY plane
% SIP_R; EIP_R; WIP_R     % Shoulder Intersection Point, Elbow Intersection Point; Wrist Intersection Point
% PMs_R                   % Projection of Mean_svs_sca on sagittal plane
% PMf_R                   % Projection of Mean_flmcp_fmmcp on sagittal plane
% Origins_2D_R            % Muscle origin points
% Insertions_2D_R         % Muscle insertion points

% Lscp_2D_MM        % Length of scapula from 2D in the main model (from PMs_R to SIP_R)
% Luarm_2D_MM       % Length of upper arm from 2D in the main model (from SIP_R to EIP_R)
% Lfarm_2D_MM       % Length of fore arm from 2D in the main model (from EIP_R to WIP_R)
% Lcar_2D_MM        % Length of carpals from 2D in the main model (from WIP_R to PMf_R)



%% Data on 3D
% JC                      % Shoulder Center Point, Elbow Center Point; Wrist Center Point
% Mean_svs_sca            % Mean of svs & sca 
% Mean_flmcp_fmmcp        % Projection of Mean of flmcp & fmmcp 
% Origins                 % Muscle origin points
% Insertions              % Muscle insertion points

% Lscp_3D_MM              % Length of scapula from 3D in the main model (from Mean_svs_sca to JC(1,:))
% Luarm_3D_MM             % Length of upper arm from 3D in the main model (from JC(1,:) to JC(2,:))
% Lfarm_3D_MM             % Length of fore arm from 3D in the main model (from JC(2,:) to JC(3,:))
% Lcar_3D_MM              % Length of carpals from 3D in the main model (from JC(3,:) to Mean_flmcp_fmmcp)


%% Data on the 2D sagittal plane before rotation (this 2D sagittal plane is in 3D space)
% SIP; EIP; WIP     % Shoulder Intersection Point, Elbow Intersection Point; Wrist Intersection Point
% PMs               % Projection of Mean_svs_sca on sagittal plane
% PMf               % Projection of Mean_flmcp_fmmcp on sagittal plane
% Origins_2D        % Muscle origin points
% Insertions_2D     % Muscle insertion points





%% Rotating points on the 2D sagittal plane to align with the XY plane
SIPO = SIP; 
EIPO = EIP;
WIPO = WIP;     
PMsO = PMs;            
PMfO = PMf;               
Origins_2DO = Origins_2D;        
Insertions_2DO = Insertions_2D;     



[M, I] = max(abs(normal_MAP));
if I==1
    AI = [1; 0; 0]; % Axis of interest
elseif I==2
    AI = [0; 1; 0]; % Axis of interest    
else
    AI = [0; 0; 1]; % Axis of interest     
end


u = normal_MAP;
v = AI;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
Theta_normalMAP_AI = real(acosd(CosTheta));

% Axis of rotation
AR = cross(u, v)/norm(cross(u, v));

% Rotation & trasfer
SIPOU = SIPO/norm(SIPO);
SIP2 = rotVecAroundArbAxis(SIPOU, AR', Theta_normalMAP_AI);
SIP_R = SIP2*norm(SIPO);
SIP_R(:,I) = SIP_R(:,I)*0; % trasfer


EIPOU = EIPO/norm(EIPO);
EIP2 = rotVecAroundArbAxis(EIPOU, AR', Theta_normalMAP_AI);
EIP_R = EIP2*norm(EIPO);
EIP_R(:,I) = EIP_R(:,I)*0; % trasfer

WIPOU = WIPO/norm(WIPO);
WIP2 = rotVecAroundArbAxis(WIPOU, AR', Theta_normalMAP_AI);
WIP_R = WIP2*norm(WIPO);
WIP_R(:,I) = WIP_R(:,I)*0; % trasfer

PMsOU = PMsO/norm(PMsO);
PMs2 = rotVecAroundArbAxis(PMsOU, AR', Theta_normalMAP_AI);
PMs_R = PMs2*norm(PMsO);
PMs_R(:,I) = PMs_R(:,I)*0; % trasfer

PMfOU = PMfO/norm(PMfO);
PMf2 = rotVecAroundArbAxis(PMfOU, AR', Theta_normalMAP_AI);
PMf_R = PMf2*norm(PMfO);
PMf_R(:,I) = PMf_R(:,I)*0; % trasfer

Origins_2DOU = Origins_2DO/norm(Origins_2DO);
Origins_2D2 = rotVecAroundArbAxis(Origins_2DOU, AR', Theta_normalMAP_AI);
Origins_2D_R = Origins_2D2*norm(Origins_2DO);
Origins_2D_R(:,I) = Origins_2D_R(:,I)*0; % trasfer

Insertions_2DOU = Insertions_2DO/norm(Insertions_2DO);
Insertions_2D2 = rotVecAroundArbAxis(Insertions_2DOU, AR', Theta_normalMAP_AI);
Insertions_2D_R = Insertions_2D2*norm(Insertions_2DO);
Insertions_2D_R(:,I) = Insertions_2D_R(:,I)*0; % trasfer

MAP_R = [Origins_2D_R; Insertions_2D_R];
meanMAP_R = mean(MAP_R);




% Joint coordinates on 2 dimensional plane
JC2D_R = [SIP_R; EIP_R; WIP_R];


% Joint centers 3D & 2D
JC3D = [Mean_svs_sca; JC; Mean_flmcp_fmmcp];
JC2D = [PMs_R; JC2D_R; PMf_R];



%% Estimation of segement length in 3D & 2D

Lscp_3D_MM = mean(sqrt(sum((Mean_svs_sca-JC(1,:)).^2,2)));
Luarm_3D_MM = mean(sqrt(sum((JC(1,:)-JC(2,:)).^2,2)));
Lfarm_3D_MM = mean(sqrt(sum((JC(2,:)-JC(3,:)).^2,2)));
Lcar_3D_MM = mean(sqrt(sum((JC(3,:)-Mean_flmcp_fmmcp).^2,2)));



Lscp_2D_MM = mean(sqrt(sum((PMs-SIP).^2,2)));
Luarm_2D_MM = mean(sqrt(sum((SIP-EIP).^2,2)));
Lfarm_2D_MM = mean(sqrt(sum((EIP-WIP).^2,2)));
Lcar_2D_MM = mean(sqrt(sum((WIP-PMf).^2,2)));




%% Segment angles in 2D
%--------Scapula angle--------------------------------------------------------------------------
Horizen_2D = [SIP_R(1,1)+1, SIP_R(1,2), SIP_R(1,3)];
v1_2D = Horizen_2D - SIP_R;
v2_2D = PMs_R - SIP_R;

for i=1:size(v1_2D,1)  
CosTheta = max(min(dot(v1_2D(i,:),v2_2D(i,:))/(norm(v1_2D(i,:))*norm(v2_2D(i,:))),1),-1);
q1_MS_2D(i,1) = real(acos(CosTheta));

if SIP_R(i,2)<PMs_R(i,2)
   q1_MS_2D(i,1) = q1_MS_2D(i,1)+pi;  
end
end

%--------Humerus angle--------------------------------------------------------------------------
Horizen_2D = [EIP_R(1,1)+1, EIP_R(1,2), EIP_R(1,3)];
v1_2D = Horizen_2D - EIP_R;
v2_2D = SIP_R - EIP_R;

for i=1:size(v1_2D,1)  
CosTheta = max(min(dot(v1_2D(i,:),v2_2D(i,:))/(norm(v1_2D(i,:))*norm(v2_2D(i,:))),1),-1);
q2_MS_2D(i,1) = real(acos(CosTheta));

if EIP_R(i,2)<SIP_R(i,2)
   q2_MS_2D(i,1) = q2_MS_2D(i,1)+pi;  
end

end

%--------Radius angle--------------------------------------------------------------------------
Horizen_2D = [WIP_R(1,1)+1, WIP_R(1,2), WIP_R(1,3)];
v1_2D = Horizen_2D - WIP_R;
v2_2D = EIP_R - WIP_R;

for i=1:size(v1_2D,1)  
CosTheta = max(min(dot(v1_2D(i,:),v2_2D(i,:))/(norm(v1_2D(i,:))*norm(v2_2D(i,:))),1),-1);
q3_MS_2D(i,1) = real(acos(CosTheta));

if WIP_R(i,2)<EIP_R(i,2)
   q3_MS_2D(i,1) = q3_MS_2D(i,1)+pi;  
end

end

%--------Metatarsus angle--------------------------------------------------------------------------
Horizen_2D = [PMf_R(1,1)+1, PMf_R(1,2), PMf_R(1,3)];
v1_2D = Horizen_2D - PMf_R;
v2_2D = WIP_R - PMf_R;

for i=1:size(v1_2D,1)  
CosTheta = max(min(dot(v1_2D(i,:),v2_2D(i,:))/(norm(v1_2D(i,:))*norm(v2_2D(i,:))),1),-1);
q4_MS_2D(i,1) = real(acos(CosTheta));

if PMf_R(i,2)<WIP_R(i,2)
   q4_MS_2D(i,1) = q4_MS_2D(i,1)+pi;  
end

end


%% Joint angles in 2D
SHLa_MS_2D = pi - q1_MS_2D + q2_MS_2D;
ELBa_MS_2D = pi + q2_MS_2D - q3_MS_2D;
WRTa_MS_2D = pi + q3_MS_2D - q4_MS_2D;



%% Computation of muscle lengths in 2D and 3D

for k=1:size(Origins,1)
Muscle_Length_3D(k,1) = sqrt(sum((Origins(k,:) - Insertions(k,:)).^2));
Muscle_Length_2D(k,1) = sqrt(sum((Origins_2D_R(k,:) - Insertions_2D_R(k,:)).^2));
end

%% Computation of muscle geometrical parameters
MuscleGeometricParameters = [];


%% --------------------------- Acromiodeltoideus(1->2) ------------------------------------
k = 1;
JCN = 2; % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_Acromiodeltoideus = -1;
f2_Acromiodeltoideus = nan;
n1_Acromiodeltoideus = 1;
n2_Acromiodeltoideus = nan;
% **** determine R1 & R2 

a1_Acromiodeltoideus = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_Acromiodeltoideus = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_Acromiodeltoideus = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_Acromiodeltoideus = Da2*real(acos(CosTheta));


R1_Acromiodeltoideus = 0.002;
R2_Acromiodeltoideus = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_Acromiodeltoideus, a2_Acromiodeltoideus,...
                                  phi1_Acromiodeltoideus, phi2_Acromiodeltoideus,...
                                  R1_Acromiodeltoideus, R2_Acromiodeltoideus,...
                                  f1_Acromiodeltoideus, f2_Acromiodeltoideus, ...
                                  n1_Acromiodeltoideus, n2_Acromiodeltoideus];
                              
                              
%% --------------------------- Anconeus (3->4)------------------------------------
k = 2;
JCN = 3; % Joint center number
Da1 = 1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_Anconeus = 1;
f2_Anconeus = nan;
n1_Anconeus = -1;
n2_Anconeus = nan;
% **** determine R1 & R2 

a1_Anconeus = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_Anconeus = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_Anconeus = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_Anconeus = Da2*real(acos(CosTheta));


R1_Anconeus = a2_Anconeus;
R2_Anconeus = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_Anconeus, a2_Anconeus,...
                                  phi1_Anconeus, phi2_Anconeus,...
                                  R1_Anconeus, R2_Anconeus,...
                                  f1_Anconeus, f2_Anconeus, ...
                                  n1_Anconeus, n2_Anconeus];
                              

                              
%% --------------------------- Abductor Pollicis Longus O-V(5->6) ------------------------------------
k = 3;
JCN = 3;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_AbductorPollicisLongusOW = nan;
f2_AbductorPollicisLongusOW = nan;
n1_AbductorPollicisLongusOW = nan;
n2_AbductorPollicisLongusOW = nan;
% **** determine R1 & R2 


a1_AbductorPollicisLongusOW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_AbductorPollicisLongusOW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_AbductorPollicisLongusOW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_AbductorPollicisLongusOW = Da2*real(acos(CosTheta));


R1_AbductorPollicisLongusOW = nan;
R2_AbductorPollicisLongusOW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_AbductorPollicisLongusOW, a2_AbductorPollicisLongusOW,...
                                  phi1_AbductorPollicisLongusOW, phi2_AbductorPollicisLongusOW,...
                                  R1_AbductorPollicisLongusOW, R2_AbductorPollicisLongusOW,...
                                  f1_AbductorPollicisLongusOW, f2_AbductorPollicisLongusOW, ...
                                  n1_AbductorPollicisLongusOW, n2_AbductorPollicisLongusOW];
                              
MuscleGeometricParameters(k,1:10) = nan; % This muscle does not contribute in the 2D flexion-extension movement                             
                              
                              
%--------------------------- Abductor Pollicis Longus V-I(6->7)-----------------------------------------
k = 4;
JCN = 4;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_AbductorPollicisLongusWI = -1;
f2_AbductorPollicisLongusWI = nan;
n1_AbductorPollicisLongusWI = -1;
n2_AbductorPollicisLongusWI = nan;
% **** determine R1 & R2 

a1_AbductorPollicisLongusWI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_AbductorPollicisLongusWI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_AbductorPollicisLongusWI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_AbductorPollicisLongusWI = Da2*real(acos(CosTheta));

R1_AbductorPollicisLongusWI = a1_AbductorPollicisLongusWI;
R2_AbductorPollicisLongusWI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_AbductorPollicisLongusWI, a2_AbductorPollicisLongusWI,...
                                  phi1_AbductorPollicisLongusWI, phi2_AbductorPollicisLongusWI,...
                                  R1_AbductorPollicisLongusWI, R2_AbductorPollicisLongusWI,...
                                  f1_AbductorPollicisLongusWI, f2_AbductorPollicisLongusWI, ...
                                  n1_AbductorPollicisLongusWI, n2_AbductorPollicisLongusWI];
                              
MuscleGeometricParameters(k,1:10) = nan; % This muscle does not contribute in the 2D flexion-extension movement                             
                                       
                              
%% --------------------------- Biceps Brachii(8->9) ------------------------------------
k = 5;
PJCN = 2; % Proximal joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_BicepsBrachii = -1;
f2_BicepsBrachii = -1;
n1_BicepsBrachii = 1;
n2_BicepsBrachii = -1;
% **** determine R1 & R2 


a1_BicepsBrachii = sqrt(sum((Origins_2D_R(k,:) - JC2D(PJCN,:)).^2));
a2_BicepsBrachii = sqrt(sum((Insertions_2D_R(k,:) - JC2D(PJCN+1,:)).^2));

PO = Origins_2D_R(k,:);  % Origin
PT =  JC2D(PJCN-1,:);    % Top Point
PC1 = JC2D(PJCN,:);      % First Center
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_BicepsBrachii = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);     % Insertion
PB =  JC2D(PJCN+2,:);          % Bottom Point
PC2 = JC2D(PJCN+1,:);          % Second Center
u = PC2-PI;
v = PC2-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_BicepsBrachii = Da2*real(acos(CosTheta));

R1_BicepsBrachii = 0.01;
R2_BicepsBrachii = 0.005;


% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_BicepsBrachii, a2_BicepsBrachii,...
                                  phi1_BicepsBrachii, phi2_BicepsBrachii,...
                                  R1_BicepsBrachii, R2_BicepsBrachii,...
                                  f1_BicepsBrachii, f2_BicepsBrachii, ...
                                  n1_BicepsBrachii, n2_BicepsBrachii];
                              
                              
%% --------------------------- Brachioradialis O-V(10->11) ------------------------------------
k = 6;
JCN = 2;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_BrachioradialisOW = nan;
f2_BrachioradialisOW = nan;
n1_BrachioradialisOW = nan;
n2_BrachioradialisOW = nan;
% **** determine R1 & R2 


a1_BrachioradialisOW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_BrachioradialisOW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_BrachioradialisOW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_BrachioradialisOW = Da2*real(acos(CosTheta));


R1_BrachioradialisOW = nan;
R2_BrachioradialisOW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_BrachioradialisOW, a2_BrachioradialisOW,...
                                  phi1_BrachioradialisOW, phi2_BrachioradialisOW,...
                                  R1_BrachioradialisOW, R2_BrachioradialisOW,...
                                  f1_BrachioradialisOW, f2_BrachioradialisOW, ...
                                  n1_BrachioradialisOW, n2_BrachioradialisOW];
                              
                              
                              
%--------------------------- Brachioradialis V-I(11->12)-----------------------------------------
k = 7;
JCN = 3;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_BrachioradialisWI = -1;
f2_BrachioradialisWI = nan;
n1_BrachioradialisWI = -1;
n2_BrachioradialisWI = nan;
% **** determine R1 & R2 

a1_BrachioradialisWI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_BrachioradialisWI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_BrachioradialisWI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_BrachioradialisWI = Da2*real(acos(CosTheta));

R1_BrachioradialisWI = a1_BrachioradialisWI;
R2_BrachioradialisWI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_BrachioradialisWI, a2_BrachioradialisWI,...
                                  phi1_BrachioradialisWI, phi2_BrachioradialisWI,...
                                  R1_BrachioradialisWI, R2_BrachioradialisWI,...
                                  f1_BrachioradialisWI, f2_BrachioradialisWI, ...
                                  n1_BrachioradialisWI, n2_BrachioradialisWI];
                              
                              
%% --------------------------- Brachialis(13->14) ------------------------------------
k = 8;
JCN = 3; % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_Brachialis = -1;
f2_Brachialis = nan;
n1_Brachialis = -1;
n2_Brachialis = nan;
% **** determine R1 & R2 

a1_Brachialis = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_Brachialis = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_Brachialis = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_Brachialis = Da2*real(acos(CosTheta));


R1_Brachialis = a2_Brachialis;
R2_Brachialis = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_Brachialis, a2_Brachialis,...
                                  phi1_Brachialis, phi2_Brachialis,...
                                  R1_Brachialis, R2_Brachialis,...
                                  f1_Brachialis, f2_Brachialis, ...
                                  n1_Brachialis, n2_Brachialis];
                              
                              

%% --------------------------- Coracobrachialis(15->16) ------------------------------------
k = 9;
JCN = 2;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 =  1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_Coracobrachialis = -1;
f2_Coracobrachialis = nan;
n1_Coracobrachialis = 1;
n2_Coracobrachialis = nan;
% **** determine R1 & R2 

a1_Coracobrachialis = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_Coracobrachialis = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_Coracobrachialis = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_Coracobrachialis = Da2*real(acos(CosTheta));


R1_Coracobrachialis = 0.005;
R2_Coracobrachialis = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_Coracobrachialis, a2_Coracobrachialis,...
                                  phi1_Coracobrachialis, phi2_Coracobrachialis,...
                                  R1_Coracobrachialis, R2_Coracobrachialis,...
                                  f1_Coracobrachialis, f2_Coracobrachialis, ...
                                  n1_Coracobrachialis, n2_Coracobrachialis];
                              



%% --------------------------- Extensor Carpi Radialis O-V(17->18) ------------------------------------
k = 10;
PJCN = 3; % Proximal joint center number
Da1 = 1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorCarpiRadialisOW = -1;
f2_ExtensorCarpiRadialisOW = -1;
n1_ExtensorCarpiRadialisOW = -1;
n2_ExtensorCarpiRadialisOW = -1;
% **** determine R1 & R2 


a1_ExtensorCarpiRadialisOW = sqrt(sum((Origins_2D_R(k,:) - JC2D(PJCN,:)).^2));
a2_ExtensorCarpiRadialisOW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(PJCN+1,:)).^2));

PO = Origins_2D_R(k,:);  % Origin
PT =  JC2D(PJCN-1,:);    % Top Point
PC1 = JC2D(PJCN,:);      % First Center
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorCarpiRadialisOW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);     % Insertion
PB =  JC2D(PJCN+2,:);          % Bottom Point
PC2 = JC2D(PJCN+1,:);          % Second Center
u = PC2-PI;
v = PC2-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorCarpiRadialisOW = Da2*real(acos(CosTheta));

R1_ExtensorCarpiRadialisOW = 0.005;
R2_ExtensorCarpiRadialisOW = a2_ExtensorCarpiRadialisOW;


% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorCarpiRadialisOW, a2_ExtensorCarpiRadialisOW,...
                                  phi1_ExtensorCarpiRadialisOW, phi2_ExtensorCarpiRadialisOW,...
                                  R1_ExtensorCarpiRadialisOW, R2_ExtensorCarpiRadialisOW,...
                                  f1_ExtensorCarpiRadialisOW, f2_ExtensorCarpiRadialisOW, ...
                                  n1_ExtensorCarpiRadialisOW, n2_ExtensorCarpiRadialisOW];
                              
                              
                              
%--------------------------- Extensor Carpi Radialis V-I(18->19) ------------------------------------
k = 11;
JCN = 4;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorCarpiRadialisWI = nan;
f2_ExtensorCarpiRadialisWI = nan;
n1_ExtensorCarpiRadialisWI = nan;
n2_ExtensorCarpiRadialisWI = nan;
% **** determine R1 & R2 

a1_ExtensorCarpiRadialisWI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorCarpiRadialisWI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN+1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorCarpiRadialisWI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorCarpiRadialisWI = Da2*real(acos(CosTheta));

R1_ExtensorCarpiRadialisWI = nan;
R2_ExtensorCarpiRadialisWI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorCarpiRadialisWI, a2_ExtensorCarpiRadialisWI,...
                                  phi1_ExtensorCarpiRadialisWI, phi2_ExtensorCarpiRadialisWI,...
                                  R1_ExtensorCarpiRadialisWI, R2_ExtensorCarpiRadialisWI,...
                                  f1_ExtensorCarpiRadialisWI, f2_ExtensorCarpiRadialisWI, ...
                                  n1_ExtensorCarpiRadialisWI, n2_ExtensorCarpiRadialisWI];
                              
                              
    
%% --------------------------- Extensor Carpi Ulnaris O-V(20->21) ------------------------------------
k = 12;
JCN = 3;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorCarpiUlnarisOW = nan;
f2_ExtensorCarpiUlnarisOW = nan;
n1_ExtensorCarpiUlnarisOW = nan;
n2_ExtensorCarpiUlnarisOW = nan;
% **** determine R1 & R2 


a1_ExtensorCarpiUlnarisOW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorCarpiUlnarisOW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorCarpiUlnarisOW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorCarpiUlnarisOW = Da2*real(acos(CosTheta));


R1_ExtensorCarpiUlnarisOW = nan;
R2_ExtensorCarpiUlnarisOW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorCarpiUlnarisOW, a2_ExtensorCarpiUlnarisOW,...
                                  phi1_ExtensorCarpiUlnarisOW, phi2_ExtensorCarpiUlnarisOW,...
                                  R1_ExtensorCarpiUlnarisOW, R2_ExtensorCarpiUlnarisOW,...
                                  f1_ExtensorCarpiUlnarisOW, f2_ExtensorCarpiUlnarisOW, ...
                                  n1_ExtensorCarpiUlnarisOW, n2_ExtensorCarpiUlnarisOW];
                                             
MuscleGeometricParameters(k,1:10) = nan; % This muscle does not contribute in the 2D flexion-extension movement                             
                                
                              
%--------------------------- Extensor Carpi Ulnaris V-I(21->22)-----------------------------------------
k = 13;
JCN = 4;   % Joint center number
Da1 = 1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorCarpiUlnarisWI = 1;
f2_ExtensorCarpiUlnarisWI = nan;
n1_ExtensorCarpiUlnarisWI = -1;
n2_ExtensorCarpiUlnarisWI = nan;
% **** determine R1 & R2 

a1_ExtensorCarpiUlnarisWI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorCarpiUlnarisWI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorCarpiUlnarisWI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorCarpiUlnarisWI = Da2*real(acos(CosTheta));

R1_ExtensorCarpiUlnarisWI = a1_ExtensorCarpiUlnarisWI;
R2_ExtensorCarpiUlnarisWI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorCarpiUlnarisWI, a2_ExtensorCarpiUlnarisWI,...
                                  phi1_ExtensorCarpiUlnarisWI, phi2_ExtensorCarpiUlnarisWI,...
                                  R1_ExtensorCarpiUlnarisWI, R2_ExtensorCarpiUlnarisWI,...
                                  f1_ExtensorCarpiUlnarisWI, f2_ExtensorCarpiUlnarisWI, ...
                                  n1_ExtensorCarpiUlnarisWI, n2_ExtensorCarpiUlnarisWI];                              
                              
MuscleGeometricParameters(k,1:10) = nan; % This muscle does not contribute in the 2D flexion-extension movement      


%% --------------------------- Extensor digitorum communis 2 O-V(23->24) ------------------------------------
k = 14;
JCN = 3;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumCommunis2OW = -1;
f2_ExtensorDigitorumCommunis2OW = nan;
n1_ExtensorDigitorumCommunis2OW = -1;
n2_ExtensorDigitorumCommunis2OW = nan;
% **** determine R1 & R2 


a1_ExtensorDigitorumCommunis2OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumCommunis2OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumCommunis2OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumCommunis2OW = Da2*real(acos(CosTheta));


R1_ExtensorDigitorumCommunis2OW = 0.005;
R2_ExtensorDigitorumCommunis2OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumCommunis2OW, a2_ExtensorDigitorumCommunis2OW,...
                                  phi1_ExtensorDigitorumCommunis2OW, phi2_ExtensorDigitorumCommunis2OW,...
                                  R1_ExtensorDigitorumCommunis2OW, R2_ExtensorDigitorumCommunis2OW,...
                                  f1_ExtensorDigitorumCommunis2OW, f2_ExtensorDigitorumCommunis2OW, ...
                                  n1_ExtensorDigitorumCommunis2OW, n2_ExtensorDigitorumCommunis2OW];
                              
                              
                              
%--------------------------- Extensor digitorum communis 2 V-I(24->25) ------------------------------------
k = 15;
JCN = 4;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumCommunis2WI = -1;
f2_ExtensorDigitorumCommunis2WI = nan;
n1_ExtensorDigitorumCommunis2WI = -1;
n2_ExtensorDigitorumCommunis2WI = nan;
% **** determine R1 & R2 

a1_ExtensorDigitorumCommunis2WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumCommunis2WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumCommunis2WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumCommunis2WI = Da2*real(acos(CosTheta));

R1_ExtensorDigitorumCommunis2WI = a1_ExtensorDigitorumCommunis2WI;
R2_ExtensorDigitorumCommunis2WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumCommunis2WI, a2_ExtensorDigitorumCommunis2WI,...
                                  phi1_ExtensorDigitorumCommunis2WI, phi2_ExtensorDigitorumCommunis2WI,...
                                  R1_ExtensorDigitorumCommunis2WI, R2_ExtensorDigitorumCommunis2WI,...
                                  f1_ExtensorDigitorumCommunis2WI, f2_ExtensorDigitorumCommunis2WI, ...
                                  n1_ExtensorDigitorumCommunis2WI, n2_ExtensorDigitorumCommunis2WI];
                              
                              
     

%% --------------------------- Extensor digitorum communis 3 O-V(26->27) ------------------------------------
k = 16;
JCN = 3;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumcommunis3OW = -1;
f2_ExtensorDigitorumcommunis3OW = nan;
n1_ExtensorDigitorumcommunis3OW = -1;
n2_ExtensorDigitorumcommunis3OW = nan;
% **** determine R1 & R2 


a1_ExtensorDigitorumcommunis3OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumcommunis3OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumcommunis3OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumcommunis3OW = Da2*real(acos(CosTheta));


R1_ExtensorDigitorumcommunis3OW = 0.005;
R2_ExtensorDigitorumcommunis3OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumcommunis3OW, a2_ExtensorDigitorumcommunis3OW,...
                                  phi1_ExtensorDigitorumcommunis3OW, phi2_ExtensorDigitorumcommunis3OW,...
                                  R1_ExtensorDigitorumcommunis3OW, R2_ExtensorDigitorumcommunis3OW,...
                                  f1_ExtensorDigitorumcommunis3OW, f2_ExtensorDigitorumcommunis3OW, ...
                                  n1_ExtensorDigitorumcommunis3OW, n2_ExtensorDigitorumcommunis3OW];
                              
                              
                              
%--------------------------- Extensor digitorum communis 3 V-I(27->28) ------------------------------------
k = 17;
JCN = 4;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumcommunis3WI = -1;
f2_ExtensorDigitorumcommunis3WI = nan;
n1_ExtensorDigitorumcommunis3WI = -1;
n2_ExtensorDigitorumcommunis3WI = nan;
% **** determine R1 & R2 

a1_ExtensorDigitorumcommunis3WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumcommunis3WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumcommunis3WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumcommunis3WI = Da2*real(acos(CosTheta));

R1_ExtensorDigitorumcommunis3WI = a1_ExtensorDigitorumcommunis3WI;
R2_ExtensorDigitorumcommunis3WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumcommunis3WI, a2_ExtensorDigitorumcommunis3WI,...
                                  phi1_ExtensorDigitorumcommunis3WI, phi2_ExtensorDigitorumcommunis3WI,...
                                  R1_ExtensorDigitorumcommunis3WI, R2_ExtensorDigitorumcommunis3WI,...
                                  f1_ExtensorDigitorumcommunis3WI, f2_ExtensorDigitorumcommunis3WI, ...
                                  n1_ExtensorDigitorumcommunis3WI, n2_ExtensorDigitorumcommunis3WI];
                              
                              
%% --------------------------- Extensor digitorum communis 4 O-V(29->30) ------------------------------------
k = 18;
JCN = 3;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumcommunis4OW = -1;
f2_ExtensorDigitorumcommunis4OW = nan;
n1_ExtensorDigitorumcommunis4OW = -1;
n2_ExtensorDigitorumcommunis4OW = nan;
% **** determine R1 & R2 


a1_ExtensorDigitorumcommunis4OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumcommunis4OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumcommunis4OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumcommunis4OW = Da2*real(acos(CosTheta));


R1_ExtensorDigitorumcommunis4OW = 0.005;
R2_ExtensorDigitorumcommunis4OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumcommunis4OW, a2_ExtensorDigitorumcommunis4OW,...
                                  phi1_ExtensorDigitorumcommunis4OW, phi2_ExtensorDigitorumcommunis4OW,...
                                  R1_ExtensorDigitorumcommunis4OW, R2_ExtensorDigitorumcommunis4OW,...
                                  f1_ExtensorDigitorumcommunis4OW, f2_ExtensorDigitorumcommunis4OW, ...
                                  n1_ExtensorDigitorumcommunis4OW, n2_ExtensorDigitorumcommunis4OW];
                              
                              
                              
%--------------------------- Extensor digitorum communis 4 V-I(30->31) ------------------------------------
k = 19;
JCN = 4;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumcommunis4WI = -1;
f2_ExtensorDigitorumcommunis4WI = nan;
n1_ExtensorDigitorumcommunis4WI = -1;
n2_ExtensorDigitorumcommunis4WI = nan;
% **** determine R1 & R2 

a1_ExtensorDigitorumcommunis4WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumcommunis4WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumcommunis4WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumcommunis4WI = Da2*real(acos(CosTheta));

R1_ExtensorDigitorumcommunis4WI = a1_ExtensorDigitorumcommunis4WI;
R2_ExtensorDigitorumcommunis4WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumcommunis4WI, a2_ExtensorDigitorumcommunis4WI,...
                                  phi1_ExtensorDigitorumcommunis4WI, phi2_ExtensorDigitorumcommunis4WI,...
                                  R1_ExtensorDigitorumcommunis4WI, R2_ExtensorDigitorumcommunis4WI,...
                                  f1_ExtensorDigitorumcommunis4WI, f2_ExtensorDigitorumcommunis4WI, ...
                                  n1_ExtensorDigitorumcommunis4WI, n2_ExtensorDigitorumcommunis4WI];
                              
                              
                              
%% --------------------------- Extensor digitorum communis 5 O-V(32->33) ------------------------------------
k = 20;
JCN = 3;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumcommunis5OW = -1;
f2_ExtensorDigitorumcommunis5OW = nan;
n1_ExtensorDigitorumcommunis5OW = -1;
n2_ExtensorDigitorumcommunis5OW = nan;
% **** determine R1 & R2 


a1_ExtensorDigitorumcommunis5OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumcommunis5OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumcommunis5OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumcommunis5OW = Da2*real(acos(CosTheta));


R1_ExtensorDigitorumcommunis5OW = 0.005;
R2_ExtensorDigitorumcommunis5OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumcommunis5OW, a2_ExtensorDigitorumcommunis5OW,...
                                  phi1_ExtensorDigitorumcommunis5OW, phi2_ExtensorDigitorumcommunis5OW,...
                                  R1_ExtensorDigitorumcommunis5OW, R2_ExtensorDigitorumcommunis5OW,...
                                  f1_ExtensorDigitorumcommunis5OW, f2_ExtensorDigitorumcommunis5OW, ...
                                  n1_ExtensorDigitorumcommunis5OW, n2_ExtensorDigitorumcommunis5OW];
                              
                              
                              
%--------------------------- Extensor digitorum communis 5 V-I(33->34) ------------------------------------
k = 21;
JCN = 4;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumcommunis5WI = -1;
f2_ExtensorDigitorumcommunis5WI = nan;
n1_ExtensorDigitorumcommunis5WI = -1;
n2_ExtensorDigitorumcommunis5WI = nan;
% **** determine R1 & R2 

a1_ExtensorDigitorumcommunis5WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumcommunis5WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumcommunis5WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumcommunis5WI = Da2*real(acos(CosTheta));

R1_ExtensorDigitorumcommunis5WI = a1_ExtensorDigitorumcommunis5WI;
R2_ExtensorDigitorumcommunis5WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumcommunis5WI, a2_ExtensorDigitorumcommunis5WI,...
                                  phi1_ExtensorDigitorumcommunis5WI, phi2_ExtensorDigitorumcommunis5WI,...
                                  R1_ExtensorDigitorumcommunis5WI, R2_ExtensorDigitorumcommunis5WI,...
                                  f1_ExtensorDigitorumcommunis5WI, f2_ExtensorDigitorumcommunis5WI, ...
                                  n1_ExtensorDigitorumcommunis5WI, n2_ExtensorDigitorumcommunis5WI];
                              
                              


%% --------------------------- Extensor digitorum lateralis 2 O-V(35->36) ------------------------------------
k = 22;
JCN = 3;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumLateralis2OW = -1;
f2_ExtensorDigitorumLateralis2OW = nan;
n1_ExtensorDigitorumLateralis2OW = -1;
n2_ExtensorDigitorumLateralis2OW = nan;
% **** determine R1 & R2 


a1_ExtensorDigitorumLateralis2OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumLateralis2OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumLateralis2OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumLateralis2OW = Da2*real(acos(CosTheta));


R1_ExtensorDigitorumLateralis2OW = a1_ExtensorDigitorumLateralis2OW;
R2_ExtensorDigitorumLateralis2OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumLateralis2OW, a2_ExtensorDigitorumLateralis2OW,...
                                  phi1_ExtensorDigitorumLateralis2OW, phi2_ExtensorDigitorumLateralis2OW,...
                                  R1_ExtensorDigitorumLateralis2OW, R2_ExtensorDigitorumLateralis2OW,...
                                  f1_ExtensorDigitorumLateralis2OW, f2_ExtensorDigitorumLateralis2OW, ...
                                  n1_ExtensorDigitorumLateralis2OW, n2_ExtensorDigitorumLateralis2OW];
                              
MuscleGeometricParameters(k,1:10) = nan; % This muscle does not contribute in the 2D flexion-extension movement      
                              
                              
%--------------------------- Extensor digitorum lateralis 2 V-I(36->37) ------------------------------------
k = 23;
JCN = 4;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumLateralis2WI = -1;
f2_ExtensorDigitorumLateralis2WI = nan;
n1_ExtensorDigitorumLateralis2WI = -1;
n2_ExtensorDigitorumLateralis2WI = nan;
% **** determine R1 & R2 

a1_ExtensorDigitorumLateralis2WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumLateralis2WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumLateralis2WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumLateralis2WI = Da2*real(acos(CosTheta));

R1_ExtensorDigitorumLateralis2WI = a1_ExtensorDigitorumLateralis2WI;
R2_ExtensorDigitorumLateralis2WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumLateralis2WI, a2_ExtensorDigitorumLateralis2WI,...
                                  phi1_ExtensorDigitorumLateralis2WI, phi2_ExtensorDigitorumLateralis2WI,...
                                  R1_ExtensorDigitorumLateralis2WI, R2_ExtensorDigitorumLateralis2WI,...
                                  f1_ExtensorDigitorumLateralis2WI, f2_ExtensorDigitorumLateralis2WI, ...
                                  n1_ExtensorDigitorumLateralis2WI, n2_ExtensorDigitorumLateralis2WI];
                              
MuscleGeometricParameters(k,1:10) = nan; % This muscle does not contribute in the 2D flexion-extension movement      

%% --------------------------- Extensor digitorum lateralis 3 O-V(37->38) ------------------------------------
k = 24;
JCN = 3;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumlateralis3OW = -1;
f2_ExtensorDigitorumlateralis3OW = nan;
n1_ExtensorDigitorumlateralis3OW = -1;
n2_ExtensorDigitorumlateralis3OW = nan;
% **** determine R1 & R2 


a1_ExtensorDigitorumlateralis3OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumlateralis3OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumlateralis3OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumlateralis3OW = Da2*real(acos(CosTheta));


R1_ExtensorDigitorumlateralis3OW = a1_ExtensorDigitorumlateralis3OW;
R2_ExtensorDigitorumlateralis3OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumlateralis3OW, a2_ExtensorDigitorumlateralis3OW,...
                                  phi1_ExtensorDigitorumlateralis3OW, phi2_ExtensorDigitorumlateralis3OW,...
                                  R1_ExtensorDigitorumlateralis3OW, R2_ExtensorDigitorumlateralis3OW,...
                                  f1_ExtensorDigitorumlateralis3OW, f2_ExtensorDigitorumlateralis3OW, ...
                                  n1_ExtensorDigitorumlateralis3OW, n2_ExtensorDigitorumlateralis3OW];
                              
MuscleGeometricParameters(k,1:10) = nan; % This muscle does not contribute in the 2D flexion-extension movement      
                              
                              
%--------------------------- Extensor digitorum lateralis 3 V-I(39->40) ------------------------------------
k = 25;
JCN = 4;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumlateralis3WI = -1;
f2_ExtensorDigitorumlateralis3WI = nan;
n1_ExtensorDigitorumlateralis3WI = -1;
n2_ExtensorDigitorumlateralis3WI = nan;
% **** determine R1 & R2 

a1_ExtensorDigitorumlateralis3WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumlateralis3WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumlateralis3WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumlateralis3WI = Da2*real(acos(CosTheta));

R1_ExtensorDigitorumlateralis3WI = a1_ExtensorDigitorumlateralis3WI;
R2_ExtensorDigitorumlateralis3WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumlateralis3WI, a2_ExtensorDigitorumlateralis3WI,...
                                  phi1_ExtensorDigitorumlateralis3WI, phi2_ExtensorDigitorumlateralis3WI,...
                                  R1_ExtensorDigitorumlateralis3WI, R2_ExtensorDigitorumlateralis3WI,...
                                  f1_ExtensorDigitorumlateralis3WI, f2_ExtensorDigitorumlateralis3WI, ...
                                  n1_ExtensorDigitorumlateralis3WI, n2_ExtensorDigitorumlateralis3WI];
                              
MuscleGeometricParameters(k,1:10) = nan; % This muscle does not contribute in the 2D flexion-extension movement      
                              
%% --------------------------- Extensor digitorum lateralis 4 O-V(41->42) ------------------------------------
k = 26;
JCN = 3;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumlateralis4OW = -1;
f2_ExtensorDigitorumlateralis4OW = nan;
n1_ExtensorDigitorumlateralis4OW = -1;
n2_ExtensorDigitorumlateralis4OW = nan;
% **** determine R1 & R2 


a1_ExtensorDigitorumlateralis4OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumlateralis4OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumlateralis4OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumlateralis4OW = Da2*real(acos(CosTheta));


R1_ExtensorDigitorumlateralis4OW = a1_ExtensorDigitorumlateralis4OW;
R2_ExtensorDigitorumlateralis4OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumlateralis4OW, a2_ExtensorDigitorumlateralis4OW,...
                                  phi1_ExtensorDigitorumlateralis4OW, phi2_ExtensorDigitorumlateralis4OW,...
                                  R1_ExtensorDigitorumlateralis4OW, R2_ExtensorDigitorumlateralis4OW,...
                                  f1_ExtensorDigitorumlateralis4OW, f2_ExtensorDigitorumlateralis4OW, ...
                                  n1_ExtensorDigitorumlateralis4OW, n2_ExtensorDigitorumlateralis4OW];
                              
                              
MuscleGeometricParameters(k,1:10) = nan; % This muscle does not contribute in the 2D flexion-extension movement      
                              
%--------------------------- Extensor digitorum lateralis 4 V-I(42->43) ------------------------------------
k = 27;
JCN = 4;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumlateralis4WI = -1;
f2_ExtensorDigitorumlateralis4WI = nan;
n1_ExtensorDigitorumlateralis4WI = -1;
n2_ExtensorDigitorumlateralis4WI = nan;
% **** determine R1 & R2 

a1_ExtensorDigitorumlateralis4WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumlateralis4WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumlateralis4WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumlateralis4WI = Da2*real(acos(CosTheta));

R1_ExtensorDigitorumlateralis4WI = a1_ExtensorDigitorumlateralis4WI;
R2_ExtensorDigitorumlateralis4WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumlateralis4WI, a2_ExtensorDigitorumlateralis4WI,...
                                  phi1_ExtensorDigitorumlateralis4WI, phi2_ExtensorDigitorumlateralis4WI,...
                                  R1_ExtensorDigitorumlateralis4WI, R2_ExtensorDigitorumlateralis4WI,...
                                  f1_ExtensorDigitorumlateralis4WI, f2_ExtensorDigitorumlateralis4WI, ...
                                  n1_ExtensorDigitorumlateralis4WI, n2_ExtensorDigitorumlateralis4WI];
                              
MuscleGeometricParameters(k,1:10) = nan; % This muscle does not contribute in the 2D flexion-extension movement      
                              

%% --------------------------- Extensor digitorum lateralis 5 O-V(44->45) ------------------------------------
k = 28;
JCN = 3;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumlateralis5OW = -1;
f2_ExtensorDigitorumlateralis5OW = nan;
n1_ExtensorDigitorumlateralis5OW = -1;
n2_ExtensorDigitorumlateralis5OW = nan;
% **** determine R1 & R2 


a1_ExtensorDigitorumlateralis5OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumlateralis5OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumlateralis5OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumlateralis5OW = Da2*real(acos(CosTheta));


R1_ExtensorDigitorumlateralis5OW = a1_ExtensorDigitorumlateralis5OW;
R2_ExtensorDigitorumlateralis5OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumlateralis5OW, a2_ExtensorDigitorumlateralis5OW,...
                                  phi1_ExtensorDigitorumlateralis5OW, phi2_ExtensorDigitorumlateralis5OW,...
                                  R1_ExtensorDigitorumlateralis5OW, R2_ExtensorDigitorumlateralis5OW,...
                                  f1_ExtensorDigitorumlateralis5OW, f2_ExtensorDigitorumlateralis5OW, ...
                                  n1_ExtensorDigitorumlateralis5OW, n2_ExtensorDigitorumlateralis5OW];
                              
MuscleGeometricParameters(k,1:10) = nan; % This muscle does not contribute in the 2D flexion-extension movement                                    
                              
%--------------------------- Extensor digitorum lateralis 5 V-I(45->46) ------------------------------------
k = 29;
JCN = 4;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorDigitorumlateralis5WI = -1;
f2_ExtensorDigitorumlateralis5WI = nan;
n1_ExtensorDigitorumlateralis5WI = -1;
n2_ExtensorDigitorumlateralis5WI = nan;
% **** determine R1 & R2 

a1_ExtensorDigitorumlateralis5WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorDigitorumlateralis5WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorDigitorumlateralis5WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorDigitorumlateralis5WI = Da2*real(acos(CosTheta));

R1_ExtensorDigitorumlateralis5WI = a1_ExtensorDigitorumlateralis5WI;
R2_ExtensorDigitorumlateralis5WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorDigitorumlateralis5WI, a2_ExtensorDigitorumlateralis5WI,...
                                  phi1_ExtensorDigitorumlateralis5WI, phi2_ExtensorDigitorumlateralis5WI,...
                                  R1_ExtensorDigitorumlateralis5WI, R2_ExtensorDigitorumlateralis5WI,...
                                  f1_ExtensorDigitorumlateralis5WI, f2_ExtensorDigitorumlateralis5WI, ...
                                  n1_ExtensorDigitorumlateralis5WI, n2_ExtensorDigitorumlateralis5WI];
                              
MuscleGeometricParameters(k,1:10) = nan; % This muscle does not contribute in the 2D flexion-extension movement      
                              
                              
%% --------------------------- Extensor Pollicis Longus 1 O-V(47->48) ------------------------------------
k = 30;
JCN = 3;   % Joint center number
Da1 = -1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorPollicisLongus1OW = nan;
f2_ExtensorPollicisLongus1OW = nan;
n1_ExtensorPollicisLongus1OW = nan;
n2_ExtensorPollicisLongus1OW = nan;
% **** determine R1 & R2 


a1_ExtensorPollicisLongus1OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorPollicisLongus1OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorPollicisLongus1OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorPollicisLongus1OW = Da2*real(acos(CosTheta));


R1_ExtensorPollicisLongus1OW = nan;
R2_ExtensorPollicisLongus1OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorPollicisLongus1OW, a2_ExtensorPollicisLongus1OW,...
                                  phi1_ExtensorPollicisLongus1OW, phi2_ExtensorPollicisLongus1OW,...
                                  R1_ExtensorPollicisLongus1OW, R2_ExtensorPollicisLongus1OW,...
                                  f1_ExtensorPollicisLongus1OW, f2_ExtensorPollicisLongus1OW, ...
                                  n1_ExtensorPollicisLongus1OW, n2_ExtensorPollicisLongus1OW];
                              
                         
                              
%--------------------------- Extensor Pollicis Longus 1 V-I(48->49)-----------------------------------------
k = 31;
JCN = 4;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorPollicisLongus1WI = -1;
f2_ExtensorPollicisLongus1WI = nan;
n1_ExtensorPollicisLongus1WI = -1;
n2_ExtensorPollicisLongus1WI = nan;
% **** determine R1 & R2 

a1_ExtensorPollicisLongus1WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorPollicisLongus1WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorPollicisLongus1WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorPollicisLongus1WI = Da2*real(acos(CosTheta));

R1_ExtensorPollicisLongus1WI = 0.003; % a1_ExtensorPollicisLongus1WI;
R2_ExtensorPollicisLongus1WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorPollicisLongus1WI, a2_ExtensorPollicisLongus1WI,...
                                  phi1_ExtensorPollicisLongus1WI, phi2_ExtensorPollicisLongus1WI,...
                                  R1_ExtensorPollicisLongus1WI, R2_ExtensorPollicisLongus1WI,...
                                  f1_ExtensorPollicisLongus1WI, f2_ExtensorPollicisLongus1WI, ...
                                  n1_ExtensorPollicisLongus1WI, n2_ExtensorPollicisLongus1WI];                              
                              
                              
                              
%% --------------------------- Extensor Pollicis Longus 2 O-V(50->51) ------------------------------------
k = 32;
JCN = 3;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorPollicisLongus2OW = nan;
f2_ExtensorPollicisLongus2OW = nan;
n1_ExtensorPollicisLongus2OW = nan;
n2_ExtensorPollicisLongus2OW = nan;
% **** determine R1 & R2 


a1_ExtensorPollicisLongus2OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorPollicisLongus2OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorPollicisLongus2OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorPollicisLongus2OW = Da2*real(acos(CosTheta));


R1_ExtensorPollicisLongus2OW = nan;
R2_ExtensorPollicisLongus2OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorPollicisLongus2OW, a2_ExtensorPollicisLongus2OW,...
                                  phi1_ExtensorPollicisLongus2OW, phi2_ExtensorPollicisLongus2OW,...
                                  R1_ExtensorPollicisLongus2OW, R2_ExtensorPollicisLongus2OW,...
                                  f1_ExtensorPollicisLongus2OW, f2_ExtensorPollicisLongus2OW, ...
                                  n1_ExtensorPollicisLongus2OW, n2_ExtensorPollicisLongus2OW];
                              
                           
                              
%--------------------------- Extensor Pollicis Longus 2 V-I(51->52)-----------------------------------------
k = 33;
JCN = 4;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_ExtensorPollicisLongus2WI = -1;
f2_ExtensorPollicisLongus2WI = nan;
n1_ExtensorPollicisLongus2WI = -1;
n2_ExtensorPollicisLongus2WI = nan;
% **** determine R1 & R2 

a1_ExtensorPollicisLongus2WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_ExtensorPollicisLongus2WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_ExtensorPollicisLongus2WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_ExtensorPollicisLongus2WI = Da2*real(acos(CosTheta));

R1_ExtensorPollicisLongus2WI = 0.003; %a1_ExtensorPollicisLongus2WI;
R2_ExtensorPollicisLongus2WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_ExtensorPollicisLongus2WI, a2_ExtensorPollicisLongus2WI,...
                                  phi1_ExtensorPollicisLongus2WI, phi2_ExtensorPollicisLongus2WI,...
                                  R1_ExtensorPollicisLongus2WI, R2_ExtensorPollicisLongus2WI,...
                                  f1_ExtensorPollicisLongus2WI, f2_ExtensorPollicisLongus2WI, ...
                                  n1_ExtensorPollicisLongus2WI, n2_ExtensorPollicisLongus2WI];
                              
                              

                              
%% --------------------------- Epitrochlearis(53->54) ------------------------------------
k = 34;
JCN = 3; % Joint center number
Da1 = 1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_Epitrochlearis = 1;
f2_Epitrochlearis = nan;
n1_Epitrochlearis = -1;
n2_Epitrochlearis = nan;
% **** determine R1 & R2 

a1_Epitrochlearis = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_Epitrochlearis = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_Epitrochlearis = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_Epitrochlearis = Da2*real(acos(CosTheta));


R1_Epitrochlearis = 0.002; % a2_Epitrochlearis;
R2_Epitrochlearis = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_Epitrochlearis, a2_Epitrochlearis,...
                                  phi1_Epitrochlearis, phi2_Epitrochlearis,...
                                  R1_Epitrochlearis, R2_Epitrochlearis,...
                                  f1_Epitrochlearis, f2_Epitrochlearis, ...
                                  n1_Epitrochlearis, n2_Epitrochlearis];
                              
                              
                              
                              
                              


%% --------------------------- Flexor Carpi Radialis O-V(55->56) ------------------------------------
k = 35;
JCN = 3;   % Joint center number
Da1 =  1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorCarpiRadialisOW = nan;
f2_FlexorCarpiRadialisOW = nan;
n1_FlexorCarpiRadialisOW = nan;
n2_FlexorCarpiRadialisOW = nan;
% **** determine R1 & R2 


a1_FlexorCarpiRadialisOW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorCarpiRadialisOW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorCarpiRadialisOW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorCarpiRadialisOW = Da2*real(acos(CosTheta));


R1_FlexorCarpiRadialisOW = nan;
R2_FlexorCarpiRadialisOW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorCarpiRadialisOW, a2_FlexorCarpiRadialisOW,...
                                  phi1_FlexorCarpiRadialisOW, phi2_FlexorCarpiRadialisOW,...
                                  R1_FlexorCarpiRadialisOW, R2_FlexorCarpiRadialisOW,...
                                  f1_FlexorCarpiRadialisOW, f2_FlexorCarpiRadialisOW, ...
                                  n1_FlexorCarpiRadialisOW, n2_FlexorCarpiRadialisOW];
                              
                         
                              
%--------------------------- Flexor Carpi Radialis V-I(56->57)-----------------------------------------
k = 36;
JCN =  4;   % Joint center number
Da1 =  1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorCarpiRadialisWI = 1;
f2_FlexorCarpiRadialisWI = nan;
n1_FlexorCarpiRadialisWI = -1;
n2_FlexorCarpiRadialisWI = nan;
% **** determine R1 & R2 

a1_FlexorCarpiRadialisWI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorCarpiRadialisWI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorCarpiRadialisWI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorCarpiRadialisWI = Da2*real(acos(CosTheta));

R1_FlexorCarpiRadialisWI = 0.003; %a1_FlexorCarpiRadialisWI;
R2_FlexorCarpiRadialisWI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorCarpiRadialisWI, a2_FlexorCarpiRadialisWI,...
                                  phi1_FlexorCarpiRadialisWI, phi2_FlexorCarpiRadialisWI,...
                                  R1_FlexorCarpiRadialisWI, R2_FlexorCarpiRadialisWI,...
                                  f1_FlexorCarpiRadialisWI, f2_FlexorCarpiRadialisWI, ...
                                  n1_FlexorCarpiRadialisWI, n2_FlexorCarpiRadialisWI];                              
                              
                              
                              



%% --------------------------- Flexor Carpi Ulnaris O-V(58->59) ------------------------------------
k = 37;
JCN = 3;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorCarpiUlnarisOW = nan;
f2_FlexorCarpiUlnarisOW = nan;
n1_FlexorCarpiUlnarisOW = nan;
n2_FlexorCarpiUlnarisOW = nan;
% **** determine R1 & R2 


a1_FlexorCarpiUlnarisOW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorCarpiUlnarisOW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorCarpiUlnarisOW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorCarpiUlnarisOW = Da2*real(acos(CosTheta));


R1_FlexorCarpiUlnarisOW = nan;
R2_FlexorCarpiUlnarisOW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorCarpiUlnarisOW, a2_FlexorCarpiUlnarisOW,...
                                  phi1_FlexorCarpiUlnarisOW, phi2_FlexorCarpiUlnarisOW,...
                                  R1_FlexorCarpiUlnarisOW, R2_FlexorCarpiUlnarisOW,...
                                  f1_FlexorCarpiUlnarisOW, f2_FlexorCarpiUlnarisOW, ...
                                  n1_FlexorCarpiUlnarisOW, n2_FlexorCarpiUlnarisOW];
                              
                              
                              
%--------------------------- Flexor Carpi Ulnaris V-I(59->60)-----------------------------------------
k = 38;
JCN = 4;   % Joint center number
Da1 = 1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorCarpiUlnarisWI = 1;
f2_FlexorCarpiUlnarisWI = nan;
n1_FlexorCarpiUlnarisWI = -1;
n2_FlexorCarpiUlnarisWI = nan;
% **** determine R1 & R2 

a1_FlexorCarpiUlnarisWI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorCarpiUlnarisWI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorCarpiUlnarisWI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorCarpiUlnarisWI = Da2*real(acos(CosTheta));

R1_FlexorCarpiUlnarisWI = a1_FlexorCarpiUlnarisWI;
R2_FlexorCarpiUlnarisWI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorCarpiUlnarisWI, a2_FlexorCarpiUlnarisWI,...
                                  phi1_FlexorCarpiUlnarisWI, phi2_FlexorCarpiUlnarisWI,...
                                  R1_FlexorCarpiUlnarisWI, R2_FlexorCarpiUlnarisWI,...
                                  f1_FlexorCarpiUlnarisWI, f2_FlexorCarpiUlnarisWI, ...
                                  n1_FlexorCarpiUlnarisWI, n2_FlexorCarpiUlnarisWI];
                              
                              
                              


%% --------------------------- Flexor Digitorum Profundus 1 O-V(61->62) ------------------------------------
k = 39;
JCN = 3;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumProfundus1OW = nan;
f2_FlexorDigitorumProfundus1OW = nan;
n1_FlexorDigitorumProfundus1OW = nan;
n2_FlexorDigitorumProfundus1OW = nan;
% **** determine R1 & R2 


a1_FlexorDigitorumProfundus1OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumProfundus1OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumProfundus1OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumProfundus1OW = Da2*real(acos(CosTheta));


R1_FlexorDigitorumProfundus1OW = nan;
R2_FlexorDigitorumProfundus1OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumProfundus1OW, a2_FlexorDigitorumProfundus1OW,...
                                  phi1_FlexorDigitorumProfundus1OW, phi2_FlexorDigitorumProfundus1OW,...
                                  R1_FlexorDigitorumProfundus1OW, R2_FlexorDigitorumProfundus1OW,...
                                  f1_FlexorDigitorumProfundus1OW, f2_FlexorDigitorumProfundus1OW, ...
                                  n1_FlexorDigitorumProfundus1OW, n2_FlexorDigitorumProfundus1OW];
                              
                              
                              
%--------------------------- Flexor Digitorum Profundus 1 V-I(62->63)-----------------------------------------
k = 40;
JCN = 4;   % Joint center number
Da1 = 1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumProfundus1WI = 1;
f2_FlexorDigitorumProfundus1WI = nan;
n1_FlexorDigitorumProfundus1WI = -1;
n2_FlexorDigitorumProfundus1WI = nan;
% **** determine R1 & R2 

a1_FlexorDigitorumProfundus1WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumProfundus1WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumProfundus1WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumProfundus1WI = Da2*real(acos(CosTheta));

R1_FlexorDigitorumProfundus1WI = a1_FlexorDigitorumProfundus1WI;
R2_FlexorDigitorumProfundus1WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumProfundus1WI, a2_FlexorDigitorumProfundus1WI,...
                                  phi1_FlexorDigitorumProfundus1WI, phi2_FlexorDigitorumProfundus1WI,...
                                  R1_FlexorDigitorumProfundus1WI, R2_FlexorDigitorumProfundus1WI,...
                                  f1_FlexorDigitorumProfundus1WI, f2_FlexorDigitorumProfundus1WI, ...
                                  n1_FlexorDigitorumProfundus1WI, n2_FlexorDigitorumProfundus1WI];
                              
                              
                              
                              
%% --------------------------- Flexor Digitorum Profundus 2 O-V(61->62) ------------------------------------
k = 41;
JCN = 3;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumProfundus2OW = nan;
f2_FlexorDigitorumProfundus2OW = nan;
n1_FlexorDigitorumProfundus2OW = nan;
n2_FlexorDigitorumProfundus2OW = nan;
% **** determine R1 & R2 


a1_FlexorDigitorumProfundus2OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumProfundus2OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumProfundus2OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumProfundus2OW = Da2*real(acos(CosTheta));


R1_FlexorDigitorumProfundus2OW = nan;
R2_FlexorDigitorumProfundus2OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumProfundus2OW, a2_FlexorDigitorumProfundus2OW,...
                                  phi1_FlexorDigitorumProfundus2OW, phi2_FlexorDigitorumProfundus2OW,...
                                  R1_FlexorDigitorumProfundus2OW, R2_FlexorDigitorumProfundus2OW,...
                                  f1_FlexorDigitorumProfundus2OW, f2_FlexorDigitorumProfundus2OW, ...
                                  n1_FlexorDigitorumProfundus2OW, n2_FlexorDigitorumProfundus2OW];
                              
                              
                              
%--------------------------- Flexor Digitorum Profundus 2 V-I(62->63)-----------------------------------------
k = 42;
JCN = 4;   % Joint center number
Da1 = 1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumProfundus2WI = 1;
f2_FlexorDigitorumProfundus2WI = nan;
n1_FlexorDigitorumProfundus2WI = -1;
n2_FlexorDigitorumProfundus2WI = nan;
% **** determine R1 & R2 

a1_FlexorDigitorumProfundus2WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumProfundus2WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumProfundus2WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumProfundus2WI = Da2*real(acos(CosTheta));

R1_FlexorDigitorumProfundus2WI = a1_FlexorDigitorumProfundus2WI;
R2_FlexorDigitorumProfundus2WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumProfundus2WI, a2_FlexorDigitorumProfundus2WI,...
                                  phi1_FlexorDigitorumProfundus2WI, phi2_FlexorDigitorumProfundus2WI,...
                                  R1_FlexorDigitorumProfundus2WI, R2_FlexorDigitorumProfundus2WI,...
                                  f1_FlexorDigitorumProfundus2WI, f2_FlexorDigitorumProfundus2WI, ...
                                  n1_FlexorDigitorumProfundus2WI, n2_FlexorDigitorumProfundus2WI];
                              
                              
                              

                              
%% --------------------------- Flexor Digitorum Profundus 3 O-V(67->68) ------------------------------------
k = 43;
JCN = 3;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumProfundus3OW = nan;
f2_FlexorDigitorumProfundus3OW = nan;
n1_FlexorDigitorumProfundus3OW = nan;
n2_FlexorDigitorumProfundus3OW = nan;
% **** determine R1 & R2 


a1_FlexorDigitorumProfundus3OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumProfundus3OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumProfundus3OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumProfundus3OW = Da2*real(acos(CosTheta));


R1_FlexorDigitorumProfundus3OW = nan;
R2_FlexorDigitorumProfundus3OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumProfundus3OW, a2_FlexorDigitorumProfundus3OW,...
                                  phi1_FlexorDigitorumProfundus3OW, phi2_FlexorDigitorumProfundus3OW,...
                                  R1_FlexorDigitorumProfundus3OW, R2_FlexorDigitorumProfundus3OW,...
                                  f1_FlexorDigitorumProfundus3OW, f2_FlexorDigitorumProfundus3OW, ...
                                  n1_FlexorDigitorumProfundus3OW, n2_FlexorDigitorumProfundus3OW];
                              
                              
                              
%--------------------------- Flexor Digitorum Profundus 3 V-I(68->69)-----------------------------------------
k = 44;
JCN = 4;   % Joint center number
Da1 = 1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumProfundus3WI = 1;
f2_FlexorDigitorumProfundus3WI = nan;
n1_FlexorDigitorumProfundus3WI = -1;
n2_FlexorDigitorumProfundus3WI = nan;
% **** determine R1 & R2 

a1_FlexorDigitorumProfundus3WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumProfundus3WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumProfundus3WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumProfundus3WI = Da2*real(acos(CosTheta));

R1_FlexorDigitorumProfundus3WI = a1_FlexorDigitorumProfundus3WI;
R2_FlexorDigitorumProfundus3WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumProfundus3WI, a2_FlexorDigitorumProfundus3WI,...
                                  phi1_FlexorDigitorumProfundus3WI, phi2_FlexorDigitorumProfundus3WI,...
                                  R1_FlexorDigitorumProfundus3WI, R2_FlexorDigitorumProfundus3WI,...
                                  f1_FlexorDigitorumProfundus3WI, f2_FlexorDigitorumProfundus3WI, ...
                                  n1_FlexorDigitorumProfundus3WI, n2_FlexorDigitorumProfundus3WI];
                              
                              

%% --------------------------- Flexor Digitorum Profundus 4 O-V(70->71) ------------------------------------
k = 45;
JCN = 3;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumProfundus4OW = nan;
f2_FlexorDigitorumProfundus4OW = nan;
n1_FlexorDigitorumProfundus4OW = nan;
n2_FlexorDigitorumProfundus4OW = nan;
% **** determine R1 & R2 


a1_FlexorDigitorumProfundus4OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumProfundus4OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumProfundus4OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumProfundus4OW = Da2*real(acos(CosTheta));


R1_FlexorDigitorumProfundus4OW = nan;
R2_FlexorDigitorumProfundus4OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumProfundus4OW, a2_FlexorDigitorumProfundus4OW,...
                                  phi1_FlexorDigitorumProfundus4OW, phi2_FlexorDigitorumProfundus4OW,...
                                  R1_FlexorDigitorumProfundus4OW, R2_FlexorDigitorumProfundus4OW,...
                                  f1_FlexorDigitorumProfundus4OW, f2_FlexorDigitorumProfundus4OW, ...
                                  n1_FlexorDigitorumProfundus4OW, n2_FlexorDigitorumProfundus4OW];
                              
                              
                              
%--------------------------- Flexor Digitorum Profundus 4 V-I(71->72)-----------------------------------------
k = 46;
JCN = 4;   % Joint center number
Da1 = 1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumProfundus4WI = 1;
f2_FlexorDigitorumProfundus4WI = nan;
n1_FlexorDigitorumProfundus4WI = -1;
n2_FlexorDigitorumProfundus4WI = nan;
% **** determine R1 & R2 

a1_FlexorDigitorumProfundus4WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumProfundus4WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumProfundus4WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumProfundus4WI = Da2*real(acos(CosTheta));

R1_FlexorDigitorumProfundus4WI = a1_FlexorDigitorumProfundus4WI;
R2_FlexorDigitorumProfundus4WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumProfundus4WI, a2_FlexorDigitorumProfundus4WI,...
                                  phi1_FlexorDigitorumProfundus4WI, phi2_FlexorDigitorumProfundus4WI,...
                                  R1_FlexorDigitorumProfundus4WI, R2_FlexorDigitorumProfundus4WI,...
                                  f1_FlexorDigitorumProfundus4WI, f2_FlexorDigitorumProfundus4WI, ...
                                  n1_FlexorDigitorumProfundus4WI, n2_FlexorDigitorumProfundus4WI];
                              
                              

%% --------------------------- Flexor Digitorum Profundus 5 O-V(73->74) ------------------------------------
k = 47;
JCN = 3;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumProfundus5OW = nan;
f2_FlexorDigitorumProfundus5OW = nan;
n1_FlexorDigitorumProfundus5OW = nan;
n2_FlexorDigitorumProfundus5OW = nan;
% **** determine R1 & R2 


a1_FlexorDigitorumProfundus5OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumProfundus5OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumProfundus5OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumProfundus5OW = Da2*real(acos(CosTheta));


R1_FlexorDigitorumProfundus5OW = nan;
R2_FlexorDigitorumProfundus5OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumProfundus5OW, a2_FlexorDigitorumProfundus5OW,...
                                  phi1_FlexorDigitorumProfundus5OW, phi2_FlexorDigitorumProfundus5OW,...
                                  R1_FlexorDigitorumProfundus5OW, R2_FlexorDigitorumProfundus5OW,...
                                  f1_FlexorDigitorumProfundus5OW, f2_FlexorDigitorumProfundus5OW, ...
                                  n1_FlexorDigitorumProfundus5OW, n2_FlexorDigitorumProfundus5OW];
                              
                              
                              
%--------------------------- Flexor Digitorum Profundus 5 V-I(74->75)-----------------------------------------
k = 48;
JCN = 4;   % Joint center number
Da1 = 1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumProfundus5WI = 1;
f2_FlexorDigitorumProfundus5WI = nan;
n1_FlexorDigitorumProfundus5WI = -1;
n2_FlexorDigitorumProfundus5WI = nan;
% **** determine R1 & R2 

a1_FlexorDigitorumProfundus5WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumProfundus5WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumProfundus5WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumProfundus5WI = Da2*real(acos(CosTheta));

R1_FlexorDigitorumProfundus5WI = a1_FlexorDigitorumProfundus5WI;
R2_FlexorDigitorumProfundus5WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumProfundus5WI, a2_FlexorDigitorumProfundus5WI,...
                                  phi1_FlexorDigitorumProfundus5WI, phi2_FlexorDigitorumProfundus5WI,...
                                  R1_FlexorDigitorumProfundus5WI, R2_FlexorDigitorumProfundus5WI,...
                                  f1_FlexorDigitorumProfundus5WI, f2_FlexorDigitorumProfundus5WI, ...
                                  n1_FlexorDigitorumProfundus5WI, n2_FlexorDigitorumProfundus5WI];
                              
                              
                              

%% --------------------------- Flexor Digitorum Superficialis 2 O-V(76->77) ------------------------------------
k = 49;
JCN = 4;   % Joint center number
Da1 =  1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumSuperficialis2OW = 1;
f2_FlexorDigitorumSuperficialis2OW = nan;
n1_FlexorDigitorumSuperficialis2OW = -1;
n2_FlexorDigitorumSuperficialis2OW = nan;
% **** determine R1 & R2 


a1_FlexorDigitorumSuperficialis2OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumSuperficialis2OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumSuperficialis2OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumSuperficialis2OW = Da2*real(acos(CosTheta));


R1_FlexorDigitorumSuperficialis2OW = 0.003;
R2_FlexorDigitorumSuperficialis2OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumSuperficialis2OW, a2_FlexorDigitorumSuperficialis2OW,...
                                  phi1_FlexorDigitorumSuperficialis2OW, phi2_FlexorDigitorumSuperficialis2OW,...
                                  R1_FlexorDigitorumSuperficialis2OW, R2_FlexorDigitorumSuperficialis2OW,...
                                  f1_FlexorDigitorumSuperficialis2OW, f2_FlexorDigitorumSuperficialis2OW, ...
                                  n1_FlexorDigitorumSuperficialis2OW, n2_FlexorDigitorumSuperficialis2OW];
                              
                              
                              
%--------------------------- Flexor Digitorum Superficialis 2 V-I(77->78) ------------------------------------
k = 50;
JCN = 4;   % Joint center number
Da1 = -1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumSuperficialis2WI = nan;
f2_FlexorDigitorumSuperficialis2WI = nan;
n1_FlexorDigitorumSuperficialis2WI = nan;
n2_FlexorDigitorumSuperficialis2WI = nan;
% **** determine R1 & R2 

a1_FlexorDigitorumSuperficialis2WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumSuperficialis2WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN+1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumSuperficialis2WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumSuperficialis2WI = Da2*real(acos(CosTheta));

R1_FlexorDigitorumSuperficialis2WI = nan;
R2_FlexorDigitorumSuperficialis2WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumSuperficialis2WI, a2_FlexorDigitorumSuperficialis2WI,...
                                  phi1_FlexorDigitorumSuperficialis2WI, phi2_FlexorDigitorumSuperficialis2WI,...
                                  R1_FlexorDigitorumSuperficialis2WI, R2_FlexorDigitorumSuperficialis2WI,...
                                  f1_FlexorDigitorumSuperficialis2WI, f2_FlexorDigitorumSuperficialis2WI, ...
                                  n1_FlexorDigitorumSuperficialis2WI, n2_FlexorDigitorumSuperficialis2WI];
                                                            
                              
                              
                              
                              
%% --------------------------- Flexor Digitorum Superficialis 3 O-V(79->80) ------------------------------------
k = 51;
JCN = 4;   % Joint center number
Da1 =  1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumSuperficialis3OW = 1;
f2_FlexorDigitorumSuperficialis3OW = nan;
n1_FlexorDigitorumSuperficialis3OW = -1;
n2_FlexorDigitorumSuperficialis3OW = nan;
% **** determine R1 & R2 


a1_FlexorDigitorumSuperficialis3OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumSuperficialis3OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumSuperficialis3OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumSuperficialis3OW = Da2*real(acos(CosTheta));


R1_FlexorDigitorumSuperficialis3OW = 0.003;
R2_FlexorDigitorumSuperficialis3OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumSuperficialis3OW, a2_FlexorDigitorumSuperficialis3OW,...
                                  phi1_FlexorDigitorumSuperficialis3OW, phi2_FlexorDigitorumSuperficialis3OW,...
                                  R1_FlexorDigitorumSuperficialis3OW, R2_FlexorDigitorumSuperficialis3OW,...
                                  f1_FlexorDigitorumSuperficialis3OW, f2_FlexorDigitorumSuperficialis3OW, ...
                                  n1_FlexorDigitorumSuperficialis3OW, n2_FlexorDigitorumSuperficialis3OW];
                              
                              
                              
%--------------------------- Flexor Digitorum Superficialis 3 V-I(80->81) ------------------------------------
k = 52;
JCN =  4;   % Joint center number
Da1 = -1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumSuperficialis3WI = nan;
f2_FlexorDigitorumSuperficialis3WI = nan;
n1_FlexorDigitorumSuperficialis3WI = nan;
n2_FlexorDigitorumSuperficialis3WI = nan;
% **** determine R1 & R2 

a1_FlexorDigitorumSuperficialis3WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumSuperficialis3WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN+1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumSuperficialis3WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumSuperficialis3WI = Da2*real(acos(CosTheta));

R1_FlexorDigitorumSuperficialis3WI = nan;
R2_FlexorDigitorumSuperficialis3WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumSuperficialis3WI, a2_FlexorDigitorumSuperficialis3WI,...
                                  phi1_FlexorDigitorumSuperficialis3WI, phi2_FlexorDigitorumSuperficialis3WI,...
                                  R1_FlexorDigitorumSuperficialis3WI, R2_FlexorDigitorumSuperficialis3WI,...
                                  f1_FlexorDigitorumSuperficialis3WI, f2_FlexorDigitorumSuperficialis3WI, ...
                                  n1_FlexorDigitorumSuperficialis3WI, n2_FlexorDigitorumSuperficialis3WI];
                              
                              
                              
%% --------------------------- Flexor Digitorum Superficialis 4 O-V(82->83) ------------------------------------
k = 53;
JCN =  4;   % Joint center number
Da1 =  1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumSuperficialis4OW = 1;
f2_FlexorDigitorumSuperficialis4OW = nan;
n1_FlexorDigitorumSuperficialis4OW = -1;
n2_FlexorDigitorumSuperficialis4OW = nan;
% **** determine R1 & R2 


a1_FlexorDigitorumSuperficialis4OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumSuperficialis4OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumSuperficialis4OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumSuperficialis4OW = Da2*real(acos(CosTheta));


R1_FlexorDigitorumSuperficialis4OW = 0.003;
R2_FlexorDigitorumSuperficialis4OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumSuperficialis4OW, a2_FlexorDigitorumSuperficialis4OW,...
                                  phi1_FlexorDigitorumSuperficialis4OW, phi2_FlexorDigitorumSuperficialis4OW,...
                                  R1_FlexorDigitorumSuperficialis4OW, R2_FlexorDigitorumSuperficialis4OW,...
                                  f1_FlexorDigitorumSuperficialis4OW, f2_FlexorDigitorumSuperficialis4OW, ...
                                  n1_FlexorDigitorumSuperficialis4OW, n2_FlexorDigitorumSuperficialis4OW];
                              
                              
                              
%--------------------------- Flexor Digitorum Superficialis 4 V-I(83->84) ------------------------------------
k = 54;
JCN =  4;   % Joint center number
Da1 = -1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumSuperficialis4WI = nan;
f2_FlexorDigitorumSuperficialis4WI = nan;
n1_FlexorDigitorumSuperficialis4WI = nan;
n2_FlexorDigitorumSuperficialis4WI = nan;
% **** determine R1 & R2 

a1_FlexorDigitorumSuperficialis4WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumSuperficialis4WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN+1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumSuperficialis4WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumSuperficialis4WI = Da2*real(acos(CosTheta));

R1_FlexorDigitorumSuperficialis4WI = nan;
R2_FlexorDigitorumSuperficialis4WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumSuperficialis4WI, a2_FlexorDigitorumSuperficialis4WI,...
                                  phi1_FlexorDigitorumSuperficialis4WI, phi2_FlexorDigitorumSuperficialis4WI,...
                                  R1_FlexorDigitorumSuperficialis4WI, R2_FlexorDigitorumSuperficialis4WI,...
                                  f1_FlexorDigitorumSuperficialis4WI, f2_FlexorDigitorumSuperficialis4WI, ...
                                  n1_FlexorDigitorumSuperficialis4WI, n2_FlexorDigitorumSuperficialis4WI];
                              
                              
                              


%% --------------------------- Flexor Digitorum Superficialis 5 O-V(85->86) ------------------------------------
k = 55;
JCN =  4;   % Joint center number
Da1 =  1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumSuperficialis5OW = 1;
f2_FlexorDigitorumSuperficialis5OW = nan;
n1_FlexorDigitorumSuperficialis5OW = -1;
n2_FlexorDigitorumSuperficialis5OW = nan;
% **** determine R1 & R2 


a1_FlexorDigitorumSuperficialis5OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumSuperficialis5OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumSuperficialis5OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumSuperficialis5OW = Da2*real(acos(CosTheta));


R1_FlexorDigitorumSuperficialis5OW = 0.003;
R2_FlexorDigitorumSuperficialis5OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumSuperficialis5OW, a2_FlexorDigitorumSuperficialis5OW,...
                                  phi1_FlexorDigitorumSuperficialis5OW, phi2_FlexorDigitorumSuperficialis5OW,...
                                  R1_FlexorDigitorumSuperficialis5OW, R2_FlexorDigitorumSuperficialis5OW,...
                                  f1_FlexorDigitorumSuperficialis5OW, f2_FlexorDigitorumSuperficialis5OW, ...
                                  n1_FlexorDigitorumSuperficialis5OW, n2_FlexorDigitorumSuperficialis5OW];
                              
                              
                              
%--------------------------- Flexor Digitorum Superficialis 5 V-I(86->87) ------------------------------------
k = 56;
JCN =  4;   % Joint center number
Da1 = -1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_FlexorDigitorumSuperficialis5WI = nan;
f2_FlexorDigitorumSuperficialis5WI = nan;
n1_FlexorDigitorumSuperficialis5WI = nan;
n2_FlexorDigitorumSuperficialis5WI = nan;
% **** determine R1 & R2 

a1_FlexorDigitorumSuperficialis5WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_FlexorDigitorumSuperficialis5WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN+1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_FlexorDigitorumSuperficialis5WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_FlexorDigitorumSuperficialis5WI = Da2*real(acos(CosTheta));

R1_FlexorDigitorumSuperficialis5WI = nan;
R2_FlexorDigitorumSuperficialis5WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_FlexorDigitorumSuperficialis5WI, a2_FlexorDigitorumSuperficialis5WI,...
                                  phi1_FlexorDigitorumSuperficialis5WI, phi2_FlexorDigitorumSuperficialis5WI,...
                                  R1_FlexorDigitorumSuperficialis5WI, R2_FlexorDigitorumSuperficialis5WI,...
                                  f1_FlexorDigitorumSuperficialis5WI, f2_FlexorDigitorumSuperficialis5WI, ...
                                  n1_FlexorDigitorumSuperficialis5WI, n2_FlexorDigitorumSuperficialis5WI];
                             
                              
                              
                              
                              
                              
                                                            
                              
%% --------------------------- Infraspinatus O-V(88->89) ------------------------------------
k = 57;
JCN = 1;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_InfraspinatusOW = nan;
f2_InfraspinatusOW = nan;
n1_InfraspinatusOW = nan;
n2_InfraspinatusOW = nan;
% **** determine R1 & R2 


a1_InfraspinatusOW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_InfraspinatusOW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_InfraspinatusOW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_InfraspinatusOW = Da2*real(acos(CosTheta));


R1_InfraspinatusOW = nan;
R2_InfraspinatusOW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_InfraspinatusOW, a2_InfraspinatusOW,...
                                  phi1_InfraspinatusOW, phi2_InfraspinatusOW,...
                                  R1_InfraspinatusOW, R2_InfraspinatusOW,...
                                  f1_InfraspinatusOW, f2_InfraspinatusOW, ...
                                  n1_InfraspinatusOW, n2_InfraspinatusOW];
                              
                              
                              
%--------------------------- Infraspinatus V-I(89->90)-----------------------------------------
k = 58;
JCN = 2;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_InfraspinatusWI = -1;
f2_InfraspinatusWI = nan;
n1_InfraspinatusWI = 1;
n2_InfraspinatusWI = nan;
% **** determine R1 & R2 

a1_InfraspinatusWI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_InfraspinatusWI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_InfraspinatusWI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_InfraspinatusWI = Da2*real(acos(CosTheta));

R1_InfraspinatusWI = 0.003; % a1_InfraspinatusWI;
R2_InfraspinatusWI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_InfraspinatusWI, a2_InfraspinatusWI,...
                                  phi1_InfraspinatusWI, phi2_InfraspinatusWI,...
                                  R1_InfraspinatusWI, R2_InfraspinatusWI,...
                                  f1_InfraspinatusWI, f2_InfraspinatusWI, ...
                                  n1_InfraspinatusWI, n2_InfraspinatusWI];
                              
                              
                              
%% --------------------------- Palmaris Longus 1 O-V(91->92) ------------------------------------
k = 59;
JCN = 3;   % Joint center number
Da1 =  1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_PalmarisLongus1OW = nan;
f2_PalmarisLongus1OW = nan;
n1_PalmarisLongus1OW = nan;
n2_PalmarisLongus1OW = nan;
% **** determine R1 & R2 


a1_PalmarisLongus1OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_PalmarisLongus1OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_PalmarisLongus1OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_PalmarisLongus1OW = Da2*real(acos(CosTheta));


R1_PalmarisLongus1OW = nan;
R2_PalmarisLongus1OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_PalmarisLongus1OW, a2_PalmarisLongus1OW,...
                                  phi1_PalmarisLongus1OW, phi2_PalmarisLongus1OW,...
                                  R1_PalmarisLongus1OW, R2_PalmarisLongus1OW,...
                                  f1_PalmarisLongus1OW, f2_PalmarisLongus1OW, ...
                                  n1_PalmarisLongus1OW, n2_PalmarisLongus1OW];
                              
                              
                              
%--------------------------- Palmaris Longus 1 V-I(92->93)-----------------------------------------
k = 60;
JCN = 4;   % Joint center number
Da1 =  1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_PalmarisLongus1WI = 1;
f2_PalmarisLongus1WI = nan;
n1_PalmarisLongus1WI = -1;
n2_PalmarisLongus1WI = nan;
% **** determine R1 & R2 

a1_PalmarisLongus1WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_PalmarisLongus1WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_PalmarisLongus1WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_PalmarisLongus1WI = Da2*real(acos(CosTheta));

R1_PalmarisLongus1WI = a1_PalmarisLongus1WI;
R2_PalmarisLongus1WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_PalmarisLongus1WI, a2_PalmarisLongus1WI,...
                                  phi1_PalmarisLongus1WI, phi2_PalmarisLongus1WI,...
                                  R1_PalmarisLongus1WI, R2_PalmarisLongus1WI,...
                                  f1_PalmarisLongus1WI, f2_PalmarisLongus1WI, ...
                                  n1_PalmarisLongus1WI, n2_PalmarisLongus1WI];
                              
                              
%% --------------------------- Palmaris Longus 2 O-V(94->95) ------------------------------------
k = 61;
JCN = 3;   % Joint center number
Da1 =  1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_PalmarisLongus2OW = nan;
f2_PalmarisLongus2OW = nan;
n1_PalmarisLongus2OW = nan;
n2_PalmarisLongus2OW = nan;
% **** determine R1 & R2 


a1_PalmarisLongus2OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_PalmarisLongus2OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_PalmarisLongus2OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_PalmarisLongus2OW = Da2*real(acos(CosTheta));


R1_PalmarisLongus2OW = nan;
R2_PalmarisLongus2OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_PalmarisLongus2OW, a2_PalmarisLongus2OW,...
                                  phi1_PalmarisLongus2OW, phi2_PalmarisLongus2OW,...
                                  R1_PalmarisLongus2OW, R2_PalmarisLongus2OW,...
                                  f1_PalmarisLongus2OW, f2_PalmarisLongus2OW, ...
                                  n1_PalmarisLongus2OW, n2_PalmarisLongus2OW];
                              
                              
                              
%--------------------------- Palmaris Longus 2 V-I(95->96)-----------------------------------------
k = 62;
JCN = 4;   % Joint center number
Da1 =  1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_PalmarisLongus2WI = 1;
f2_PalmarisLongus2WI = nan;
n1_PalmarisLongus2WI = -1;
n2_PalmarisLongus2WI = nan;
% **** determine R1 & R2 

a1_PalmarisLongus2WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_PalmarisLongus2WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_PalmarisLongus2WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_PalmarisLongus2WI = Da2*real(acos(CosTheta));

R1_PalmarisLongus2WI = a1_PalmarisLongus2WI;
R2_PalmarisLongus2WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_PalmarisLongus2WI, a2_PalmarisLongus2WI,...
                                  phi1_PalmarisLongus2WI, phi2_PalmarisLongus2WI,...
                                  R1_PalmarisLongus2WI, R2_PalmarisLongus2WI,...
                                  f1_PalmarisLongus2WI, f2_PalmarisLongus2WI, ...
                                  n1_PalmarisLongus2WI, n2_PalmarisLongus2WI];
                              
                              
                              
%% --------------------------- Palmaris Longus 3 O-V(97->98) ------------------------------------
k = 63;
JCN = 3;   % Joint center number
Da1 =  1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_PalmarisLongus3OW = nan;
f2_PalmarisLongus3OW = nan;
n1_PalmarisLongus3OW = nan;
n2_PalmarisLongus3OW = nan;
% **** determine R1 & R2 


a1_PalmarisLongus3OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_PalmarisLongus3OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_PalmarisLongus3OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_PalmarisLongus3OW = Da2*real(acos(CosTheta));


R1_PalmarisLongus3OW = nan;
R2_PalmarisLongus3OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_PalmarisLongus3OW, a2_PalmarisLongus3OW,...
                                  phi1_PalmarisLongus3OW, phi2_PalmarisLongus3OW,...
                                  R1_PalmarisLongus3OW, R2_PalmarisLongus3OW,...
                                  f1_PalmarisLongus3OW, f2_PalmarisLongus3OW, ...
                                  n1_PalmarisLongus3OW, n2_PalmarisLongus3OW];
                              
                              
                              
%--------------------------- Palmaris Longus 3 V-I(98->99)-----------------------------------------
k = 64;
JCN = 4;   % Joint center number
Da1 =  1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_PalmarisLongus3WI = 1;
f2_PalmarisLongus3WI = nan;
n1_PalmarisLongus3WI = -1;
n2_PalmarisLongus3WI = nan;
% **** determine R1 & R2 

a1_PalmarisLongus3WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_PalmarisLongus3WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_PalmarisLongus3WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_PalmarisLongus3WI = Da2*real(acos(CosTheta));

R1_PalmarisLongus3WI = a1_PalmarisLongus3WI;
R2_PalmarisLongus3WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_PalmarisLongus3WI, a2_PalmarisLongus3WI,...
                                  phi1_PalmarisLongus3WI, phi2_PalmarisLongus3WI,...
                                  R1_PalmarisLongus3WI, R2_PalmarisLongus3WI,...
                                  f1_PalmarisLongus3WI, f2_PalmarisLongus3WI, ...
                                  n1_PalmarisLongus3WI, n2_PalmarisLongus3WI];
                              
                              

%% --------------------------- Palmaris Longus 4 O-V(100->101) ------------------------------------
k = 65;
JCN = 3;   % Joint center number
Da1 =  1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_PalmarisLongus4OW = nan;
f2_PalmarisLongus4OW = nan;
n1_PalmarisLongus4OW = nan;
n2_PalmarisLongus4OW = nan;
% **** determine R1 & R2 


a1_PalmarisLongus4OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_PalmarisLongus4OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_PalmarisLongus4OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_PalmarisLongus4OW = Da2*real(acos(CosTheta));


R1_PalmarisLongus4OW = nan;
R2_PalmarisLongus4OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_PalmarisLongus4OW, a2_PalmarisLongus4OW,...
                                  phi1_PalmarisLongus4OW, phi2_PalmarisLongus4OW,...
                                  R1_PalmarisLongus4OW, R2_PalmarisLongus4OW,...
                                  f1_PalmarisLongus4OW, f2_PalmarisLongus4OW, ...
                                  n1_PalmarisLongus4OW, n2_PalmarisLongus4OW];
                              
                              
                              
%--------------------------- Palmaris Longus 4 V-I(101->102)-----------------------------------------
k = 66;
JCN = 4;   % Joint center number
Da1 =  1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_PalmarisLongus4WI = 1;
f2_PalmarisLongus4WI = nan;
n1_PalmarisLongus4WI = -1;
n2_PalmarisLongus4WI = nan;
% **** determine R1 & R2 

a1_PalmarisLongus4WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_PalmarisLongus4WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_PalmarisLongus4WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_PalmarisLongus4WI = Da2*real(acos(CosTheta));

R1_PalmarisLongus4WI = a1_PalmarisLongus4WI;
R2_PalmarisLongus4WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_PalmarisLongus4WI, a2_PalmarisLongus4WI,...
                                  phi1_PalmarisLongus4WI, phi2_PalmarisLongus4WI,...
                                  R1_PalmarisLongus4WI, R2_PalmarisLongus4WI,...
                                  f1_PalmarisLongus4WI, f2_PalmarisLongus4WI, ...
                                  n1_PalmarisLongus4WI, n2_PalmarisLongus4WI];
                              
                              
               
%% --------------------------- Palmaris Longus 5 O-V(103->104) ------------------------------------
k = 67;
JCN = 3;   % Joint center number
Da1 =  1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_PalmarisLongus5OW = nan;
f2_PalmarisLongus5OW = nan;
n1_PalmarisLongus5OW = nan;
n2_PalmarisLongus5OW = nan;
% **** determine R1 & R2 


a1_PalmarisLongus5OW = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_PalmarisLongus5OW = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);         % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PB =  JC2D(JCN+1,:);       % Top Point
u = PC1-PO;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_PalmarisLongus5OW = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_PalmarisLongus5OW = Da2*real(acos(CosTheta));


R1_PalmarisLongus5OW = nan;
R2_PalmarisLongus5OW = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_PalmarisLongus5OW, a2_PalmarisLongus5OW,...
                                  phi1_PalmarisLongus5OW, phi2_PalmarisLongus5OW,...
                                  R1_PalmarisLongus5OW, R2_PalmarisLongus5OW,...
                                  f1_PalmarisLongus5OW, f2_PalmarisLongus5OW, ...
                                  n1_PalmarisLongus5OW, n2_PalmarisLongus5OW];
                              
                              
                              
%--------------------------- Palmaris Longus 5 V-I(104->105)-----------------------------------------
k = 68;
JCN = 4;   % Joint center number
Da1 =  1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_PalmarisLongus5WI = 1;
f2_PalmarisLongus5WI = nan;
n1_PalmarisLongus5WI = -1;
n2_PalmarisLongus5WI = nan;
% **** determine R1 & R2 

a1_PalmarisLongus5WI = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_PalmarisLongus5WI = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);   % Origin
PT =  JC2D(JCN-1,:);      % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_PalmarisLongus5WI = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);  % Insertion
PB =  JC2D(JCN+1,:);        % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_PalmarisLongus5WI = Da2*real(acos(CosTheta));

R1_PalmarisLongus5WI = a1_PalmarisLongus5WI;
R2_PalmarisLongus5WI = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_PalmarisLongus5WI, a2_PalmarisLongus5WI,...
                                  phi1_PalmarisLongus5WI, phi2_PalmarisLongus5WI,...
                                  R1_PalmarisLongus5WI, R2_PalmarisLongus5WI,...
                                  f1_PalmarisLongus5WI, f2_PalmarisLongus5WI, ...
                                  n1_PalmarisLongus5WI, n2_PalmarisLongus5WI];
                              
                              
%% --------------------------- PronatorTeres(106->107) ------------------------------------
k = 69;
JCN = 3;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_PronatorTeres = -1;
f2_PronatorTeres = nan;
n1_PronatorTeres = -1;
n2_PronatorTeres = nan;
% **** determine R1 & R2 

a1_PronatorTeres = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_PronatorTeres = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_PronatorTeres = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_PronatorTeres = Da2*real(acos(CosTheta));


R1_PronatorTeres = 0.005;
R2_PronatorTeres = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_PronatorTeres, a2_PronatorTeres,...
                                  phi1_PronatorTeres, phi2_PronatorTeres,...
                                  R1_PronatorTeres, R2_PronatorTeres,...
                                  f1_PronatorTeres, f2_PronatorTeres, ...
                                  n1_PronatorTeres, n2_PronatorTeres];
                              
                              

%% --------------------------- Spinodeltoideus(108->109) ------------------------------------
k = 70;
JCN = 2;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_Spinodeltoideus = 1;
f2_Spinodeltoideus = nan;
n1_Spinodeltoideus = 1;
n2_Spinodeltoideus = nan;
% **** determine R1 & R2 

a1_Spinodeltoideus = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_Spinodeltoideus = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_Spinodeltoideus = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_Spinodeltoideus = Da2*real(acos(CosTheta));


R1_Spinodeltoideus = 0.002;
R2_Spinodeltoideus = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_Spinodeltoideus, a2_Spinodeltoideus,...
                                  phi1_Spinodeltoideus, phi2_Spinodeltoideus,...
                                  R1_Spinodeltoideus, R2_Spinodeltoideus,...
                                  f1_Spinodeltoideus, f2_Spinodeltoideus, ...
                                  n1_Spinodeltoideus, n2_Spinodeltoideus];                              
                              
                              
                              
%% --------------------------- Supraspinatus(110->111) ------------------------------------
k = 71;
JCN = 2;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_Supraspinatus = -1;
f2_Supraspinatus = nan;
n1_Supraspinatus = 1;
n2_Supraspinatus = nan;
% **** determine R1 & R2 

a1_Supraspinatus = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_Supraspinatus = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_Supraspinatus = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_Supraspinatus = Da2*real(acos(CosTheta));


R1_Supraspinatus = a2_Supraspinatus;
R2_Supraspinatus = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_Supraspinatus, a2_Supraspinatus,...
                                  phi1_Supraspinatus, phi2_Supraspinatus,...
                                  R1_Supraspinatus, R2_Supraspinatus,...
                                  f1_Supraspinatus, f2_Supraspinatus, ...
                                  n1_Supraspinatus, n2_Supraspinatus];
                              
                              
%% --------------------------- Subscapularis(112->113) ------------------------------------
k = 72;
JCN = 2;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_Subscapularis = -1;
f2_Subscapularis = nan;
n1_Subscapularis = 1;
n2_Subscapularis = nan;
% **** determine R1 & R2 

a1_Subscapularis = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_Subscapularis = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_Subscapularis = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_Subscapularis = Da2*real(acos(CosTheta));


R1_Subscapularis = 0.007;
R2_Subscapularis = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_Subscapularis, a2_Subscapularis,...
                                  phi1_Subscapularis, phi2_Subscapularis,...
                                  R1_Subscapularis, R2_Subscapularis,...
                                  f1_Subscapularis, f2_Subscapularis, ...
                                  n1_Subscapularis, n2_Subscapularis];                              
                              
                              
                              

%% --------------------------- Teres Major(114->115) ------------------------------------
k = 73;
JCN = 2;  % Joint center number
Da1 = 1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_TeresMajor = 1;
f2_TeresMajor = nan;
n1_TeresMajor = 1;
n2_TeresMajor = nan;
% **** determine R1 & R2 

a1_TeresMajor = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_TeresMajor = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_TeresMajor = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_TeresMajor = Da2*real(acos(CosTheta));


R1_TeresMajor = 0.004;
R2_TeresMajor = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_TeresMajor, a2_TeresMajor,...
                                  phi1_TeresMajor, phi2_TeresMajor,...
                                  R1_TeresMajor, R2_TeresMajor,...
                                  f1_TeresMajor, f2_TeresMajor, ...
                                  n1_TeresMajor, n2_TeresMajor];

                              
                              
%% --------------------------- Triceps Brachii Lateralis(116->117) ------------------------------------
k = 74;
JCN = 3;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_TricepsBrachiiLateralis = 1;
f2_TricepsBrachiiLateralis = nan;
n1_TricepsBrachiiLateralis = -1;
n2_TricepsBrachiiLateralis = nan;
% **** determine R1 & R2 

a1_TricepsBrachiiLateralis = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_TricepsBrachiiLateralis = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_TricepsBrachiiLateralis = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_TricepsBrachiiLateralis = Da2*real(acos(CosTheta));


R1_TricepsBrachiiLateralis = a2_TricepsBrachiiLateralis;
R2_TricepsBrachiiLateralis = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_TricepsBrachiiLateralis, a2_TricepsBrachiiLateralis,...
                                  phi1_TricepsBrachiiLateralis, phi2_TricepsBrachiiLateralis,...
                                  R1_TricepsBrachiiLateralis, R2_TricepsBrachiiLateralis,...
                                  f1_TricepsBrachiiLateralis, f2_TricepsBrachiiLateralis, ...
                                  n1_TricepsBrachiiLateralis, n2_TricepsBrachiiLateralis];                              
                              
                              
%% --------------------------- Triceps Brachii Long(118->119) ------------------------------------
k = 75;
PJCN = 2;  % Proximal joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_TricepsBrachiiLong = 1;
f2_TricepsBrachiiLong = 1;
n1_TricepsBrachiiLong = 1;
n2_TricepsBrachiiLong = -1;
% **** determine R1 & R2 


a1_TricepsBrachiiLong = sqrt(sum((Origins_2D_R(k,:) - JC2D(PJCN,:)).^2));
a2_TricepsBrachiiLong = sqrt(sum((Insertions_2D_R(k,:) - JC2D(PJCN+1,:)).^2));

PO = Origins_2D_R(k,:);  % Origin
PT =  JC2D(PJCN-1,:);    % Top Point
PC1 = JC2D(PJCN,:);      % First Center
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_TricepsBrachiiLong = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);     % Insertion
PB =  JC2D(PJCN+2,:);          % Bottom Point
PC2 = JC2D(PJCN+1,:);          % Second Center
u = PC2-PI;
v = PC2-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_TricepsBrachiiLong = Da2*real(acos(CosTheta));

R1_TricepsBrachiiLong = 0.005;
R2_TricepsBrachiiLong = a2_TricepsBrachiiLong;


% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_TricepsBrachiiLong, a2_TricepsBrachiiLong,...
                                  phi1_TricepsBrachiiLong, phi2_TricepsBrachiiLong,...
                                  R1_TricepsBrachiiLong, R2_TricepsBrachiiLong,...
                                  f1_TricepsBrachiiLong, f2_TricepsBrachiiLong, ...
                                  n1_TricepsBrachiiLong, n2_TricepsBrachiiLong];


 
%% --------------------------- Triceps Brachii Medial(120->121) ------------------------------------
k = 76;
JCN = 3;   % Joint center number
Da1 = -1;  % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = -1;  % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_TricepsBrachiiMedial = 1;
f2_TricepsBrachiiMedial = nan;
n1_TricepsBrachiiMedial = -1;
n2_TricepsBrachiiMedial = nan;
% **** determine R1 & R2 

a1_TricepsBrachiiMedial = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_TricepsBrachiiMedial = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_TricepsBrachiiMedial = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_TricepsBrachiiMedial = Da2*real(acos(CosTheta));


R1_TricepsBrachiiMedial = a2_TricepsBrachiiMedial;
R2_TricepsBrachiiMedial = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_TricepsBrachiiMedial, a2_TricepsBrachiiMedial,...
                                  phi1_TricepsBrachiiMedial, phi2_TricepsBrachiiMedial,...
                                  R1_TricepsBrachiiMedial, R2_TricepsBrachiiMedial,...
                                  f1_TricepsBrachiiMedial, f2_TricepsBrachiiMedial, ...
                                  n1_TricepsBrachiiMedial, n2_TricepsBrachiiMedial];
                              
                              

%% --------------------------- Teres Minor Medial(122->123) ------------------------------------
k = 77;
JCN = 2;   % Joint center number
Da1 = 1;   % Direction of a1 (counterclockwise is positive, clockwise is negative)
Da2 = 1;   % Direction of a2 (counterclockwise is positive, clockwise is negative)
f1_TeresMinorMedial = -1;
f2_TeresMinorMedial = nan;
n1_TeresMinorMedial = 1;
n2_TeresMinorMedial = nan;
% **** determine R1 & R2 

a1_TeresMinorMedial = sqrt(sum((Origins_2D_R(k,:) - JC2D(JCN,:)).^2));
a2_TeresMinorMedial = sqrt(sum((Insertions_2D_R(k,:) - JC2D(JCN,:)).^2));
PC1 = JC2D(JCN,:);        % Joint Center

PO = Origins_2D_R(k,:);    % Origin
PT =  JC2D(JCN-1,:);       % Top Point
u = PC1-PO;
v = PC1-PT;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi1_TeresMinorMedial = Da1*real(acos(CosTheta));


PI = Insertions_2D_R(k,:);      % Insertion
PB =  JC2D(JCN+1,:);            % Bottom Point
u = PC1-PI;
v = PC1-PB;
CosTheta = max(min(dot(u,v)/(norm(u)*norm(v)),1),-1);
phi2_TeresMinorMedial = Da2*real(acos(CosTheta));


R1_TeresMinorMedial = 0.002;
R2_TeresMinorMedial = nan;

% Muscle Geometric Parameters
MuscleGeometricParameters(k,:) = [a1_TeresMinorMedial, a2_TeresMinorMedial,...
                                  phi1_TeresMinorMedial, phi2_TeresMinorMedial,...
                                  R1_TeresMinorMedial, R2_TeresMinorMedial,...
                                  f1_TeresMinorMedial, f2_TeresMinorMedial, ...
                                  n1_TeresMinorMedial, n2_TeresMinorMedial];                              
                              
                              
                              
                              
%% Data to be written in 3D & 2D

% Find the column numbers by searching the header
[~, MuscleFullName_col] = find(cellfun(@(x) strcmp(x, 'Full Muscle Name'), txtMS));
MusculoskeletalData.Names = txtMS(2:end, MuscleFullName_col);

MusculoskeletalData.Three_Dimension.Origins = Origins;
MusculoskeletalData.Three_Dimension.Insertions = Insertions;
MusculoskeletalData.Three_Dimension.Joint_Centers = JC3D;
MusculoskeletalData.Three_Dimension.Segment_Length.Scapula_Length = Lscp_3D_MM;
MusculoskeletalData.Three_Dimension.Segment_Length.Upperarm_Length = Luarm_3D_MM;
MusculoskeletalData.Three_Dimension.Segment_Length.Forearm_Length = Lfarm_3D_MM;
MusculoskeletalData.Three_Dimension.Segment_Length.Carpals_Length = Lcar_3D_MM;
MusculoskeletalData.Three_Dimension.Musculotendon_Length = Muscle_Length_3D;

MusculoskeletalData.Two_Dimension.Origins = Origins_2D_R(:,1:2);
MusculoskeletalData.Two_Dimension.Insertions = Insertions_2D_R(:,1:2);
MusculoskeletalData.Two_Dimension.Joint_Centers = JC2D(:,1:2);
MusculoskeletalData.Two_Dimension.Segment_Length.Scapula_Length = Lscp_2D_MM;
MusculoskeletalData.Two_Dimension.Segment_Length.Upperarm_Length = Luarm_2D_MM;
MusculoskeletalData.Two_Dimension.Segment_Length.Forearm_Length = Lfarm_2D_MM;
MusculoskeletalData.Two_Dimension.Segment_Length.Carpals_Length = Lcar_2D_MM;
MusculoskeletalData.Two_Dimension.Musculotendon_Length = Muscle_Length_2D;

MusculoskeletalData.Two_Dimension.MuscleGeometricParameters = MuscleGeometricParameters;
MusculoskeletalData.Two_Dimension.MuscleGeometricParameterHeaders = {'a1, m', 'a2, m', 'phi1, rad', 'phi2, rad', 'R1, m', 'R2, m', 'f1', 'f2', 'n1', 'n2'};
MusculoskeletalData.Units = {'Attachment Point and Joint Center Coordinates', 'meters'; 'Segment and Muscle Lengths', 'meters'; 'Angles', 'radians'};


%% Writing Segment Angles and Joint Angles 

MusculoskeletalData.Angles.Segment.q1 = q1_MS_2D;
MusculoskeletalData.Angles.Segment.q2 = q2_MS_2D;
MusculoskeletalData.Angles.Segment.q3 = q3_MS_2D;
MusculoskeletalData.Angles.Segment.q4 = q4_MS_2D;

MusculoskeletalData.Angles.Joint.Shoulder = SHLa_MS_2D;
MusculoskeletalData.Angles.Joint.Elbow = ELBa_MS_2D;
MusculoskeletalData.Angles.Joint.Wrist = WRTa_MS_2D;

save('MusculoskeletalData.mat', 'MusculoskeletalData');


%% ------------------ Figure 2 (Plot muscles and segments along with numbered specific muscle in 2D) ----------------------------------------



% marker size
MaS=4;
% textsize
textsize = 12;


figure(2)
%--------------------- Plot joints and segments ------------------------------------------
for i=1:length(JC)
if i==1
    color = [1, 0, 0];
elseif i==2
    color = [0, 1, 0];
else
    color = [0, 0, 1];
end


s = scatter3(JC2D_R(i,1),JC2D_R(i,2),JC2D_R(i,3),'MarkerFaceColor',color,'MarkerEdgeColor',color);
h(i) = s;
hold on
alpha(s,.7)
end
h(i+1) = plot3([JC2D_R(1,1) JC2D_R(2,1)],[JC2D_R(1,2) JC2D_R(2,2)],[JC2D_R(1,3) JC2D_R(2,3)],'LineWidth',4);
hold on
h(i+2) = plot3([JC2D_R(2,1) JC2D_R(3,1)],[JC2D_R(2,2) JC2D_R(3,2)],[JC2D_R(2,3) JC2D_R(3,3)],'LineWidth',4);
hold on
Ls1 = h(i+1);
Ls2 = h(i+2);
Ls1.Color = [[0 0 0] 0.3];
Ls2.Color = [[0 0 0] 0.3];


%--------------------- Plot muscles ------------------------------------------
% Muscle colors
    colors = [
    0.8147, 0.9058, 0.1270; 0.9134, 0.6324, 0.0975; 0.2785, 0.5469, 0.9575;
    0, 0, 0; 0.9649, 0.1576, 0.9706; 0.9572, 0.4854, 0.8003; 0, 0, 0;
    0.1419, 0.4218, 0.9157; 0.7922, 0.9595, 0.6557; 0.0357, 0.8491, 0.9340;
    0, 0, 0; 0.6787, 0.7577, 0.7431; 0, 0, 0; 0.3922, 0.6555, 0.1712;
    0, 0, 0; 0.7060, 0.0318, 0.2769; 0, 0, 0; 0.0462, 0.0971, 0.8235;
    0, 0, 0; 0.6948, 0.3171, 0.9502; 0, 0, 0; 0.0344, 0.4387, 0.3816;
    0, 0, 0; 0.7655, 0.7952, 0.1869; 0, 0, 0; 0.4898, 0.4456, 0.6463;
    0, 0, 0; 0.7094, 0.7547, 0.2760; 0, 0, 0; 0.6797, 0.6551, 0.1626;
    0, 0, 0; 0.1190, 0.4984, 0.9597; 0, 0, 0; 0.3404, 0.5853, 0.2238;
    0.7513, 0.2551, 0.5060; 0, 0, 0; 0.6991, 0.8909, 0.9593; 0, 0, 0;
    0.5472, 0.1386, 0.1493; 0, 0, 0; 0.2575, 0.8407, 0.2543; 0, 0, 0;
    0.8143, 0.2435, 0.9293; 0, 0, 0; 0.3500, 0.1966, 0.2511; 0, 0, 0;
    0.6160, 0.4733, 0.3517; 0, 0, 0; 0.8308, 0.5853, 0.5497; 0, 0, 0;
    0.9172, 0.2858, 0.7572; 0, 0, 0; 0.7537, 0.3804, 0.5678; 0, 0, 0;
    0.0759, 0.0539, 0.5308; 0, 0, 0; 0.7792, 0.9340, 0.1299; 0, 0, 0;
    0.5688, 0.4694, 0.0119; 0, 0, 0; 0.3371, 0.1622, 0.7943; 0, 0, 0;
    0.3112, 0.5285, 0.1656; 0, 0, 0; 0.6020, 0.2630, 0.6541; 0, 0, 0;
    0.6892, 0.7482, 0.4505; 0, 0, 0; 0.0838, 0.2290, 0.9133; 0.1524, 0.8258, 0.5383;
    0.9961, 0.0782, 0.4427; 0.1067, 0.9619, 0.0046; 0.7749, 0.8173, 0.8687;
    0.0844, 0.3998, 0.2599; 0.8001, 0.4314, 0.9106; 0.1818, 0.2638, 0.1455;
    0.1361, 0.8693, 0.5797
    ];


nn = i+2;
k=0; % k is the row of each muscle in the excel file
for i=1:length(Insertions)

  k = k+1;
  hold on 
  %-------- 2D -------------------
  h(nn+k) = line([Origins_2D_R(k,1) Insertions_2D_R(k,1)],[Origins_2D_R(k,2) Insertions_2D_R(k,2)],[Origins_2D_R(k,3) Insertions_2D_R(k,3)],'color',colors(k,:),'LineWidth',1.5);
 
   if MuscleNumber == k
     txtO = num2str(Origin_nums(k,1));
     tO = text(Origins_2D_R(k,1), Origins_2D_R(k,2), Origins_2D_R(k,3),txtO,'FontSize',textsize, 'FontWeight','bold');
     hold on
     txtI = num2str(Insertion_nums(k,1));
     tI = text(Insertions_2D_R(k,1), Insertions_2D_R(k,2), Insertions_2D_R(k,3),txtI,'FontSize',textsize, 'FontWeight','bold');
     hold on
   end


  % -------- 2D ------------------
  plot3(Origins_2D_R(k,1),Origins_2D_R(k,2),Origins_2D_R(k,3),'o','LineWidth',1,...
                'MarkerEdgeColor',colors(k,:),...
                'MarkerFaceColor','none',...
                'MarkerSize',MaS)
  hold on
  plot3(Insertions_2D_R(k,1),Insertions_2D_R(k,2),Insertions_2D_R(k,3),'o','LineWidth',1,...
                'MarkerEdgeColor',colors(k,:),...
                'MarkerFaceColor','none',...
                'MarkerSize',MaS)
  hold on
  
  


  if sum(ismember(k,RBM(:,1)))==1
   k = k+1;
   hold on 
   %-------- 2D -------------------
   line([Origins_2D_R(k,1) Insertions_2D_R(k,1)],[Origins_2D_R(k,2) Insertions_2D_R(k,2)],[Origins_2D_R(k,3) Insertions_2D_R(k,3)],'color',colors(k-1,:),'LineWidth',1.5)
   hold on 
   
   
   if MuscleNumber == k-1
     txtO = num2str(Origin_nums(k,1));
     tO = text(Origins_2D_R(k,1), Origins_2D_R(k,2), Origins_2D_R(k,3),txtO,'FontSize',textsize, 'FontWeight','bold');
     hold on
     txtI = num2str(Insertion_nums(k,1));
     tI = text(Insertions_2D_R(k,1), Insertions_2D_R(k,2), Insertions_2D_R(k,3),txtI,'FontSize',textsize, 'FontWeight','bold');
     hold on
   end


   %-------- 2D -------------------
    plot3(Origins_2D_R(k,1),Origins_2D_R(k,2),Origins_2D_R(k,3),'o','LineWidth',1,...
                'MarkerEdgeColor',colors(k-1,:),...
                'MarkerFaceColor','none',...
                'MarkerSize',MaS)
    hold on
    plot3(Insertions_2D_R(k,1),Insertions_2D_R(k,2),Insertions_2D_R(k,3),'o','LineWidth',1,...
                'MarkerEdgeColor',colors(k-1,:),...
                'MarkerFaceColor','none',...
                'MarkerSize',MaS)
    hold on
    
  end



   if k==length(Insertions)
      break;
   end

end

if AI(3)==0
    AI(3) = 10^-10;
end


 
%-------------------- Segments (2D) --------------------------
S2D(1) = plot3([SIP_R(1,1) PMs_R(1,1)],[SIP_R(1,2) PMs_R(1,2)],[SIP_R(1,3) PMs_R(1,3)],'LineWidth',4);
hold on
S2D(2) = plot3([EIP_R(1,1) SIP_R(1,1)],[EIP_R(1,2) SIP_R(1,2)],[EIP_R(1,3) SIP_R(1,3)],'LineWidth',4);
hold on
S2D(3) = plot3([EIP_R(1,1) WIP_R(1,1)],[EIP_R(1,2) WIP_R(1,2)],[EIP_R(1,3) WIP_R(1,3)],'LineWidth',4);
hold on
S2D(4) = plot3([WIP_R(1,1) PMf_R(1,1)],[WIP_R(1,2) PMf_R(1,2)],[WIP_R(1,3) PMf_R(1,3)],'LineWidth',4);
hold on
S2D(1).Color = [[0 0 0] 0.6];
S2D(2).Color = [[0 0 0] 0.6];
S2D(3).Color = [[0 0 0] 0.6];
S2D(4).Color = [[0 0 0] 0.6];




legend([h(1:3), h(6:8), h(10), h(11), h(13), h(14), h(15), h(17), h(19), h(21), h(23), h(25), h(27), h(29), h(31), h(33), h(35), h(37), ...
        h(39), h(40), h(42), h(44), h(46), h(48), h(50), h(52), h(54), h(56), h(58), h(60), h(62), h(64), h(66), h(68), h(70), h(72), h(74), ...
        h(75), h(76), h(77), h(78), h(79), h(80), h(81), h(82)], ... %, h(83)], ...
                             'Shoulder','Elbow', 'Wrist', ...
                             'Acromiodeltoideus (1->2)', 'Anconeus (3->4)', 'Abductor pollicis longus (5->6->7)',...
                             'Biceps brachii(8->9)', 'Brachioradialis(10->11->12)','Brachialis(13->14)',...
                             'Coracobrachialis(15>16)', 'Extensor carpi radialis(17>18>19)', 'Extensor Carpi Ulnaris(20>21>22)', ...
                             'Extensor digitorum communis 2(23>24>25)', 'Extensor digitorum communis 3(26>27>28)', 'Extensor digitorum communis 4(29>30>31)','Extensor digitorum communis 5(32>33>34)',...
                             'Extensor digitorum lateralis 2(35>36>37)', 'Extensor digitorum lateralis 3(38>39>40)','Extensor digitorum lateralis 4(41>42>43)', 'Extensor digitorum lateralis 5(44>45>46)', ...
                             'Extensor pollicis longus 1 (47>48>49)', 'Extensor pollicis longus 2(50>51>52)', ...
                             'Epitrochlearis(53>54)', 'Flexor Carpi radialis(55>56>57)','Flexor Carpi Ulnaris(58>59>60)', ...
                             'Flexor digitorum profundus 1(61>62>63)', 'Flexor digitorum profundus 2(64>65>66)','Flexor digitorum profundus 3(67>68>69)', 'Flexor digitorum profundus 4(70>71>72)', 'Flexor digitorum profundus 5(73>74>75)', ...
                             'Flexor Digitorum Superficialis 2(76>77>78)', 'Flexor Digitorum Superficialis 3(79>80>81)', 'Flexor Digitorum Superficialis 4(82>83>84)', 'Flexor Digitorum Superficialis 5(85>86>87)', ...
                             'Infraspinatus(88>89>90)', ...
                             'Palmaris Longus 1(91>92>93)', 'Palmaris Longus 2(94>95>96)','Palmaris Longus 3(97>98>99)', 'Palmaris Longus 4(100>101>102)', 'Palmaris Longus 5(103>104>105)', ...
                             'Pronator Teres(106>107)', 'Spinodeltoideus(108>109)','Supraspinatus(110>111)', ...
                             'Subscapularis(112>113)', 'Teres Major(114>115)','Triceps Brachii Lateralis(116>117)', ...
                             'Triceps Brachii Long(118>119)', 'Triceps brachii medial(120>121)','Teres Minor(122>123)'); %,...
                             % 'Plane fitted to muscle attachment points');
                         
                         


box on
grid on
axis equal
xlabel('x(m)');
ylabel('z(m)');
zlabel('y(m)');



xlim([min(MAP(:,1))-0.001 max(MAP(:,1))+0.001])
ylim([min(MAP(:,2))-0.001 max(MAP(:,2))+0.001])
zlim([min(MAP(:,3))-0.001 max(MAP(:,3))+0.01])


az = 0;
el = 90;
view(az, el)

title('Selected Muscle (Numbered Attachments) and Other Muscles in Sagittal Plane')









%% ------------------ Figure 3 (Plot Specific Muscle in 3D & 2D and Segments) ----------------------------------------


% marker size
MaS=4;
% textsize
textsize = 12;


figure(3)
subplot(1, 2, 1)
%--------------------- Plot joints and segments ------------------------------------------
for i=1:length(JC)
if i==1
    color = [1, 0, 0];
elseif i==2
    color = [0, 1, 0];
else
    color = [0, 0, 1];
end


s = scatter3(JC(i,1),JC(i,2),JC(i,3),'MarkerFaceColor',color,'MarkerEdgeColor',color);
h3(i) = s;
hold on
alpha(s,.7)
end
h3(i+1) = plot3([JC(1,1) JC(2,1)],[JC(1,2) JC(2,2)],[JC(1,3) JC(2,3)],'LineWidth',4);
hold on
h3(i+2) = plot3([JC(2,1) JC(3,1)],[JC(2,2) JC(3,2)],[JC(2,3) JC(3,3)],'LineWidth',4);
hold on
Ls1 = h3(i+1);
Ls2 = h3(i+2);
Ls1.Color = [[0 0 0] 0.6];
Ls2.Color = [[0 0 0] 0.6];


%--------------------- Plot muscles ------------------------------------------
nn = i+2;
k=0; % k is the row of each muscle in the excel file
for i=1:length(Insertions)

  k = k+1;


   if MuscleNumber == k
     h3(nn+k) = line([Origins(k,1) Insertions(k,1)],[Origins(k,2) Insertions(k,2)],[Origins(k,3) Insertions(k,3)],'color',colors(k,:),'LineWidth',1.5);
     hold on 

     txtO = num2str(Origin_nums(k,1));
     tO = text(Origins(k,1), Origins(k,2), Origins(k,3),txtO,'FontSize',textsize, 'FontWeight','bold');
     hold on
     txtI = num2str(Insertion_nums(k,1));
     tI = text(Insertions(k,1), Insertions(k,2), Insertions(k,3),txtI,'FontSize',textsize, 'FontWeight','bold');
     hold on

     plot3(Origins(k,1),Origins(k,2),Origins(k,3),'o','LineWidth',1,...
                'MarkerEdgeColor',colors(k,:),...
                'MarkerFaceColor','none',...
                'MarkerSize',MaS)
     hold on
     plot3(Insertions(k,1),Insertions(k,2),Insertions(k,3),'o','LineWidth',1,...
                'MarkerEdgeColor',colors(k,:),...
                'MarkerFaceColor','none',...
                'MarkerSize',MaS)
     hold on
  end




  if sum(ismember(k,RBM(:,1)))==1
   k = k+1;

   if MuscleNumber == k-1
     line([Origins(k,1) Insertions(k,1)],[Origins(k,2) Insertions(k,2)],[Origins(k,3) Insertions(k,3)],'color',colors(k-1,:),'LineWidth',1.5)
     hold on 

     txtO = num2str(Origin_nums(k,1));
     tO = text(Origins(k,1), Origins(k,2), Origins(k,3),txtO,'FontSize',textsize, 'FontWeight','bold');
     hold on
     txtI = num2str(Insertion_nums(k,1));
     tI = text(Insertions(k,1), Insertions(k,2), Insertions(k,3),txtI,'FontSize',textsize, 'FontWeight','bold');
     hold on

     plot3(Origins(k,1),Origins(k,2),Origins(k,3),'o','LineWidth',1,...
                'MarkerEdgeColor',colors(k-1,:),...
                'MarkerFaceColor','none',...
                'MarkerSize',MaS)
     hold on
     plot3(Insertions(k,1),Insertions(k,2),Insertions(k,3),'o','LineWidth',1,...
                'MarkerEdgeColor',colors(k-1,:),...
                'MarkerFaceColor','none',...
                'MarkerSize',MaS)
     hold on
   end


  end



   if k==length(Insertions)
      break;
   end

end

%------------------- Scapula & paw and corresponding lines --------------------
Scapula_Seg = plot3([Mean_svs_sca(1,1) JC(1,1)],[Mean_svs_sca(1,2) JC(1,2)],[Mean_svs_sca(1,3) JC(1,3)],'LineWidth',4);
hold on
Scapula_Seg.Color = [[0 0 0] 0.6];
Paw_Seg = plot3([Mean_flmcp_fmmcp(1,1) JC(3,1)],[Mean_flmcp_fmmcp(1,2) JC(3,2)],[Mean_flmcp_fmmcp(1,3) JC(3,3)],'LineWidth',4);
hold on
Paw_Seg.Color = [[0 0 0] 0.6];

box on
grid on
axis equal
az = 10;
el = 60;

xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
 
xlim([min(MAP(:,1))-0.001 max(MAP(:,1))+0.001])
ylim([min(MAP(:,2))-0.001 max(MAP(:,2))+0.001])
zlim([min(MAP(:,3))-0.001 max(MAP(:,3))+0.01])
view(az, el)
title('3D View')



subplot(1, 2, 2)
%--------------------- Plot joints and segments ------------------------------------------
for i=1:length(JC)
if i==1
    color = [1, 0, 0];
elseif i==2
    color = [0, 1, 0];
else
    color = [0, 0, 1];
end


s = scatter3(JC2D_R(i,1),JC2D_R(i,2),JC2D_R(i,3),'MarkerFaceColor',color,'MarkerEdgeColor',color);
h(i) = s;
hold on
alpha(s,.7)
end
h(i+1) = plot3([JC2D_R(1,1) JC2D_R(2,1)],[JC2D_R(1,2) JC2D_R(2,2)],[JC2D_R(1,3) JC2D_R(2,3)],'LineWidth',4);
hold on
h(i+2) = plot3([JC2D_R(2,1) JC2D_R(3,1)],[JC2D_R(2,2) JC2D_R(3,2)],[JC2D_R(2,3) JC2D_R(3,3)],'LineWidth',4);
hold on
Ls1 = h(i+1);
Ls2 = h(i+2);
Ls1.Color = [[0 0 0] 0.6];
Ls2.Color = [[0 0 0] 0.6];


%--------------------- Plot muscles ------------------------------------------
nn = i+2;
k=0; % k is the row of each muscle in the excel file
for i=1:length(Insertions)

  k = k+1;

  if MuscleNumber == k
  hold on 
  %-------- 2D -------------------
  h(nn+k) = line([Origins_2D_R(k,1) Insertions_2D_R(k,1)],[Origins_2D_R(k,2) Insertions_2D_R(k,2)],[Origins_2D_R(k,3) Insertions_2D_R(k,3)],'color',colors(k,:),'LineWidth',1.5);
 
 
     txtO = num2str(Origin_nums(k,1));
     tO = text(Origins_2D_R(k,1), Origins_2D_R(k,2), Origins_2D_R(k,3),txtO,'FontSize',textsize, 'FontWeight','bold');
     hold on
     txtI = num2str(Insertion_nums(k,1));
     tI = text(Insertions_2D_R(k,1), Insertions_2D_R(k,2), Insertions_2D_R(k,3),txtI,'FontSize',textsize, 'FontWeight','bold');
     hold on


  % -------- 2D ------------------
  plot3(Origins_2D_R(k,1),Origins_2D_R(k,2),Origins_2D_R(k,3),'o','LineWidth',1,...
                'MarkerEdgeColor',colors(k,:),...
                'MarkerFaceColor','none',...
                'MarkerSize',MaS)
  hold on
  plot3(Insertions_2D_R(k,1),Insertions_2D_R(k,2),Insertions_2D_R(k,3),'o','LineWidth',1,...
                'MarkerEdgeColor',colors(k,:),...
                'MarkerFaceColor','none',...
                'MarkerSize',MaS)
  hold on
  end
  

  if MuscleNumber == k
      
  if sum(ismember(k,RBM(:,1)))==1
   k = k+1;
   hold on 
   %-------- 2D -------------------
   line([Origins_2D_R(k,1) Insertions_2D_R(k,1)],[Origins_2D_R(k,2) Insertions_2D_R(k,2)],[Origins_2D_R(k,3) Insertions_2D_R(k,3)],'color',colors(k-1,:),'LineWidth',1.5)
   hold on 
   
   
   % k = k-1;
     txtO = num2str(Origin_nums(k,1));
     tO = text(Origins_2D_R(k,1), Origins_2D_R(k,2), Origins_2D_R(k,3),txtO,'FontSize',textsize, 'FontWeight','bold');
     hold on
     txtI = num2str(Insertion_nums(k,1));
     tI = text(Insertions_2D_R(k,1), Insertions_2D_R(k,2), Insertions_2D_R(k,3),txtI,'FontSize',textsize, 'FontWeight','bold');
     hold on
  

   %-------- 2D -------------------
    plot3(Origins_2D_R(k,1),Origins_2D_R(k,2),Origins_2D_R(k,3),'o','LineWidth',1,...
                'MarkerEdgeColor',colors(k-1,:),...
                'MarkerFaceColor','none',...
                'MarkerSize',MaS)
    hold on
    plot3(Insertions_2D_R(k,1),Insertions_2D_R(k,2),Insertions_2D_R(k,3),'o','LineWidth',1,...
                'MarkerEdgeColor',colors(k-1,:),...
                'MarkerFaceColor','none',...
                'MarkerSize',MaS)
    hold on
    
    end

  end

   if k==length(Insertions)
      break;
   end

end

if AI(3)==0
    AI(3) = 10^-10;
end


 
%-------------------- Segments (2D) --------------------------
S2D(1) = plot3([SIP_R(1,1) PMs_R(1,1)],[SIP_R(1,2) PMs_R(1,2)],[SIP_R(1,3) PMs_R(1,3)],'LineWidth',4);
hold on
% S2D(2) = plot3([EIP_R(1,1) SIP_R(1,1)],[EIP_R(1,2) SIP_R(1,2)],[EIP_R(1,3) SIP_R(1,3)],'LineWidth',4);
hold on
% S2D(3) = plot3([EIP_R(1,1) WIP_R(1,1)],[EIP_R(1,2) WIP_R(1,2)],[EIP_R(1,3) WIP_R(1,3)],'LineWidth',4);
hold on
S2D(4) = plot3([WIP_R(1,1) PMf_R(1,1)],[WIP_R(1,2) PMf_R(1,2)],[WIP_R(1,3) PMf_R(1,3)],'LineWidth',4);
hold on
S2D(1).Color = [[0 0 0] 0.6];
% S2D(2).Color = [[0 0 0] 0.6];
% S2D(3).Color = [[0 0 0] 0.6];
S2D(4).Color = [[0 0 0] 0.6];




legend([h(1:3), h(6:8), h(10), h(11), h(13), h(14), h(15), h(17), h(19), h(21), h(23), h(25), h(27), h(29), h(31), h(33), h(35), h(37), ...
        h(39), h(40), h(42), h(44), h(46), h(48), h(50), h(52), h(54), h(56), h(58), h(60), h(62), h(64), h(66), h(68), h(70), h(72), h(74), ...
        h(75), h(76), h(77), h(78), h(79), h(80), h(81), h(82)],...
                             'Shoulder','Elbow', 'Wrist', ...
                             'Acromiodeltoideus (1->2)', 'Anconeus (3->4)', 'Abductor pollicis longus (5->6->7)',...
                             'Biceps brachii(8->9)', 'Brachioradialis(10->11->12)','Brachialis(13->14)',...
                             'Coracobrachialis(15>16)', 'Extensor carpi radialis(17>18>19)', 'Extensor Carpi Ulnaris(20>21>22)', ...
                             'Extensor digitorum communis 2(23>24>25)', 'Extensor digitorum communis 3(26>27>28)', 'Extensor digitorum communis 4(29>30>31)','Extensor digitorum communis 5(32>33>34)',...
                             'Extensor digitorum lateralis 2(35>36>37)', 'Extensor digitorum lateralis 3(38>39>40)','Extensor digitorum lateralis 4(41>42>43)', 'Extensor digitorum lateralis 5(44>45>46)', ...
                             'Extensor pollicis longus 1 (47>48>49)', 'Extensor pollicis longus 2(50>51>52)', ...
                             'Epitrochlearis(53>54)', 'Flexor Carpi radialis(55>56>57)','Flexor Carpi Ulnaris(58>59>60)', ...
                             'Flexor digitorum profundus 1(61>62>63)', 'Flexor digitorum profundus 2(64>65>66)','Flexor digitorum profundus 3(67>68>69)', 'Flexor digitorum profundus 4(70>71>72)', 'Flexor digitorum profundus 5(73>74>75)', ...
                             'Flexor Digitorum Superficialis 2(76>77>78)', 'Flexor Digitorum Superficialis 3(79>80>81)', 'Flexor Digitorum Superficialis 4(82>83>84)', 'Flexor Digitorum Superficialis 5(85>86>87)', ...
                             'Infraspinatus(88>89>90)', ...
                             'Palmaris Longus 1(91>92>93)', 'Palmaris Longus 2(94>95>96)','Palmaris Longus 3(97>98>99)', 'Palmaris Longus 4(100>101>102)', 'Palmaris Longus 5(103>104>105)', ...
                             'Pronator Teres(106>107)', 'Spinodeltoideus(108>109)','Supraspinatus(110>111)', ...
                             'Subscapularis(112>113)', 'Teres Major(114>115)','Triceps Brachii Lateralis(116>117)', ...
                             'Triceps Brachii Long(118>119)', 'Triceps brachii medial(120>121)','Teres Minor(122>123)');
                         
                         


box on
grid on
axis equal
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');

xlim([min(MAP(:,1))-0.001 max(MAP(:,1))+0.001])
ylim([min(MAP(:,2))-0.001 max(MAP(:,2))+0.001])
zlim([min(MAP(:,3))-0.001 max(MAP(:,3))+0.01])

% Sagittal View
az = 0;
el = 90;
view(az, el)
title('Sagittal View')
% print(gcf,[VA, '.png'],'-dpng','-r600');
















