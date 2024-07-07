% MATLAB code provided by Seyed Mohammad Ali Rahmati
% Affiliation: Biomechanics and Motor Control Lab, Department of Biological Sciences, Georgia Institute of Technology, Atlanta, GA.
%
% Description:
% This M-file creates MuscleLengthVelocityMomentArm corresponding to the musculotendon length, velocity and moment arm during locomotion
% This M-file also creates ScaledMuscleMorphologicalParameters corresponding to the musculotendon geometric parameters (a1 & a2)


clear;
clc;

% 1
Muscle_Length_Computation_Acromiodeltoideus_1_2
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 2
Muscle_Length_Computation_Anconeus_3_4
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 3
Muscle_Length_Computation_BicepsBrachii_8_9
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 4
Muscle_Length_Computation_Brachioradialis_10_11_12
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 5
Muscle_Length_Computation_Brachialis_13_14
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 6
Muscle_Length_Computation_Coracobrachialis_15_16
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 7
Muscle_Length_Computation_ExtensorCarpiRadialis_17_18_19
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4));  

% 8
Muscle_Length_Computation_ExtensorDigitorumCommunis2_23_24_25
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 9
Muscle_Length_Computation_ExtensorDigitorumCommunis3_26_27_28
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 10
Muscle_Length_Computation_ExtensorDigitorumCommunis4_29_30_31
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 11
Muscle_Length_Computation_ExtensorDigitorumCommunis5_32_33_34
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 12
Muscle_Length_Computation_ExtensorPollicisLongus1_47_48_49
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 13
Muscle_Length_Computation_ExtensorPollicisLongus2_50_51_52
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 14
Muscle_Length_Computation_Epitrochlearis_53_54
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 15
Muscle_Length_Computation_FlexorCarpiRadialis_55_56_57
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 16
Muscle_Length_Computation_FlexorCarpiUlnaris_58_59_60
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 17
Muscle_Length_Computation_FlexorDigitorumProfundus1_61_62_63
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 18
Muscle_Length_Computation_FlexorDigitorumProfundus2_64_65_66
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 19
Muscle_Length_Computation_FlexorDigitorumProfundus3_67_68_69
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 20
Muscle_Length_Computation_FlexorDigitorumProfundus4_70_71_72
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 21
Muscle_Length_Computation_FlexorDigitorumProfundus5_73_74_75
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 22
Muscle_Length_Computation_FlexorDigitorumSuper2_76_77_78
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 23
Muscle_Length_Computation_FlexorDigitorumSuper3_79_80_81
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 24
Muscle_Length_Computation_FlexorDigitorumSuper4_82_83_84
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 25
Muscle_Length_Computation_FlexorDigitorumSuper5_85_86_87
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 26
Muscle_Length_Computation_Infraspinatus_88_89_90
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 27
Muscle_Length_Computation_PalmarisLongus1_91_92_93
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 28
Muscle_Length_Computation_PalmarisLongus2_94_95_96
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 29
Muscle_Length_Computation_PalmarisLongus3_97_98_99
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 30
Muscle_Length_Computation_PalmarisLongus4_100_101_102
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 31
Muscle_Length_Computation_PalmarisLongus5_103_104_105
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 32
Muscle_Length_Computation_PronatorTeres_106_107
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 33
Muscle_Length_Computation_Spinodeltoideus_108_109
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 34
Muscle_Length_Computation_Supraspinatus_110_111
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 35
Muscle_Length_Computation_Subscapularis_112_113
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 36
Muscle_Length_Computation_TeresMajor114_115
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 37
Muscle_Length_Computation_TricepsBrachiiLateralis_116_117
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 38
Muscle_Length_Computation_TricepsBrachiiLong_118_119
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 39
Muscle_Length_Computation_TricepsBrachiiMedial_120_121
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 

% 40
Muscle_Length_Computation_TeresMinor_122_123
close(figure(1)); close(figure(2)); close(figure(3)); close(figure(4)); 






























