
set(0,'DefaultFigureWindowStyle','docked')

%% Dummy values for mech quantities (AW trial 1)

P = [1.2 1.3 1.4 0 0 3 2 2 800 800 800 600 0 0 ];

%% Define dummy system with previous dummy mech quantities

[A B C D] = ss_mech_linear(P);

sys_dum = ss(A,B,C,D);

clear A B C D;

open('AW_1.slx');