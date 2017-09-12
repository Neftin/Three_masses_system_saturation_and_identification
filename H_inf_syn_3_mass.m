
%  #######################################################
%  #  _______   __  _____ ______ _______ _ ____   _____  #
%  #         \ |  |   ___|   ___|__   __| |    \ |       #
%  #          \|  |  __| |  __|    | |  | |     \|       #
%  #       |\     | |____| |       | |  | |   |\         #
%  #  _____| \____| _____|_|       |_|  |_|___| \______  #
%  #                                                     #
%  #######################################################

%% H - infinity systhesys for the 3-mass system
set(0,'DefaultFigureWindowStyle','docked')
clear all;
% IMPLEMENTA IL PULITORE DI ZERI E IL DE-IMMAGINATORE
load('data_4/sys1_tf.mat'); % the Plant
load('data_4/sys2_tf.mat'); % the Plant
load('data_3/sys3_tf.mat'); % the Plant FROM NEW DATA

%_________CHOOSE YOUR PLANT____________

run('./dida/threemass_fake_soft')

              %G = idtf2;
              G = smoll;

             
figure(1)
bode(G);

G.u = 'u';
G.y = 'y';

% controller structure creation:

% first trial: PID + order 1 filter


Co = tunablePID('COpid','pid');  % tunable PID

a0 = realp('a0',1);
Fo = tf([a0],[1 a0]);

CoFo = series(Co,Fo);

%____________choose your pass-band________________
               wc = 20;   

s = tf('s');                        % trucchetto per definire le funzioni di trasferimento direttamente con s
LS = (1+0.001*s/wc)/(0.001+s/wc);   % funzione finale target (bi-proper) no marginally

figure(2)
bodemag(LS,{1e-1,1e2}), grid, title('Target loop shape');

% FAI DISEGNONE!

% Label the block I/Os
Wn = 1/LS;                                 % Peso del disturbo in catena di retroazione
Wn.u = 'nw';  Wn.y = 'n';                  % Input e output disturbo (n)

We = LS;    We.u = 'e';   We.y = 'ew';     % the same for the error before the controller
CoFo.u = 'e';   CoFo.y = 'u';   

% Specify summing junctions
Sum1 = sumblk('e = r - yn');
Sum2 = sumblk('yn = y + n');

% Connect the blocks together
StatoSpesso = connect(G,Wn,We,CoFo,Sum1,Sum2,{'r','nw'},{'y','ew'});

% impostazione della hinfstruct

rng('default')
opt = hinfstructOptions('Display','final','RandomStart',5);
StatoSveglio = hinfstruct(StatoSpesso,opt);

showTunable(StatoSveglio)

COpid  = getBlockValue(StatoSveglio,'COpid');
aa     = getBlockValue(StatoSveglio,'a0');
F      = tf([aa],[1 aa]);

C = series(COpid,F);

tf(C)

figure(3)
bode(LS,'r--',G*C,'b',{1e-1,1e3}), grid,
title('Open-loop response'), legend('Target','Actual')

figure(4)
step(feedback(G*C,1)), grid, title('Closed-loop response')

CLGC = feedback(G*C,1);

GG   = ss(G); %per il main

CC   = ss(C); %per il main


