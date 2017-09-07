
set(0,'DefaultFigureWindowStyle','docked')
clear all;
% IMPLEMENTA IL PULITORE DI ZERI E IL DE-IMMAGINATORE

load hinfstruct_demo.mat % carica esempio

% H infinity systhesys - prova
figure(4),
bodemag(G);

C0 = tunablePID('C','pi');  % tunable PI
        
a = realp('a',1);           % Create tunable parameter!

F0 = tf(a,[1 a]);           % filter parameterized by a!

wc = 1000;                          % target crossover frequency (è un fottuto hard drive dev'esse straveloce
s = tf('s');                        % trucchetto per definire le funzioni di trasferimento direttamente con s
LS = (1+0.001*s/wc)/(0.001+s/wc);   % funzione finale target (bi-proper) no marginally
bodemag(LS,{1e1,1e5}), grid, title('Target loop shape');

% presa come LS la oper loop TF, la chiudiamo per avere le specifiche a
% loop aperto

% Label the block I/Os
Wn = 1/LS;                                 % Peso del disturbo in catena di retroazione
Wn.u = 'nw';  Wn.y = 'n';                  % Input e output disturbo (n)
We = LS;    We.u = 'e';   We.y = 'ew';     % the same for the error before the controller
C0.u = 'e';   C0.y = 'u';                  
F0.u = 'yn';  F0.y = 'yf';

% Specify summing junctions
Sum1 = sumblk('e = r - yf');
Sum2 = sumblk('yn = y + n');

% Connect the blocks together
T0 = connect(G,Wn,We,C0,F0,Sum1,Sum2,{'r','nw'},{'y','ew'});

% impostazione della hinfstruct

rng('default')
opt = hinfstructOptions('Display','final','RandomStart',5);
T = hinfstruct(T0,opt);

showTunable(T)

C = getBlockValue(T,'C');
F = getValue(F0,T.Blocks);  % propagate tuned parameters from T to F

tf(F)

bode(LS,'r--',G*C*F,'b',{1e1,1e6}), grid,
title('Open-loop response'), legend('Target','Actual');

step(feedback(G*C,F)), grid, title('Closed-loop response');


