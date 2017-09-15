
% prova disperata

GG = ss([0 -3;0 -4],2*ones(2,1),eye(2),zeros(2,1));

K = place(GG.A, GG.B, [-100 -4000] );

hello = feedback(GG,K);

CC = ss( zeros(2),zeros(2) , zeros(2) , [K(1) 0; 0 K(2)] );