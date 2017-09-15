
%sistema smollo di prova (tensione-metri)
addpath ../

P = [ 2 1 3 3 3 10 0 0 800 800 175 ones(1,3) ];

[AA BB CC DD] = ss_mech_linear(P);

smoll = ss(AA,BB(:,1),CC(2,:),DD(1,1)); %first input third output!