
%sistema smollo di prova (tensione-metri)

cd('..\')

P = [ 1.5 1.4 1.2 3 2 2 0 0 700 800 0 6*ones(1,3) ];

[AA BB CC DD] = ss_mech_linear(P);

smoll = ss(AA,BB(:,1),CC(3,:),DD(1,1)); %first input third output!

cd('dida')