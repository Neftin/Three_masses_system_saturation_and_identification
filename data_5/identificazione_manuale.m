
%identificazione_freq5

% identificazione manuale provvisoria (o definitva, chi lo sa)

sa1 = pole(tf(idtf1));

sa1 = sa1(1:4);

ze1 = zero(tf(idtf1));

ze1 = ze1(1:2);

%polo a 8.2 radianti

sa2 = [-2 + 1i*8.6*2*pi ; -2 - 1i*8.6*2*pi ];

ze2 = [-2 + 1i*8.2*2*pi ; -2 - 1i*8.2*2*pi ];

prova = [sa1; sa2];

zeppa = [ze1; ze2];

micro = zpk(zeppa,prova,0.49);

figure(1),bode(micro);

[magm11, phasem11, fm11] = bode(micro);
magm11 = reshape(magm11(1,1,:), length(fm11), 1);
phasem11 = reshape(phasem11(1,1,:), length(fm11), 1);

figure(2);

semilogx(fr1, 20*log10(magr1), 'linewidth', 1.5); hold on
semilogx(fm11/(2*pi), 20*log10(magm11), 'r', 'linewidth', 1)
xlim([0.1 1e1])
grid on
title('DOF 1');
xlabel('frequenza [Hz]')
ylabel('modulo [dB]')
hold off

% identificazione manuale DOF 2
% note that the poles are the same of the first DOF

% gli zeri sono da buttare in quanto zeri a fase non minima. (credo sia
% impossibile) CERCA DI DIMOSTRARLO

zenzero = [-0.95 + 1i*3.08*2*pi ; -0.95 - 1i*3.08*2*pi ];

limone = zpk(zenzero,prova,1);

figure(3),bode(limone);

limone = zpk(zenzero,prova,0.005/(dcgain(limone)));

[magm22, phasem22, fm22] = bode(limone);
magm22 = reshape(magm22(1,1,:), length(fm22), 1);
phasem22 = reshape(phasem22(1,1,:), length(fm22), 1);

figure(4);
semilogx(fr2, 20*log10(magr2), 'linewidth', 1.5); hold on
semilogx(fm22/(2*pi), 20*log10(magm22), 'r', 'linewidth', 1)
xlim([0.1 1e1])
grid on
xlabel('frequenza [Hz]')
ylabel('modulo [dB]')
hold off;

non_deludermi = ss(limone);

save('la_speranza','non_deludermi');
