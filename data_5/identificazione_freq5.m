clc
close all
clear all
%%
ts = 0.005;

enc_counts2m = 7.252372446811487e+06; 

load('../data_1/force_i2N.mat') %  convertion factor between FORCE and SIGNAL 

%% 
% load('sweep.mat')
load('PRBS_16_44hope2.mat');

figure
subplot(4,1,1)
plot(time, input_signal)
subplot(4,1,2)
plot(time, enc.x1)
subplot(4,1,3)
plot(time, enc.x2)
subplot(4,1,4)
%pulitura enc.x3
enc.x3(find(abs(enc.x3)>10^7)) = 0;
%replot
plot(time, enc.x3)
%axis([-inf inf -200000 200000])


t_start = 1;
t_end   = 100;

idx = time>t_start & time<t_end;
t = time(idx) - t_start;
u = input_signal(idx);
u = u - u(1);
y1 = enc.x1(idx);
y1 = y1 - y1(1);
y2 = enc.x2(idx);
y2 = y2 - y2(1);
y3 = enc.x3(idx);
y3 = y3 - y3(1);
y1 = y1 / enc_counts2m;
y2 = y2 / enc_counts2m;
y3 = y3 / enc_counts2m;
u = u * force_i2N;

%% add settings

opt  = tfestOptions('EnforceStability',false)
%opt = tfestOptions('WeightingFilter',[0.1 9])

%% GDL 1
[tfreal1, fr1] = tfestimate(u, y1, [], [], [], 1/ts);
magr1   = abs(tfreal1);
phaser1 = angle(tfreal1);

tfreal1 = tfreal1(find(fr1 < 10));
fr1_ = fr1(find(fr1 < 10));
phaser1(phaser1>0) = phaser1(phaser1>0) - 2*pi;

data1 = frd(tfreal1, fr1_, 'FrequencyUnit', 'Hz');

idtf1 = tfest(data1, 6, 4,opt)
[magm1, phasem1, fm1] = bode(idtf1);
magm1 = reshape(magm1(1,1,:), length(fm1), 1);
phasem1 = reshape(phasem1(1,1,:), length(fm1), 1);

%% GDL 2
[tfreal2, fr2] = tfestimate(u, y2, [], [], [], 1/ts);
magr2   = abs(tfreal2);
phaser2 = angle(tfreal2);
phaser2(phaser2 > -0) = phaser2(phaser2 > 0) - 2*pi;
phaser2(fr2 > 8 & phaser2 > -300*pi/180) = phaser2(fr2 > 8 & phaser2 > -300*pi/180) - 2*pi;

tfreal2 = tfreal2(find(fr2 < 10));
fr2_ = fr2(find(fr2 < 10));
phaser2(phaser2>0) = phaser2(phaser2>0) - 2*pi;

data2 = frd(tfreal2, fr2_, 'FrequencyUnit', 'Hz');

idtf2 = tfest(data2, 6, 2,opt)
[magm2, phasem2, fm2] = bode(idtf2);
magm2 = reshape(magm2(1,1,:), length(fm2), 1);
phasem2 = reshape(phasem2(1,1,:), length(fm2), 1);

%% GDL 3
[tfreal3, fr3] = tfestimate(u, y3, [], [], [], 1/ts);
magr3   = abs(tfreal3);
phaser3 = angle(tfreal3);
phaser3(phaser3 > -0) = phaser3(phaser3 > 0) - 2*pi;
phaser3(fr3 > 5 & phaser3 > -300*pi/180) = phaser3(fr3 > 5 & phaser3 > -300*pi/180) - 2*pi;

tfreal3 = tfreal3(find(fr3 < 10));
fr3_ = fr3(find(fr3 < 10));
data3 = frd(tfreal3, fr3_, 'FrequencyUnit', 'Hz');

idtf3 = tfest(data3, 6, 0,opt)
[magm3, phasem3, fm3] = bode(idtf3);
magm3 = reshape(magm3(1,1,:), length(fm3), 1);
phasem3 = reshape(phasem3(1,1,:), length(fm3), 1);

%%
figure; 

subplot(2,3,1)
semilogx(fr1, 20*log10(magr1), 'linewidth', 1.5); hold on
semilogx(fm1/(2*pi), 20*log10(magm1), 'r', 'linewidth', 1)
xlim([0.5 1e1])
grid on
xlabel('freqenza [Hz]')
ylabel('modulo [dB]')

subplot(2,3,4)
semilogx(fr1, phaser1*180/pi, 'linewidth', 1.5); hold on
semilogx(fm1/(2*pi), phasem1, 'r', 'linewidth', 1)
xlim([0.5 1e1])
grid on
xlabel('freqenza [Hz]')
ylabel('fase [°]')

%

subplot(2,3,2)
semilogx(fr2, 20*log10(magr2), 'linewidth', 1.5); hold on
semilogx(fm2/(2*pi), 20*log10(magm2), 'r', 'linewidth', 1)
xlim([0.5 1e1])
grid on
xlabel('freqenza [Hz]')
ylabel('modulo [dB]')

subplot(2,3,5)
semilogx(fr2, phaser2*180/pi, 'linewidth', 1.5); hold on
semilogx(fm2/(2*pi), phasem2, 'r', 'linewidth', 1)
xlim([0.5 1e1])
grid on
xlabel('freqenza [Hz]')
ylabel('fase [°]')

%

subplot(2,3,3)
semilogx(fr3, 20*log10(magr3), 'linewidth', 1.5); hold on
semilogx(fm3/(2*pi), 20*log10(magm3), 'r', 'linewidth', 1)
xlim([0.5 1e1])
grid on
xlabel('freqenza [Hz]')
ylabel('modulo [dB]')

subplot(2,3,6)
semilogx(fr3, phaser3*180/pi, 'linewidth', 1.5); hold on
semilogx(fm3/(2*pi), phasem3, 'r', 'linewidth', 1)
xlim([0.5 1e1])
grid on
xlabel('freqenza [Hz]')
ylabel('fase [°]')

%%
y1sim = lsim(idtf1, u, t);
y2sim = lsim(idtf2, u, t);
y3sim = lsim(idtf3, u, t);

% figure
% subplot(4,1,1)
% plot(t, u)
% subplot(4,1,2)
% plot(t, y1); hold on;
% plot(t, y1sim); hold on;
% subplot(4,1,3)
% plot(t, y2); hold on;
% plot(t, y2sim); hold on;
% subplot(4,1,4)
% plot(t, y3); hold on;
% plot(t, y3sim); hold on;

%% save the file only

    save('sys3_tf','idtf3')
    save('sys2_tf','idtf2')
    save('sys1_tf','idtf1')





