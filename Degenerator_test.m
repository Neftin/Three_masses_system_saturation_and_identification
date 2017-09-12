
% from simulink, it is TEMPORARY a main is required

run('dida/threemass_fake_soft.m')

close all;

H_inf_syn_3_mass

%%

satValue = 3;  %Saturation in ? (current test this is Newton???)
setPoint = 0.015; %meters

sim('AW_1.slx')

figure(5);
plot(x3_output_comparison.time,x3_output_comparison.signals.values);
grid minor;


