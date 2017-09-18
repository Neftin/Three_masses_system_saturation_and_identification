
% from simulink, it is TEMPORARY a main is required

run('dida/threemass_fake_soft.m') % in case G is the fake one

close all;

H_inf_syn_3_mass % here the G and the pass band of the cloop is chosen

AW_synthesis_LMI % here the D_aw synthesis takes place

%%

satValue = 3.6;  %Saturation in ? (current test in Newton)
setPoint = 0.015; %meters

if ~exist('D_aw')
    D_aw = [0 0 0 0];
    display('anti_windup will be implemented, but not today')
else
    D_aw
end

sim('AW_1.slx')


%%
figure(5);
h = plot(x3_output_comparison.time,x3_output_comparison.signals.values);
set(h,'LineWidth',3);
legend('Unconstrained','saturated','AW')
xlabel('$t [s]$','interpreter','latex')
ylabel('$x_2 [m]$','interpreter','latex')
grid minor;

