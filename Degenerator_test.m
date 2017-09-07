
% from simulink, it is TEMPORARY a main is required

sim('AW_1.slx')

plot(x3_output_comparison.time,x3_output_comparison.signals.values);
grid minor;
