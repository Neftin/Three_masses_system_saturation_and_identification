
% control definition



% system definition

load('sys3_tf.mat');

Plant = ss(idtf3);

% saturation definition

figure(6),step(Plant)

% 