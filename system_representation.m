set(0,'DefaultFigureWindowStyle','docked')

%   3 masse sopra il cielo, capitolo 2.

% system parametrical representation
    % state space parametrical representation of the system 3MDS
    
% parameters will be the classical mechanical quantities: k , m and c.

m1      = P(1);
m2      = P(2);
m3      = P(3);

c12     = P(4);
c23     = P(5);

c1      = P(6);
c2      = P(7);
c3      = P(8);

k1      = P(9);
k2      = P(10);
k3      = P(11);

gain_v  = P(12);

I = eye(3);

Z = zeros(3);

M = [m1 0 0;  
     0 m2 0; 
     0 0 m3];

 
 K = [k1   -k1        0;  
     -k1 k1+k2      -k2; 
      0    -k2     k2+k3];
  
 C = [+c1+c12   -c12        0
      -c12  +c2+c12+c23     -c23
        0        -c23      +c3+c23]; 
 
 b = [gain_v 0 0].';
 
 A = [Z I; -M\K -M\C];    % left divide for the inverse
 B = [Z(:,1); M\b];       % single input
 C = [I Z];               % multiple output
 D = Z(:,1);