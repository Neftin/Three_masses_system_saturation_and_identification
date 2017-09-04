function [ A,B,C,D ] = ss_mech_linear( P )
%it return the matrices ABCD of the linear state space from the parameters:
% m1,m2,m3,c12,c23,c1,c2,c3,k1,k2,k3,gain_1,gain_2,gain_3. The input is
% force on every mass. output is position.

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

gain_1  = P(12);
gain_2  = P(13);
gain_3  = P(14);

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
 
 b = [gain_1 0 0;
      0 gain_2 0;
      0 0 gain_3].';
 
 A = [Z I; -M\K -M\C];    % left divide for the inverse
 B = [Z; M\b];            % multiple input
 C = [I Z];               % multiple output
 D = Z;
 
 