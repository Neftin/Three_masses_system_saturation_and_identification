
%%avvia definizione sistema e controllo prima!

close all

% LMI for the static regional full authority anti-windup

fakeZero = 0.00000001; % It is used in the LMI definitions to avoid strict inequalities

% LMI Refererence: paper Zack P2 eq 17a,17c,18

%% import the controller and the plant data

% GG: plant and CC: controller

load('tambora_export')


%% Example 1 bookAWzack (comment it)

% GG = ss(zeros(2),zeros(2),zeros(2),zeros(2));
% CC = ss(zeros(2),zeros(2),zeros(2),zeros(2));
% 
% GG.A = [-0.01 0; 0 -0.01];
% GG.B = eye(2);
% GG.C = [-0.4 0.5; 0.3 -0.4];
% GG.D = zeros(2);
% 
% CC.A = zeros(2);
% CC.B = eye(2);
% CC.C = [0.02 0.025; 0.015 0.02];
% CC.D = [2 2.5; 1.5 2];



%% Plant definition

% Nomenclature of the AW book pag. 85 (w as reference) (book: modern
% anti-windup synthesis)

A_p    = GG.A; % plant quantities: % u input from controller
B_p_u  = GG.B;                     % y output feedback
C_p_y  = GG.C;                     % z performance output
D_p_yu = GG.D;                     % w external reference

B_p_w         =  zeros(size(B_p_u));                % B for reference (null)

C_p_z         =  C_p_y;                             % Performance output (same as y here)

D_p_yw        = -D_p_yu;                            % D for w->y (null)

D_p_zu        =  D_p_yu;                            % D for u->z (plant D)
D_p_zw        =  D_p_yw - eye(size(D_p_yw,1));      % D for q->z (substraction for performance (z = y-w))


%% Controller definition 

A_c    = CC.A;   % control quantities:  y input  u output  w reference
B_c_y  = -CC.B;  % Here the negative feedback (y -   w +).
C_c    = CC.C;   
D_c_y  = CC.D;                    

B_c_w  = -B_c_y; % reference (positive)
            
D_c_w  = -D_c_y; % reference

%% Closed loop 

% reference: Modern anti-windup synthesis book pg 78

Delta_u  = inv(eye(size(D_c_y,1)) - D_c_y*D_p_yu);                               % write something on dimensions
Delta_y  = inv(eye(size(D_p_yu,1)) - D_p_yu*D_c_y);

A_cl     = [ A_p + B_p_u*Delta_u*D_c_y*C_p_y,              B_p_u*Delta_u*C_c ;
                  B_c_y*Delta_y*C_p_y,         A_c + B_c_y*Delta_y*D_p_yu*C_c];

C_cl_z   = [ D_p_zu*Delta_u*D_c_y*C_p_y + C_p_z, D_p_zu*Delta_u*C_c ];

C_cl_u   = [ Delta_u*D_c_y*C_p_y         , Delta_u*C_c ];

B_cl_q  =  [ -B_p_u*Delta_u;
             -B_c_y*Delta_y*D_p_yu];

B_cl_w  =  [ B_p_w + B_p_u*Delta_u*(D_c_y*D_p_yw + D_c_w);
             B_c_w + B_c_y*Delta_y*(D_p_yu*D_c_w + D_p_yw)]; 

D_cl_zq =  [ -D_p_zu*Delta_u ];

D_cl_zw =  [ D_p_zw + D_p_zu*Delta_u*(D_c_y*D_p_yw + D_c_w) ];

D_cl_uq =  [ eye(size(Delta_u,1)) - Delta_u ];

D_cl_uw =  [ Delta_u*(D_c_w + D_c_y*D_p_yw) ];

% anti-windup quantities

B_cl_v  =  [ zeros(size(B_p_u,1),size(A_c,1)),       B_p_u*Delta_u ;
             eye(size(A_c,1))              ,  B_c_y*Delta_y*D_p_yu ];
         
D_cl_uv =  [ zeros(size(C_c,1),size(A_c,1)),               Delta_u ];

D_cl_zv =  [ zeros(size(C_p_z,1),size(A_c,1)),      D_p_zu*Delta_u ];

%% Verify the definition through impulse response. Closed loop matrices and Matlab 'feedback'
% function must return the same impulse response.

 impulsaRispostiva_matrici = impulse(ss(A_cl,B_cl_w,C_cl_z,D_cl_zw));
 impulsaRispostiva_comando = impulse(feedback(CC*GG,1));
 
 figure(1)
 h = plot([impulsaRispostiva_matrici impulsaRispostiva_comando])
 title('impulse response double check')
 set(h(2),'LineStyle','--');
 set(h(2),'LineWidth',2);
 legend('Command feedback','matrix direct c.loop definitions')
 grid on;
 
%% TRUE LMI's     season finale, on netflix

% strong well posedness
ni_swp = 0.001; 

% reference: equation 17a,17b,18 on paper ZACK_the_proposition_2

% Every LMI is called L_(equation number on paper) for the sake of clarity

ncl = size(A_cl,1);
nv  = size(A_c,1) + size(C_c,1); % number of the antiwind up inputs (full authority)
% v1 has dimension of the controller state, v2 of the controller output eq (14)

nu  = size(C_c,1);

s = 0.01;  % fixed ellipsoidal set of "warranty" x' Q^-1 x =< s^2.
satValue = 3.6;

% satValue from Degenerator test

Q           = sdpvar(ncl);          % Luya inverse
U           = diag(sdpvar(nu,1));   % Diagonal U
X           = sdpvar(nv,nu);        % For antiwindup gains
gammaSquare = sdpvar(1);            % gamma^2

L_17a(1) = Q >= fakeZero;
L_17a(2) = U >= fakeZero;

YY = [];

for kk = 1:size(C_c) %loop tp generate the L_17c iteratively
    
    Y{kk}     = sdpvar(1,size(Q,1));
    
    L_17c(kk) = [ Q  ,    Y{kk}' ;
                  Y{kk},  (satValue(kk)^2/s^2)] >= 0; % semi definite
              
    YY = [YY ;Y{kk}];
end

 CLC  = He( [ A_cl*Q,                               (B_cl_q*U + B_cl_v*X + YY'),       B_cl_w                   zeros(size(Q,1),1,size(D_cl_zw,1))
              C_cl_u*Q,                             (D_cl_uq*U + D_cl_uv*X - U),       D_cl_uw                  zeros(size(C_cl_u,1),size(D_cl_zw,1))  
              zeros(size(B_cl_w,2),size(Q,1))        zeros(size(B_cl_w,2),nu)          -eye(size(B_cl_w,2))/2,  zeros(size(B_cl_w,2),size(D_cl_zw,1))  
              C_cl_z*Q,                             (D_cl_zq*U + D_cl_zv*X),           D_cl_zw,                 -(gammaSquare/2)*eye(size(D_cl_zw,1))]);  
 
 L_18  = CLC <= -fakeZero; 
 
 L_swp = (D_cl_uq*U + D_cl_uv*X - U*(1 - ni_swp)) <= - fakeZero;
 
 L     = [ L_swp L_18 L_17a(1) L_17a(2), (gammaSquare >= fakeZero) ];
 
 % I don't forget L_17c! but the size is variable with the system!
  
 for jj = 1:size(C_c) % insert L_17c (all the instances)
     L = [ L L_17c(jj) ];
 end
   
diagnostics = optimize(L,gammaSquare);

if diagnostics.problem == 0
 disp('Feasible')
elseif diagnostics.problem == 1
 disp('Infeasible')
else
 disp('Something else happened')
end

% Extract anti wind up gains

D_aw  = value(U)\value(X) % eq 22
gamma = sqrt(value(gammaSquare))

%%

noteName = input('nome antiwindup sintesi:  ','s')

save([ 'E:/D_antiwindup_' noteName ],'D_aw');



% vedi come estrarre le condizioni iniziali ammissibili (ellipsoide)