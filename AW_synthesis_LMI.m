
% LMI for the static regional full authority anti wind up

fakeZero = 0.0000001;

% Define the system (nomenclature refererence: ZACK_THEPROPOSITION2 eq 16)
%% Plant definition

% Nomenclatura as in the book pag. 85 (w as reference)

A_p    = GG.A; % plant quantities: % u input from controller
B_p_u  = GG.B;                     % y output feedback
C_p_y  = GG.C;                     % z performance output
D_p_yu = GG.D;                     % w external reference

B_p_w         =  zeros(size(B_p_u));               % B for reference (in the plant only for z is worth)

C_p_z         =   C_p_y;                           % performance output (plant side)

D_p_yw        = -D_p_yu;                           % D for w->y (null)

D_p_zu        = zeros(size(C_p_y,1));              % D for u->z (plant D)
D_p_zw        = D_p_yw - eye(size(D_p_yw,1));      % D for q->z (substraction for performance (z = y-w))


%% Controller definition 

A_c    = CC.A;    % control quantities: % y input % u output % w reference % v Aw input
B_c_y  = CC.B;
C_c    = CC.C;
D_c_y  = CC.D;                    

B_c_w  = - B_c_y;            % reference

B_c_v  = eye(size(A_c,1));   % full authority anti wind-up in practise
            
D_c_w = -D_c_y;            % reference (important)

D_c_v = eye(size(C_c,1)); % full authority anti wind-up in practise

%% Closed loop

%   useful definition
Delta_u  = inv(eye(size(D_c_y,1)) - D_c_y*D_p_yu); % write something on dimensions
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

%% TRUE LMI's     season finale, on netflix

% reference: equation 17a,17b,18 on ZACK_the_proposition_2

ncl = size(A_cl,1);
nv  = size(A_c,1) + size(C_c,1); % number of the antiwind up inputs (full authority)
% v1 has dimension of the controller state, v2 of the controller output eq (14)
nu  = size(C_c,1);

s = 1;        % fixed ellipsoidal set of "warranty" x' Q^-1 x =< s^2.
satValue = 1; % DA SISTEMARE

Q = sdpvar(ncl);          % Luya inverse
U = diag(sdpvar(nv,1));   % Diagonal U

L_17a(1) = Q >= fakeZero;
L_17a(2) = U >= fakeZero;

for kk = 1:size(C_c)
    
    Y{kk}     = sdpvar(nu,size(Q,1));
    
    L_17c(kk) = [ Q  ,    Y{kk}' ;
                  Y{kk},  (satValue(kk)^2/s^2)] >= fakeZero;
end











% vedi come estrarre le condizioni iniziali ammissibili (ellipsoide)

