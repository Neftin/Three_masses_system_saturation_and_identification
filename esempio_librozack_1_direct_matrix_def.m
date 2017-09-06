set(0,'DefaultFigureWindowStyle','docked')
clear all;
% IMPLEMENTA IL PULITORE DI ZERI E IL DE-IMMAGINATORE

%% Plant definition
fintozero = 0.00001;

nu = 2;
nq = 2;
nz = 2;
nw = 2;

% plant quantities: Plant = system object
% Nomenclatura as in the book pag. 85

A_p = [ -0.01     0   % plant
           0    -0.01];
         
B_p_u         =  eye(nu);       % B for the u input   
B_p_w         = zeros(nq);      % B for the q disturbance (w in the book)


C_p_y         = [ -0.4   0.5;   % C for y
                   0.3  -0.4]; 

C_p_z         = C_p_y;          % performance output (plant side)

D_p_yu  = zeros(2);           % D for u->y
D_p_yw  = zeros(2);           % D for q->y

D_p_zu  = zeros(2);           % D for u->z
D_p_zw  = D_p_yw - eye(2);    % D for q->z

np = size(A_p,1);

Plant = ss(A_p,[ B_p_u B_p_w ],C_p_y,[ D_p_yu D_p_yw ]); % For now no disturbances / z
% numbers of input output states
    nu = 2;
%   nq = 2;
    ny = 2;
% create cell arrays of names
    u = nameExpand('u',nu);
    wq = nameExpand('wq',nq);
    y = nameExpand('y',ny);
% name the plant quantities
Plant.inputName  = {u{1:end},wq{1:end}};
Plant.outputName = y;

%% Controller definition (No anti wind-up yet)
nc          = 2;

A_c   = zeros(nc);

B_c_y = eye(nc);

B_c_w = - B_c_y;

B_c_v = eye(nc) ; % full authority anti wind-up 

D_c_uy = [ 2    2.5;
           1.5    2];
            
D_c_uw = -D_c_uy;

C_c_u =  0.01*D_c_uy;

Control = ss(A_c,[ B_c_y B_c_w ] ,C_c_u ,[ D_c_uy D_c_uw ]);

% new names
    w = nameExpand('w',ny);
% define names
Control.InputName = {y{1:end},w{1:end}};
Control.outputName = u;

%% Closed loop

%   useful definition
Delta_u  = inv(eye(nu) - D_c_uy*D_p_yu); % write something on dimensions
Delta_y  = inv(eye(ny) - D_p_yu*D_c_uy);

A_cl     = [ A_p + B_p_u*Delta_u*D_c_uy*C_p_y,              B_p_u*Delta_u*C_c_u ;
                 B_c_y*Delta_y*C_p_y,         A_c + B_c_y*Delta_y*D_p_yu*C_c_u];

C_cl_z   = [ D_p_zu*Delta_u*D_c_uy*C_p_y + C_p_z, D_p_zu*Delta_u*C_c_u ];

C_cl_u   = [ Delta_u*D_c_uy*C_p_y         , Delta_u*C_c_u ];

B_cl_q  =  [ -B_p_u*Delta_u;
             -B_c_y*Delta_y*D_p_yu]; %OK

B_cl_w  =  [ B_p_w + B_p_u*Delta_u*(D_c_uy*D_p_yw + D_c_uw);
             B_c_w + B_c_y*Delta_y*(D_p_yu*D_c_uw + D_p_yw)]; %
      
D_cl_zq =  [ -D_p_zu*Delta_u ];

D_cl_zw =  [ D_p_zw + D_p_zu*Delta_u*(D_c_uy*D_p_yw + D_c_uw) ];

D_cl_uq =  [ eye(ny) - Delta_u ];

D_cl_uw =  [ Delta_u*(D_c_uw + D_c_uy*D_p_yw) ];

ncl = size(A_cl,1);

%DOUBLE CHECK THE DEFINITIONS!

%% Closed loop - antiwind up augmentation quantities
nv = nu + nc;

B_cl_v  = [zeros(nu,nc), B_p_u*Delta_u;
           eye(nc)     , B_c_y*Delta_y*D_p_yu];

D_cl_uv = [zeros(nu,nc) , Delta_u];

D_cl_zv = [zeros(nz,nc) , D_p_zu*Delta_u];
      
%% Algorithm - step 1-2

% LMI - yalmip

R_11 = sdpvar(np,np);                         % Plant order
R_22 = sdpvar(ncl-np,ncl-np);                 % Cloop order - Plant order 
R_12 = sdpvar(np,ncl-np);                     % "off diagonal" blocks

R = [ R_11 R_12; R_12' R_22 ];

% quantities to define

    % open loop
    I_nz  = eye(nz);

    % closed loop
    I_nw  = eye(nw);     
    
% FEASIBILITY CONDITIONS (step 2)

L =  [ R >= fintozero , R_11*A_p' + A_p*R_11 <= fintozero, R*A_cl' + A_cl*R <= fintozero ]

diagnostics = optimize(L);
if diagnostics.problem == 0
 disp('Feasible')
elseif diagnostics.problem == 1
 disp('Infeasible')
else
 disp('Something else happened')
end

%% Algorithm 1 step 3 

% open and closed loop conditions

% Luyapunov positivity

    % new variables
    gamma = sdpvar(1);

    % posititvity of common luya
    R    = [ R_11 R_12; R_12' R_22 ];
    % LMI(1)
    L(1) = R >= fintozero;
    
    % openloop conditions (witouth w!) (you can put it but it become only
    % zero
    OLC  = [ R_11*A_p'+A_p*R_11,  B_p_w                     ,                  R_11*C_p_z';
             B_p_w'            ,  -gamma*eye(size(B_p_w,2)) ,                       D_p_zw;
             C_p_z*R_11        ,  D_cl_zw                   ,              ( - gamma*I_nz)];
   
    L(2) = OLC <= fintozero; 
        
    % closed loop conditions (with w as reference)
    CLC  = [ R*A_cl' + A_cl*R  , B_cl_w       ,    R*C_cl_z';
             B_cl_w'           , -gamma*I_nw  ,     D_cl_zw';
             C_cl_z*R          , D_cl_zw      , -gamma*I_nz;];
         
    L(3) = CLC <= fintozero;

diagnostics = optimize(L,gamma);
if diagnostics.problem == 0
 disp('Feasible')
elseif diagnostics.problem == 1
 disp('Infeasible')
else
 disp('Something else happened')
end

%% Algorithm 1 step 4 - definition of matricionas 

%definitions
nu = size(B_p_u,2);

U = diag(sdpvar(nu,1));
Q = sdpvar(4);

Psi      = He([ A_cl*R        (B_cl_q*U + Q*C_cl_u')   B_cl_w                     Q*C_cl_z'   ; 
                zeros(nu,ncl) (D_cl_uq*U - U)          D_cl_uw                    U*D_cl_zq'  ;
                zeros(nz,ncl)  zeros(nz,nu)           -(gamma/2)*eye(nz)          D_cl_zw'    ;
                zeros(nw,ncl)  zeros(nw,nu)            zeros(nw,nz)        -(gamma/2)*eye(nw) ]); 

            
H         = [ B_cl_v'  D_cl_uv' zeros(nu+nc,nu) D_cl_zv' ]'; % this is the tough one to assemble because of the strange shape of the first term there are some zeros if 

G_u       = [ zeros(nu,ncl)  eye(nu)  zeros(nu,nz) zeros(nu,nw) ];

%% Algorithm 1 step 5 - the final inequalities

ni       = 0;
Lambda_u = sdpvar(nu+nc,nu);
gamma    = sdpvar(1);


 Lm_1 = -2*(1 - ni)*U + He((D_cl_uq*U + [zeros(nu,nc) eye(nu)]*Lambda_u));
 Lm_2 = Psi + G_u'*Lambda_u'*H' + H*Lambda_u*G_u;
 
 
% solving LMIs
L = [Lm_1 <= fintozero,Lm_2 <= fintozero ,U >= fintozero, gamma >= fintozero];

diagnostics = optimize(L,gamma);
if diagnostics.problem == 0
 disp('Feasible')
elseif diagnostics.problem == 1
 disp('Infeasible')
else
 disp('Something else happened')
end



