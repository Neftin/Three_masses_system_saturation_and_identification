set(0,'DefaultFigureWindowStyle','docked')

fintozero = 0.001;

% simulation quantities (from AW zack book pag 85) COMMENTA COME CRISTO
% COMANDA

t      = linspace(0,150,10000);   %time
steppo = ones(2,length(t)).*[0.63 0.79]';     % step signal

% plant

plant.A = [ -0.01     0
             0    -0.01];
         
plant.B_u =  eye(2);
plant.B_q = -eye(2);


plant.C = [ -0.4   0.5;
             0.3  -0.4];

plant.D = zeros(2,4);

Plant = ss(plant.A,[plant.B_u plant.B_q],plant.C,plant.D);
Plant.inputName  = {'U_1','U_2','q_1','q_2'};
Plant.outputName = {'z_1','z_2'};

% controller
nc          = 2;

control.A   = zeros(nc);

control.B_y = eye(2);

control.B_w = - control.B_y;

control.B_v = eye(nc); % full authority anti wind-up 

control.D_y = [ 2 2.5;
                1.5 2];
            
control.D_w = -control.D_y;

control.D_v = -control.D_w;

control.C   = 0.01*control.D_y;

Control = ss(control.A,[ control.B_y control.B_w control.B_v ], control.C, [ control.D_y control.D_w control.D_v ] );

Control.InputName = {'z_1' ,'z_2','w_1','w_2','v_1','v_2'};
Control.outputName = {'U_1','U_2'};

% cloop

UncoCloopBLO = (connect(Plant,Control,{'w_1','w_2','q_1','q_2','v_1','v_2'},{'z_1','z_2'},{'U_1','U_2'}));

UncoCloop    = ss(UncoCloopBLO);

UNCL_step = lsim(UncoCloopBLO,[ steppo ; zeros(2,length(steppo));  zeros(2,length(steppo)) ] ,t);

plot(t,UNCL_step);

% Introduzione della saturazione: si veda simulinko
%%
% algorithm 1 AW book

% LMI - yalmip

npxnp   = size(Plant.A);        % DOUBLE VALUES
nclxncl = size(UncoCloop.A);
np  = npxnp(1);                 % plant order
ncl = nclxncl(1);               % 
nw  = 2;                        % ref order order
nz  = 2;                        % output order
clear npxnp nclxncl

R_11 = sdpvar(np,np);                         % Plant order
R_22 = sdpvar(ncl-np,ncl-np);                 % Cloop order - Plant order 
R_12 = sdpvar(np,ncl-np);                     % "off diagonal" blocks

R = [ R_11 R_12; R_12' R_22 ];

% quantities to define

    % open loop
    A_p   = Plant.A;
    C_p_z = Plant.C;
    I_nz  = eye(nz);

    % closed loop
    temp    = getIOTransfer(UncoCloopBLO,{'w_1','w_2'},{'z_1','z_2'});
    temp    = ss(temp);
    A_cl    = temp.A;
    B_cl_w  = temp.B; % The total input is only w!
    C_cl_z  = temp.C; % The total output is only z!
    D_cl_zw = temp.D; % The total... oh well, I think you got it!
    I_nw    = eye(nw);     
    
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

% step 3 - conditions on gamma and R (refind R)
    % new variables
    gamma = sdpvar(1);

    % posititvity of common luya
    R    = [ R_11 R_12; R_12' R_22 ];
    % LMI(1)
    L(1) = R >= fintozero;
    
    % openloop conditions (witouth w)
    OLC  = [ R_11*A_p'+A_p*R_11,     R_11*C_p_z';
            C_p_z*R_11      ,    (- gamma*I_nz)];
   
    L(2) = OLC <= fintozero; 
        
    % closed loop conditions (with w as reference)
    CLC  = [ R*A_cl' + A_cl*R , B_cl_w       ,   R*C_cl_z';
             B_cl_w'           , -gamma*I_nw ,    D_cl_zw';
             C_cl_z*R         , D_cl_zw      , -gamma*I_nz;];
         
    L(3) = CLC <= fintozero;

diagnostics = optimize(L,gamma);
if diagnostics.problem == 0
 disp('Feasible')
elseif diagnostics.problem == 1
 disp('Infeasible')
else
 disp('Something else happened')
end
    
%% step 4 - define the matricionas

nu = 2;

U = diag(sdpvar(nu,1));
Q = sdpvar(4);

% saturated closed loop quantities
    %find the input matrix of the deadzone nonlinearity: it must be the
    %same as the inputs to saturate (write it in the thesys)
    temp    = getIOTransfer(UncoCloopBLO,{'q_1','q_2'},{'z_1','z_2'});
    temp    = ss(temp);
    B_cl_q  = temp.B;
    temp    = getIOTransfer(UncoCloopBLO,{'w_1','w_2'},{'U_1','U_2'});
    temp    = ss(temp);
    C_cl_u  = temp.C;
    temp    = getIOTransfer(UncoCloopBLO,{'q_1','q_2'},{'U_1','U_2'});
    temp    = ss(temp);
    D_cl_uq = temp.D;
    temp    = getIOTransfer(UncoCloopBLO,{'w_1','w_2'},{'U_1','U_2'});
    temp    = ss(temp);
    D_cl_uw = temp.D;
    temp    = getIOTransfer(UncoCloopBLO,{'q_1','q_2'},{'z_1','z_2'});
    temp    = ss(temp);
    D_cl_zq = temp.D;
    temp    = getIOTransfer(UncoCloopBLO,{'w_1','w_2'},{'z_1','z_2'});
    temp    = ss(temp);
    D_cl_zw = temp.D;
    nzwxnzw = size(D_cl_zw);
    nzw     = nzwxnzw(1);
    nu      = length(C_cl_u(:,1) );
    nz      = length(D_cl_zw(:,1));
    ncl     = length(A_cl(:,1));
    nw      = length(D_cl_uw(1,:));
    % anti uindappo related quantities
    temp    = getIOTransfer(UncoCloopBLO,{'v_1','v_2'},{'z_1','z_2'});
    temp    = ss(temp);
    B_cl_v  = temp.B;
    temp    = getIOTransfer(UncoCloopBLO,{'v_1','v_2'},{'U_1','U_2'});
    temp    = ss(temp);
    D_cl_uv  = temp.D;
    temp    = getIOTransfer(UncoCloopBLO,{'v_1','v_2'},{'z_1','z_2'});
    temp    = ss(temp);
    D_cl_zv  = temp.D;
    
He = zeros((ncl+nu+nw+nz),(ncl+nu+nw+nz));    %CAMBIA CAMBIA CAMBIA
    
He        = [ A_cl*R        (B_cl_q*U + Q*C_cl_u')   B_cl_w                     Q*C_cl_z'   ; % ok!
              zeros(nu,ncl) (D_cl_uq*U - U)          D_cl_uw                    U*D_cl_zq'  ;
              zeros(nz,ncl)  zeros(nz,nu)           -(gamma/2)*eye(nz)          D_cl_zw'    ;
              zeros(nw,ncl)  zeros(nw,nu)            zeros(nw,nz)        -(gamma/2)*eye(nw) ]; 

H         = [ B_cl_v' zeros(nu,nu) D_cl_uv' zeros(nu,nz) D_cl_zv' ];

G_u       = [ zeros(nu,ncl)  eye(nu)  zeros(nu,nz) zeros(nu,nw) ];
%%
% LMI vere e proprie
ni       = 0.001;
Lambda_u = sdpvar(nu);


 Lm_1 = -2*(1 - ni)*U + He*(D_cl_uq*U + [zeros(nu,nc) eye(nu)]*Lambda_u) ;
 Lm_2 = He + G_u'*Delta_u'*H' + H*Delta_u*G_u;
 
 
% solving LMIs
L = [Lm_1 <= fintozero,Lm_2 <= fintozero ];
 
diagnostics = optimize(L,gamma);
if diagnostics.problem == 0
 disp('Feasible')
elseif diagnostics.problem == 1
 disp('Infeasible')
else
 disp('Something else happened')
end

