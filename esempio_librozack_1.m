set(0,'DefaultFigureWindowStyle','docked')

fintozero = 0.00001;

% plant quantities: Plant = system object
%                   plant = only stucture, useful to define partial B,D,C matrices

plant.A = [ -0.01     0   % plant
             0    -0.01];
         
plant.B_u =  eye(2);      % B for the u input   
plant.B_q = -eye(2);      % B for the q disturbance


plant.C = [ -0.4   0.5;   % C for y
             0.3  -0.4]; 

plant.D_yu = zeros(2,2);  % D for u->y
plant.D_yq = zeros(2,2);  % D for q->y

Plant = ss(plant.A,[plant.B_u plant.B_q],plant.C, [plant.D_yu plant.D_yq]);
% numbers of input output states
    nu = 2;
    nq = 2;
    ny = 2;
% create cell arrays of names
    u = nameExpand('u',nu);
    q = nameExpand('q',nq);
    y = nameExpand('y',ny);
% name the plant quantities
Plant.inputName  = {u{1:end} q{1:end}};
Plant.outputName = y;

%% controller
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

Control = ss(control.A,[ control.B_y control.B_w control.B_v zeros(nc)], control.C, [ control.D_y control.D_w control.D_v eye(nc) ] );

nv = nu + nc; % anti wind up signals
% new names
    v = nameExpand('v',nv);
    w = nameExpand('w',ny);
% define names
Control.InputName = {y{1:end},w{1:end},v{1:end}};
Control.outputName = u;
%%
% cloop
perfoSum_1   = sumblk('z_1 = y(1) - w_1',1);
perfoSum_2   = sumblk('z_2 = y(2) - w_2',1);

UncoCloopBLO = (connect(Plant,Control,perfoSum_1,perfoSum_2,{'w_1','w_2','q(1)','q(2)','v_1','v_2','v_3','v_4'},{'y(1)','y(2)','z_1','z_2'},{'u_1','u_2'}));

UncoCloop    = ss(UncoCloopBLO);

%UNCL_step = lsim(UncoCloopBLO,[ steppo ; zeros(2,length(steppo));  zeros(2,length(steppo)) ] ,t);

%plot(t,UNCL_step);

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
    C_cl_z  = temp.C; % The performance output z!
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
    temp    = getIOTransfer(UncoCloopBLO,{'q(1)','q(2)'},{'y(1)','y(2)','z_1','z_2'});
    temp    = ss(temp);
    B_cl_q  = temp.B;
    temp    = getIOTransfer(UncoCloopBLO,{'w_1','w_2'},{'u_1','u_2'});
    temp    = ss(temp);
    C_cl_u  = temp.C;
    temp    = getIOTransfer(UncoCloopBLO,{'q(1)','q(2)'},{'u_1','u_2'});
    temp    = ss(temp);
    D_cl_uq = temp.D;
    temp    = getIOTransfer(UncoCloopBLO,{'w_1','w_2'},{'u_1','u_2'});
    temp    = ss(temp);
    D_cl_uw = temp.D;
    temp    = getIOTransfer(UncoCloopBLO,{'q(1)','q(2)'},{'z_1','z_2'});
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
    temp    = getIOTransfer(UncoCloopBLO,{'v_1','v_2','v_3','v_4'},{'y(1)','y(2)','z_1','z_2'});
    temp    = ss(temp);
    B_cl_v  = temp.B;
    temp    = getIOTransfer(UncoCloopBLO,{'v_1','v_2','v_3','v_4'},{'u_1','u_2'});
    temp    = ss(temp);
    D_cl_uv = temp.D;
    temp    = getIOTransfer(UncoCloopBLO,{'v_1','v_2','v_3','v_4'},{'z_1','z_2'});
    temp    = ss(temp);
    D_cl_zv = temp.D;
    
Psi      = He([ A_cl*R        (B_cl_q*U + Q*C_cl_u')   B_cl_w                     Q*C_cl_z'   ; % ok!
                zeros(nu,ncl) (D_cl_uq*U - U)          D_cl_uw                    U*D_cl_zq'  ;
                zeros(nz,ncl)  zeros(nz,nu)           -(gamma/2)*eye(nz)          D_cl_zw'    ;
                zeros(nw,ncl)  zeros(nw,nu)            zeros(nw,nz)        -(gamma/2)*eye(nw) ]); 

H         = [ B_cl_v'  D_cl_uv' zeros(nu+nc,nu) D_cl_zv' ]';

G_u       = [ zeros(nu,ncl)  eye(nu)  zeros(nu,nz) zeros(nu,nw) ];

%%
% LMI vere e proprie
ni       = 0.001;
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



