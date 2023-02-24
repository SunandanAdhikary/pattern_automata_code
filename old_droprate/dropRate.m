% Cruise Control System from RTAS06.pdf 

% execution_pattern='10110111111000';
%   execution_pattern='111110000011111'; %(0.02 sec)[0.5 0.7]
 execution_pattern='111111111111111'; %(0.03 sec)[0.3 0.4]

%%%%%%%%%%%%%%%%%% Plant-Controller Design %%%%%%%%%%%%%%%%%%%%
% A=[0 1 0;0 0 1;-6.0476 -5.2856 -0.238];
% B=[0;0;2.4767];
% C=[1 0 0];
% D=0;

% Power system
A = [0.66 0.53;
   -0.53 0.13];
B = [0.34;    %B1=Bp1
    0.53];
C = eye(2);
%   L=[0.36 0.27;  -0.31 0.08];    
D = [0 ;
    0]; 
dimension=size(A,1);
% ESP
% A = [0.4450 -0.0458;1.2939 0.4402];
% B = [0.0550;4.5607];
% C = [0 1];
% D = 0;
% 
% dimension=size(A,1);
% Ts=0.1;

% Double integrator
% A = [0 1;
%       0 0];
% B = [0;
%       1];
% C = [1 0];
% D = 0;
% dimension=size(A,1);
% Ts=0.1;

%Cruise Control (from CTMS)
% A = -0.0500
% B = 0.001
% C = 1
% D = 0
% 
% dimension=size(A,1);
% Ts=0.01;

plant = ss(A, B, C, D);
[dim_s,dim_i]=size(B);       

ZOH = c2d(plant, Ts,'zoh');               % State space equation of plant-- discrete time
[Apd, Bpd, Cpd, Dpd] = ssdata(ZOH);
 
%%%%%%%%%%%--- controller design ---%%%%%%%%%%
  
Q_1c_1=10^3*(C'*C);
Q_12c_1=[0;0;0];
Q_2c_1=10^-5;
R_1c_1=0.005*(B*B');
R_2_1=0.005*eye(dimension);

% QXU=[Q_1c_1 Q_12c_1;Q_12c_1' Q_2c_1];
% QWU=[R_1c_1 zeros(dimension,1);zeros(1,dimension) R_2_1];

QXU = blkdiag(Q_1c_1,Q_2c_1);
QWU = blkdiag(R_1c_1,R_2_1)

lqg_reg=lqg(ZOH,QXU,QWU);
[Acd,Bcd,Ccd,Dcd]=ssdata(lqg_reg) ;

Ic=eye(dimension);
Z=zeros(dimension,dimension);   

A1=[Apd Bpd*Ccd;Bcd*Cpd Acd];
A0=[Apd Bpd*Ccd;Z Ic];

K=(-1)*lqg_reg.c;  % same with K1

Q=Q_1c_1;
R=Q_2c_1;

%{
[K,S,e] = dlqr(Apd, Bpd, Q, R);     % Get Optimal S matrix

Z=zeros(dimension,dimension);

A1=[Apd Bpd;-K zeros(dim_i,dim_i)]; % one interval delayed system
A0=[Apd Bpd;zeros(dim_i,dim_s) eye(dim_i)];

P=[S Z;Z S];  % Get Optimal P matrix correponding to composed matrix A1, A0 
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% beta_1=max(abs(eig(A1)))^2
% beta_0=max(abs(eig(A0)))^2


beta_1=max(abs(eig(Apd-Bpd*K)))^2
beta_0=max(abs(eig(Apd)))^2

l1=15;

l2=15;

unit_decay=0.005;
total_points=(1/unit_decay)-1;
decay=zeros(1,total_points);
rate=zeros(1,total_points);
epsilon=unit_decay;  %
% alpha=(1-epsilon)^(1/l1);  % damping rate- smaller epsilon -> higher alpha
alpha=log(1/epsilon)/l1;
r_min=(2*log(alpha)+log(beta_0))/(log(beta_0)-log(beta_1))
for k=1:total_points
    
    r_min=(2*log(alpha)+log(beta_0))/(log(beta_0)-log(beta_1))
    % r_min=(log(beta_0))/(log(beta_0)-log(beta_1))

    %%%%%%%%%%%%%%%%%%%%%%% Solve LMI %%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%========= lmi variables are r and P ================%%

    setlmis([]);
    r=lmivar(1,[1,0]);
    P=lmivar(1,[size(A1,1),1]);

    lmiterm([-1 1 1 r],1,1);                  % LMI # 1: r > r_min
    lmiterm([1 1 1 0],r_min);                      

    lmiterm([2 1 1 P],A1',A1);                % LMI # 2 : A1'*P*A1 <= beta_1*P
    lmiterm([2 1 1 P],-beta_1,1);        

    lmiterm([3 1 1 P],A0',A0);                % LMI # 3 : A0'*P*A0 <= beta_0*P
    lmiterm([3 1 1 P],-beta_0,1);        

    lmiterm([4 1 1 r],1,1);                   % LMI # 4 : r < 1
    lmiterm([4 1 1 0],-1);   

    lmiterm([-5 1 1 r],1,1);                   % LMI # 5 : r > 0

    lmiterm([-6 1 1 P],1,1);                  % LMI # 6  P > 0
    % lmiterm([6 1 1 0],0);                    

    lmisys = getlmis;                         % Create the LMI system

    [tmin,rP_feas] = feasp(lmisys) ;
    P = dec2mat(lmisys,rP_feas,P);            % display P matrix
    r = dec2mat(lmisys,rP_feas,r)    ;         % display r
    
    decay(k)=epsilon;
    rate(k)=r;
    k=k+1;
    
    epsilon=epsilon+unit_decay; 
    alpha=log(1/epsilon)/l1;
    
end
plot(decay,rate,'r'),xlabel('Decay Rate'),ylabel('Min. Exe. Rate');grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%