
A=[1.00 0.01 0.00;
    -0.003 0.997 0.01;
    -0.0604 -0.0531 0.9974];
B=[0.0001;
    0.0001;
    0.0247];
C=eye(size(A));
D=zeros(size(B));
p = 100;
Q = p*eye(3);
R = 1;
K0 = dlqr(A,B,Q,R);
K1 = [-2092.6 -190.6 -39.0];
K2 = [-1457.9 -113.8 -30.6];
K3 = [1822.1 350.6 3.4];%stable
K4=[-872.5367 -131.494 -10.0972];
%x(k+1)0=A*x(k)+B*u;
%u(k+1)=K*x(k);
Ts=0.01;
K=K0;
cruise_contr_cl_dt=ss(A-B*K,zeros(size(B)),C,D,Ts);

cl_dt_poles = pole(cruise_contr_cl_dt)
% T=0:0.01:10;
% u=ones(size(T));
% lsim(cruise_contr,u,T)
initial(cruise_contr_cl_dt,[100;50;50])

