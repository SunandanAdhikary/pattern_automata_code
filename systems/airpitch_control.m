A = [-0.313 56.7 0; -0.0139 -0.426 0; 0 56.7 0];
B = [0.232; 0.0203; 0];
C = [0 0 1];
D = [0];  

x0 = [1;1;1];                             % ini state
x = x0;         
t0 = [0];                               
t = t0;
k=0;
n=0;
h = 0.01*30;                                % sampling rate 
open_loop = ss(A,B,C,D);
eig_open= eig(A);                   
Ts=h;
ZOH_open= c2d(open_loop,Ts,'zoh');
[Ap1,Bp1,Cp1,Dp1] = ssdata(ZOH_open);
eig_openZOH=eig(Ap1)   
p = 50000;
Q = p*Cp1'*Cp1;
R = 0.0001;
K = dlqr(Ap1,Bp1,Q,R);

%p=100;
%K= [0.0556 0.3306 0.243];
A1= (Ap1-Bp1*K);
B1=zeros(size(A1,1),1);             %%%--Becomes zero
C1=Cp1;                             %%%--remains same,outputs
D1=Dp1;                             %%%--remains same i.e. zero
closed_loop = ss(A1,B1,C1,D1,Ts);
%    figure(10)
%    step(closed_loop)
eig_closed= eig(A1)
initial(closed_loop,x0)