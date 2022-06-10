clear;clf;cla;clc;

A= [2];
B= [2];
C= [1];
D= [1];
%x_dot = A*x+B*u;
%y= C*x;
t= 0:0.01:5;
x0= 1;

sys_ol= ss(A,B,C,D);

%figure(1)
[Y1,T,X1] = initial(sys_ol,x0,t);
poles_ol = eig(A)

p=3;
Q=p.*C'*C;
R= 1;
K=lqr(A,B,Q,R);

poles_cl = eig(A-B*K) 

sys_cl= ss((A-B*K),B,C,D);


figure(1)
[Y2,T,X2] = initial(sys_cl,x0,t);
plotyy(T,X2,T,X1);
syms s;
[N,D]= ss2tf(A,B,C,D);
G = tf(N,D)
%ilaplace(G,s)
%figure(3)
%plot(t,exp(max(poles_cl)*t));