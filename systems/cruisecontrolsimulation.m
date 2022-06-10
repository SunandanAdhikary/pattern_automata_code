clear;
clf;
cla;
clc;
A = [ 0       1       0;
      0       0       1;
  -6.0476  -5.2856  -0.238];  
B = [0; 0; 2.4767];
C = [1 0 0];
D = 0;
sys_ss = ss(A,B,C,D);
co = ctrb(A,B);

Controllability = rank(co);
Ts = .01;
sys_d = c2d(sys_ss,Ts,'zoh')

co = ctrb(sys_d);
Controllability = rank(co);

A = sys_d.a;
B = sys_d.b;
C = sys_d.c;
D = sys_d.d;
p = 100;
Q = p*eye(3);
R = 1;
[K] = dlqr(A,B,Q,R)

QN = 1; 
RN = 1;

P = 1.0e-02 * [1    1   1;
               1    1   1;
               1    1   1];
for i=1:1000
    L=P*C'*inv(C*P*C'+RN);
    P=(eye(3)-L*C)*P;
    P=(A*P*A'+QN);
end
L

time=1000;

plot_vector1=zeros(1,time);
plot_vector2=zeros(1,time);
time_axis=zeros(1,time);
x=[3;
   0;
   0];
x_hat=[0;
       0;
       0];
u=-K*x_hat;
for i=1:time
    x_hat=A*x_hat+B*u+L*C*(x-x_hat);
    u=-K*x_hat;
    x=A*x+B*u;
    time_axis(i)=i;
    plot_vector1(i)=x(3);
    plot_vector2(i)=u;
end

hold on
plot(time_axis,plot_vector1,'-','Color',[0,0,0]),xlabel('Time(s)'),ylabel('Position')
plot(time_axis,plot_vector2,'-','Color',[0,0,0]),xlabel('Time(s)'),ylabel('Position')