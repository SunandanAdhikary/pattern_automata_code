clear all;
A = [2];
B = [3];
C = [1];
D = [1];
sys= ss(A,B,C,D,1);
gain = place(A,B,0.2)
%initial(sys,10,1:0.1:5);
K=1:1:50;
s=size(K);
x=zeros(1,s(1,2));
u=zeros(1,s(1,2));
y=zeros(1,s(1,2));
x(1)=10;
u(1)=2;
y(1)=x(1)-u(1);

for k=1:1:s(1,2)-1
    x(k+1)= 2*x(k)+3*u(k);
    y(k+1)= x(k)+u(k);
    u(k+1)= -gain*x(k+1);    
%     if x(k+1)<=10
%        gain=0.03;
%     elseif x(k+1)<=20 && x(k+1)>=10
%            gain=0.2;
%     else
%        gain=1.2;
%     end
end
figure(1)
plot(K,x,K,u)
% P= sdpvar(size(A,1),size(A,2));
% prob= [A*P*A'-1.6*eye(size(P))*P zeros(size(A));
%         zeros(size(A)) -P];
% constr= prob <= 0
% 
% solvesdp(constr);
% P_lyap= double(P);
% V=x*P*x';
%P=dlyap(A,B,C);
%V=x'*P*x+u'*P*u;
V=x'*x+u'*u;
[X,Y]=meshgrid(x,u);
figure(3)
meshc(X,Y,V)
figure(2)
plot(x,V')