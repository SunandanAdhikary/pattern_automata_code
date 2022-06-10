PL = 0.2;
A = [-5 0 -100; 2 -2 0; 0 0.1 -0.08];
B = [0; 0; -0.1]; BPL = B*PL;
C = [0 0 1]; D = 0;
disp('(a)')
t=0:0.02:10;
[y, x] = step(A, BPL, C, D, 1, t);
figure(1), plot(t, y), grid
xlabel('t, sec'), ylabel('pu')
r =eig(A)

n=length(A);
for i=1:n;
S(:,n+1-i) = A^(n-i)*B;
end
if rank(S)~=n
error('System is not state controllable')
else
T=inv(S);
end
q=zeros(1,n); q(n)=1;
H=q*T;
p=poly(P);
AL=zeros(n);
for i=1:n+1
AL=AL+p(n+2-i)*A^(i-1);
end
K=H*AL;
Af=A-B*K;
fprintf('Feedback gain vector K \n'),
for i=1:n, fprintf(' %g',K(i)),fprintf(' '),end,fprintf('\n\n')
D=0;
if length(C(:,1)) > 1 return
else
[num, den]=ss2tf(A,B,C,D,1);
for i=1:length(num)
if abs(num(i)) <=1e-08 num(i)=0;else end,end
[numclsd, denclsd]=ss2tf(Af,B,C,D,1);
for i=1:length(numclsd)
if abs(numclsd(i)) <=1e-08 numclsd(i)=0;else end,end
fprintf('Uncompensated Plant')
GH = tf(num, den)
fprintf('Compensated system closed-loop')
T = tf(numclsd, denclsd)
fprintf('Compensated system matrix A - B*K \n')
disp(Af)
end