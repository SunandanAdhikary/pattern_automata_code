A = [0.66 0.53;
       -0.53 0.13];
   B = [0.34;    %B1=Bp1
        0.53];
   C = eye(2);
%   L=[0.36 0.27;  -0.31 0.08];    
   D = [0 ;
        0];    
pwrsystem=ss(A,B,C,D,1);
step(pwrsystem);
eig(A)
[num,den]=ss2tf(A,B,C,D);
psys1=tf(num(2,:),den,1);
psys2=tf(num(1,:),den,1);
damp(psys1)
damp(psys2)