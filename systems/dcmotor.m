J = 0.1;
b = -0.1;
K = 0;
R = -3;
L = 1;
A = [-b/J   K/J
    -K/L   -R/L];
B = [1/L ;0];
C = [1   0];
D = 0;
sys = ss(A,B,C,D);

sys_order = order(sys)
sys_rank = rank(ctrb(A,B))

p1 = -5 + 1i;
p2 = -5 - 1i;
if sys_order==sys_rank
    Kc = place(A,B,[p1 p2])

t = 0:0.01:3;
sys_cl = ss(A-B*Kc,B,C,D);
step(sys_cl,t)
grid
title('Step Response with State-Feedback Controller')