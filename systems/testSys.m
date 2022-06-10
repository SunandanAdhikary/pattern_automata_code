A=[1.38 -0.2077 6.715 -5.676;-0.5814 -4.29 0 0.675;
    1.067 4.273 -6.654 5.893;0.048 4.273 1.343 -2.104];
B=[0 0;5.679 0;1.136 -3.146;1.136 0];
C=[1 0 1 -1;0 1 0 0];
sys= ss(A,B,C,zeros(2,2));
step(sys)
FD=bode(sys)
%h=1/(30*bandwidth(sys))
h=0.5
pole(sys)
sysd=c2d(sys,h);
pole(sysd)
controllable=[rank(sysd.a)==rank(ctrb(sysd))]
