clear;
clear;
clc;
cla;
clf;
A = [0.66 0.53;
     -0.53 0.13];
B = [0.34;
      0.53];
C = eye(2);

D = [0
    ;0];

L=[0.36 0.27;
  -0.31 0.08];
 
Ba = [0.34 0;
      0.53 0];
Da = [0 1;
      0 0];

K=[0.0556 0.3306];

abs(eig(A - B*K));
abs(eig(A - L*C));

x = [0;0];
y = C*x;
z = [0;0];
u = 0;

time=12;
plot_vector1=zeros(1,time);
plot_vector2=zeros(1,time);
plot_vector3=zeros(1,time);
plot_vector4=zeros(1,time);
plot_vector5=zeros(1,time);
plot_vector6=zeros(1,time);
plot_vector7=zeros(1,time);
plot_vector8=zeros(1,time);
plot_vector9=zeros(1,time);
plot_vector10=zeros(1,time);
plot_vector11=zeros(1,time);
plot_vector12=zeros(1,time);
plot_vector13=zeros(1,time);
plot_vector14=zeros(1,time);
plot_vector15=zeros(1,time);
time_axis=zeros(1,time);

pattern = ones(1,time);
ak = zeros(2,time);


%using same attack vector of periodic execution on threshold = 0.0249
% ak = [0.054264 0.077519 0.108527 0.077519 0.023256 0.007752;
%       -0.015504 -0.046512 -0.069767 -0.062016 -0.062016 -0.031008];

% %threshold = 0.03, attack len =5, pattern =1010
% ak = [0.077519 0 0.046512 0 -0.031008;
%      -0.023256 0 -0.077519 0 -0.031008];

% threshold = 0.0249, attack len = 6, pattern = 10001000
% ak = [-0.155039 0 0 0 -0.007752 0;
%       0.007752 0 0 0 0.170543 0];

%threshold = 0.03, attack len = 4, pattern =100100
% ak = [0.108527 0 0 0.015504;
%       0 0 0 -0.108527];

%threshold = 0.0249, attack len=6, pattern=10101000
% ak = [0 0 0.046512 0 0.100775 0 0;
%      0 0 0.023256 0 -0.031008 0 -0.085271];

%threshold = 0.0275, attack len=6, pattern=10100
% ak = [-0.046512 0 0.062016 0 0 0.015504;
%       0.023256 0 0.03876 0 0 -0.069767];

%threshold = 0.0275, attack len=5, pattern=10111
% ak = [0 0 -0.062016 -0.077519 -0.015504 0.031008;
%      0 0 0.023256 0.031008 0.046512 0.031008];

 %threshold = 0.0275, attack len=5, pattern=100111
%  ak = [0 0 0 -0.062016 -0.077519 -0.124031 0.015504;
%        0 0 0 0.023256 0.046512 0.085271 0.093023];

%threshold = 0.0275, attack len=5, pattern=10011110
% ak = [0 0 0 0 0.062016 -0.023256 -0.069767 0 -0.015504;
%       0 0 0 0 -0.023256 -0.007752 0.015504 0 0.03876];

delta = 0.0275;
ak = [ak zeros(2, time-size(ak, 2))];
pattern = [1 0 0  1 1 1 1 0 1 0 0  1 1 1 1 0 1 0 0  1 1 1 1 0 ];

for i=1:time
    i
    pattern(i)
    if pattern(i)
       u= -K*z
       u_attacked= u + ak(1,i)

       y= C*x
       y_attacked= y + Da*ak(:,i)

       r= y_attacked - C*z

       z= A*z + B*u + L*r
       x= A*x + B*u_attacked
    else       
       r=[0;0];
       z= A*z + B*u
       x= A*x + B*u_attacked 
       
       y_estimated = C*z;
    end
   
    time_axis(i)=i;
    plot_vector1(i)=0.1;
    plot_vector2(i)=0.05;
    plot_vector3(i)=delta;
    plot_vector4(i)=abs(x(1));
    plot_vector5(i)=abs(x(2));
    plot_vector6(i)= abs(norm(r,inf)); 
end

clf
hold on;
plot(plot_vector1, 'b');
plot(plot_vector2, 'k');
plot(plot_vector3, 'm');
plot(plot_vector4, 'b--');
plot(plot_vector5, 'k--');
plot(plot_vector6, 'm--');

axis([1 time 0 0.12]);
legend({'safe \theta_{k}', 'safe \omega_{k}','\delta','\theta_{k}','\omega_{k}','r_{k}'},'FontSize',10);
% title('Attack on periodic execution');
% legend({'safe \theta_{k}','Th','\theta_{k}','r_{k}'},'FontSize',30);
xlabel('Time(s)','FontSize',10);
% ylabel('\theta_{k}(rad), r_{k}','FontSize',20);
grid on;
hold off;