clc;
clf;
clear;
n=5;
v=logspace(log10(0.75),log10(32),n)
v(4)=8;
v(3)=5;
v(2)=2;

 Ts=0.01;
 A1c=zeros(2,2,n);
 A2c=zeros(2,2,n);
 A1=zeros(2,2,n);
 A2=zeros(2,2,n);
 B1c=zeros(2,1,n);
 B1=zeros(2,1,n);
 B2c=zeros(2,1,n);
 B2=zeros(2,1,n);
 L1=zeros(2,1,n);
 L2=zeros(2,1,n);
 alpha=9.75;
 beta=0.1;
 Fn=4414;
 m=450;
 R=0.32;
 J=1;
 for i=1:n
     A1c(:,:,i)=[0 (-1*alpha*Fn)/m;
         (alpha*Fn*R*R)/(v(i)*v(i)*J) (-1*alpha*Fn*R*R)/(v(i)*J)];
     A2c(:,:,i)=[0 Fn/(4*m);
         ((-1/4+3/4-0.2)*Fn*R*R)/(v(i)*v(i)*J) (Fn*R*R)/(4*v(i)*J)];
     B1c(:,:,i)=[0;
                (-1*alpha*Fn*R*R)/(v(i)*v(i)*J)+(R*1200)/(J*v(i)*v(i))];
     B2c(:,:,i)=[((-3/4-0.2)*Fn)/m;
                ((1/4-3/2-0.4)*Fn*R*R)/(v(i)*J)];
 end
     
 Bc=[0;
    R/J];
 Cc=[1 0];
 Dc=0;

QN = 0.0025;
RN = 100;

 for i=1:n
    sys=ss(A1c(:,:,i),B1c(:,:,i),Cc,Dc);
    sys_d = c2d(sys,Ts,'zoh');
    A1(:,:,i) = sys_d.a;
    B1(:,:,i) = sys_d.b;
    C = sys_d.c;
    D = sys_d.d;
    G=B1(:,:,i);
    H=0;
    syse = ss(A1(:,:,i),[B1(:,:,i) G],C,[D H]);
    syse_d = c2d(syse,Ts,'zoh');
    [Kest,L1(:,:,i),P]=kalman(syse_d,QN,RN,'delayed');
    %L1(:,:,i) = place(A1c(:,:,i)',Cc',[-100 -101])'*(1.0e-4)
    sys=ss(A2c(:,:,i),B2c(:,:,i),Cc,Dc);
    sys_d = c2d(sys,Ts,'zoh');
    A2(:,:,i) = sys_d.a;
    B2(:,:,i) = sys_d.b;
    C = sys_d.c;
    D = sys_d.d;
    G=B2(:,:,i);
    H=0;
    syse = ss(A2(:,:,i),[B2(:,:,i) G],C,[D H]);
    syse_d = c2d(syse,Ts,'zoh');
    [Kest,L2(:,:,i),P]=kalman(syse_d,QN,RN,'delayed');
    %L2(:,:,i) = place(A2c(:,:,i)',Cc',[-100 -101])'*(1.0e-4)
 end
 
QN = 1;
RN = 1;

P = 1.0e-02 * Cc'*Cc;
for i=1:1000
    L1(:,:,1)=P*Cc'*inv(Cc*P*Cc'+RN);
    P=(eye(2)-L1(:,:,1)*Cc)*P;
    P=(A1(:,:,1)*P*A1(:,:,1)'+QN);
end

P = 1.0e-02 * Cc'*Cc;
for i=1:1000
    L2(:,:,1)=P*Cc'*inv(Cc*P*Cc'+RN);
    P=(eye(2)-L2(:,:,1)*Cc)*P;
    P=(A2(:,:,1)*P*A2(:,:,1)'+QN);
end

P = 1.0e-02 * Cc'*Cc;
for i=1:1000
    L1(:,:,2)=P*Cc'*inv(Cc*P*Cc'+RN);
    P=(eye(2)-L1(:,:,2)*Cc)*P;
    P=(A1(:,:,2)*P*A1(:,:,2)'+QN);
end

P = 1.0e-02 * Cc'*Cc;
for i=1:1000
    L2(:,:,2)=P*Cc'*inv(Cc*P*Cc'+RN);
    P=(eye(2)-L2(:,:,2)*Cc)*P;
    P=(A2(:,:,2)*P*A2(:,:,2)'+QN);
end

P = 1.0e-02 * Cc'*Cc;
for i=1:1000
    L1(:,:,3)=P*Cc'*inv(Cc*P*Cc'+RN);
    P=(eye(2)-L1(:,:,3)*Cc)*P;
    P=(A1(:,:,3)*P*A1(:,:,3)'+QN);
end

P = 1.0e-02 * Cc'*Cc;
for i=1:1000
    L2(:,:,3)=P*Cc'*inv(Cc*P*Cc'+RN);
    P=(eye(2)-L2(:,:,3)*Cc)*P;
    P=(A2(:,:,3)*P*A2(:,:,3)'+QN);
end

P = 1.0e-02 * Cc'*Cc;
for i=1:1000
    L1(:,:,4)=P*Cc'*inv(Cc*P*Cc'+RN);
    P=(eye(2)-L1(:,:,4)*Cc)*P;
    P=(A1(:,:,4)*P*A1(:,:,4)'+QN);
end

P = 1.0e-02 * Cc'*Cc;
for i=1:1000
    L2(:,:,4)=P*Cc'*inv(Cc*P*Cc'+RN);
    P=(eye(2)-L2(:,:,4)*Cc)*P;
    P=(A2(:,:,4)*P*A2(:,:,4)'+QN);
end

P = 1.0e-02 * Cc'*Cc;
for i=1:1000
    L1(:,:,5)=P*Cc'*inv(Cc*P*Cc'+RN);
    P=(eye(2)-L1(:,:,5)*Cc)*P;
    P=(A1(:,:,5)*P*A1(:,:,5)'+QN);
end

P = 1.0e-02 * Cc'*Cc;
for i=1:1000
    L2(:,:,5)=P*Cc'*inv(Cc*P*Cc'+RN);
    P=(eye(2)-L2(:,:,5)*Cc)*P;
    P=(A2(:,:,5)*P*A2(:,:,5)'+QN);
end
 
 A1
 B1
 A2
 B2
 
 total_running_time=30;
 total_samples=ceil(total_running_time/Ts);
 plot_vector=zeros(1,total_samples+1);
 plot_vector0=zeros(1,total_samples+1);
 plot_vector1=zeros(1,total_samples+1);
 plot_vector2=zeros(1,total_samples+1);
 plot_vector3=zeros(1,total_samples+1);
 plot_vector4=zeros(1,total_samples+1);
 plot_vector5=zeros(1,total_samples+1);
 plot_vector6=zeros(1,total_samples+1);
 time_axis=zeros(1,total_samples+1);
 
 stop_dist=0;
 x=[20;0];
 time_axis(1)=0;
 plot_vector(1)=x(1);
 
 % No attack
 for i=1:total_samples
     for j=1:n
         if (x(1)>v(n-j+1))
             break;
         end
     end
     j=n-j+1;
     stop_dist=stop_dist+x(1)*Ts;
     if(x(2)<=0.2)
        u=1*x(1); 
        x=(A1(:,:,j)*x+B1(:,:,j)*u);
     else
        x=(A2(:,:,j)*x+B2(:,:,j));
     end
     time_axis(i+1)=i;
     plot_vector(i+1)=x(1);
 end
 stop_dist
 
 %Threshold = 0.15
 lambda=5;
 g = [0.01;
    -0.01];
 kappa = 1;
 ak = kappa*lambda*g;
 stop_dist=0;
 x=[20;0];
 
 time_axis(1)=0;
 plot_vector1(1)=x(1);
  
 xhat=[20;0];
 plot_vector4(1)= abs(x(1)-xhat(1));
 for i=1:total_samples
     kappa=ceil((3*i)/10);
     ak = kappa*lambda*g;
     
     for j=1:n
         if ((x(1)+ak(1))>v(n-j+1))
             break;
         end
     end
     j=n-j+1;
    if(x(1)>1)
        stop_dist=stop_dist+x(1)*Ts;
     end
     if(x(2)<=0.2)
        y=x(1)+ak(1);
        u=1*y; 
        x=(A1(:,:,j)*x+B1(:,:,j)*u);
        xhat=(A1(:,:,j)*xhat+B1(:,:,j)*u)+L1(:,:,j)*(x(1)-xhat(1));
        if (abs(x(1)-xhat(1))>0.15)
            i;
            detected=1
        end
     else
        x=(A2(:,:,j)*x+B2(:,:,j));
        xhat=(A2(:,:,j)*xhat+B2(:,:,j))+L2(:,:,j)*(x(1)-xhat(1));
     end
     time_axis(i+1)=i;
     plot_vector1(i+1)=x(1);
     plot_vector4(i+1)= abs(x(1)-xhat(1));
 end
 hold on
stop_dist

%Threshold = 0.2
lambda=5;
 g = [0.01;
    -0.01];
 kappa = 1;
 ak = kappa*lambda*g;
 stop_dist=0;

 x=[20;0];
 
 time_axis(1)=0;
 plot_vector2(1)=x(1);
 
 xhat=[20;0];
 for i=1:total_samples
     kappa=ceil((4*i)/10);
     ak = kappa*lambda*g;
        
     
     for j=1:n
         if ((x(1)+ak(1))>v(n-j+1))
             break;
         end
     end
     j=n-j+1;
     if(x(1)>1)
        stop_dist=stop_dist+x(1)*Ts;
     end
     if(x(2)<=0.2)
        y=x(1)+ak(1);
        u=1*y; 
        x=(A1(:,:,j)*x+B1(:,:,j)*u);
        xhat=(A1(:,:,j)*xhat+B1(:,:,j)*u)+L1(:,:,j)*(x(1)-xhat(1));
        if (abs(x(1)-xhat(1))>0.2)
            i;
            detected=1
        end
     else
        x=(A2(:,:,j)*x+B2(:,:,j));
        xhat=(A2(:,:,j)*xhat+B2(:,:,j))+L2(:,:,j)*(x(1)-xhat(1));
     end
     time_axis(i+1)=i;
     plot_vector2(i+1)=x(1);
     plot_vector5(i+1)= abs(x(1)-xhat(1));
 end
stop_dist

%Threshold = 0.3
lambda=5;
 g = [0.01;
    -0.01];
 kappa = 1;
 ak = kappa*lambda*g;
 stop_dist=0;

 x=[20;0];
 
 time_axis(1)=0;
 plot_vector3(1)=x(1);
 
 xhat=[20;0];
 for i=1:total_samples
     kappa=ceil((6*i)/10);
     ak = kappa*lambda*g;
     
     for j=1:n
         if ((x(1)+ak(1))>v(n-j+1))
             break;
         end
     end
     j=n-j+1;
     if(x(1)>1)
        stop_dist=stop_dist+x(1)*Ts;
     end
     if(x(2)<=0.2)
        y=x(1)+ak(1);
        u=1*y; 
        x=(A1(:,:,j)*x+B1(:,:,j)*u);
        xhat=(A1(:,:,j)*xhat+B1(:,:,j)*u)+L1(:,:,j)*(x(1)-xhat(1));
        if (abs(x(1)-xhat(1))>.3)
            i;
            detected=1
        end
     else
        x=(A2(:,:,j)*x+B2(:,:,j));
        xhat=(A2(:,:,j)*xhat+B2(:,:,j))+L2(:,:,j)*(x(1)-xhat(1));
     end
     time_axis(i+1)=i;
     plot_vector3(i+1)=x(1);
     plot_vector6(i+1)= abs(x(1)-xhat(1));
 end
stop_dist

%Attack detected by PUF
lambda=5;
 g = [0.01;
    -0.01];
 kappa = 1;
 ak = kappa*lambda*g;
 stop_dist=0;

 x=[20;0];
 
 time_axis(1)=0;
 plot_vector0(1)=x(1);
  
 xhat=[20;0];

 for i=1:total_samples
     kappa=ceil((6*i)/10);
     ak = kappa*lambda*g;
     
     if(i>60)
         ak(1)=0;
     end
     
     for j=1:n
         if ((x(1)+ak(1))>v(n-j+1))
             break;
         end
     end
     j=n-j+1;
     if(x(1)>1)
        stop_dist=stop_dist+x(1)*Ts;
     end
     if(x(2)<=0.2)
        y=x(1)+ak(1);
        u=1*y; 
        x=(A1(:,:,j)*x+B1(:,:,j)*u);
        xhat=(A1(:,:,j)*xhat+B1(:,:,j)*u)+L1(:,:,j)*(x(1)-xhat(1));
        if (abs(x(1)-xhat(1))>0.3)
            i;
            detected=1
        end
     else
        x=(A2(:,:,j)*x+B2(:,:,j));
        xhat=(A2(:,:,j)*xhat+B2(:,:,j))+L2(:,:,j)*(x(1)-xhat(1));
     end
     time_axis(i+1)=i;
     plot_vector0(i+1)=x(1);
 end
stop_dist

hold on
plot(time_axis,plot_vector,time_axis,plot_vector1,time_axis,plot_vector2,time_axis,plot_vector3),xlabel('Time(x10ms)'),ylabel('Speed(m/s)'),legend('No attack','Threshold=0.15','Threshold=0.2','Threshold=0.3','Location','northeast');
hold off
figure
hold on
plot(time_axis,plot_vector,time_axis,plot_vector3,time_axis,plot_vector0),xlabel('Time(x10ms)'),ylabel('Speed(m/s)'),legend('No attack','Threshold=0.3','Bounded attack of length = 600 ms','Location','northeast');
hold off
figure
hold on
 plot(time_axis,plot_vector4,time_axis,plot_vector5,time_axis,plot_vector6),xlabel('Time(x10ms)'),ylabel('Speed(m/s)'),legend('Residue signal when threshold = 0.15','Residue signal when threshold = 0.2','Residue signal when threshold = 0.3','Location','northeast');
hold off