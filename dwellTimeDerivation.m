clc;
clear all;
ops = sdpsettings('verbose',1)
%%%%%%%%%%%%%%--Given l,epsolon,sampling period--%%%%%%%%%%%%%
exec_pattern='10001001101101011101';
exec_pattern1= strrep(exec_pattern, '1','.1');
exec_pattern2= split(exec_pattern1,'.');
exec_pattern2(1)=[];
exec_pattern_states=unique(exec_pattern2);
l_pattern=length(exec_pattern);
% l=l_pattern;
l=30;
epsilon = 0.364;
exp_decay= (log(1/epsilon))/l             % desired decay rate from (l,epsilon)
stab=1;                                     % while loop first run
i=1;
cnt=6;
horizon=100;
%%%%%%%%%%% plant %%%%%%%%%%%
Ts =1;
A = [0.66 0.53;
   -0.53 0.13];
B = [0.34;    %B1=Bp1
    0.53];
C = eye(2);
%   L=[0.36 0.27;  -0.31 0.08];    
D = [0 ;
    0];    

x0 = [1;1];                             % ini state
x = x0;         
t0 = [0];                               
t = t0;
k=0;
n=0;    
open_loop = ss(A,B,C,D,Ts);
eig_open= eig(A);                   
ZOH_open= open_loop;
[Ap1,Bp1,Cp1,Dp1] = ssdata(ZOH_open);
eig_openZOH=eig(Ap1);   
%    figure(11)
%    step(ZOH_open);
Q_1c_1=10^3*(C'*C)
Q_12c_1=[0;0]
Q_2c_1=10^-4;
R_1c_1=0.005*(B*B');
R_2_1=0.005*eye(2);
p=100;
%K= [0.0556 0.3306];
%A1= (Ap1-Bp1*K);
QXU = blkdiag(Q_1c_1,Q_2c_1);
% QXU=[Q_1c_1 Q_12c_1;Q_12c_1' Q_2c_1];
dimension = size(A,1)
QWU = blkdiag(R_1c_1,R_2_1)
% QWU=[R_1c_1 zeros(dimension,1);zeros(1,dimension) R_2_1]
lqg_reg=lqg(ZOH_open,QXU,QWU);
[Acd,Bcd,Ccd,Dcd]=ssdata(lqg_reg);  %K = -Ccd;

A1=[Ap1 Bp1*Ccd;Bcd*Cp1 Acd]; 
A0=[Ap1 Bp1*Ccd;0.*Bcd*Cp1 eye(size(Acd))]; 
eig_closed= eig(Ap1);
constraints=[];
constraints_di = [];
isCtrb =[];
isStable =[];
slack =0.001;
ctrbl = 1 ;
%%%%%%%%%%%%%%%%%5%%%%--plant with sampling period=m*h --%%%%%%%%%%%%%%%%%%%%%
while ctrbl
    Ts=i*Ts;        
    Am = A1*A0^(i-1);
    eig_state{i} = eig(Am);
    isCtrb(i) = rank(Am)>=rank(ctrb(Am,blkdiag(B,B)));
    ctrbl = isCtrb(i);
    max_eig = max(abs(eig_state{i}));
    isStable(i) = max_eig<1;
    if(isCtrb(i))
        Bm = zeros(2.*size(B));
        Cm = blkdiag(Cp1,zeros(size(Ccd)));
        Dm = zeros(size(Cm,1),size(Bm,2));
        closed_loops{i}=ss(Am,Bm,Cm,Dm,Ts);
        if isStable(i)
            fprintf("\n stable for %dh sampling time",i);
        else
            fprintf("\n unstable for %dh sampling time",i);
        end
        Alpha_di(i) = max_eig;%^2-1;
        Alpha_diBar(i)=1+Alpha_di(i);
%         Pm_di{i} = dlyap(Am,slack);
    else
        fprintf("\n not stabilizable for %dh sampling time",i);
        break;
    end

    %%%--LMi solution to find out Lyapunov func Slow switching--%%%
    Pm_di{i}=sdpvar(size(Am,1),size(Am,1));
    constraint_di=[Am'*Pm_di{i}*Am-(1+Alpha_di(i))*eye(size(Pm_di{i}))*Pm_di{i} <= slack,...
                            -Pm_di{i} <= slack];

    optimize(constraint_di,[],ops);
    P_di{i}=value(Pm_di{i});           %%--P values for discrete sys     
    i=i+1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% p=1;q=1;
for i=1:size(closed_loops,2)
    mum_di{i}=sdpvar(1,1);
    for j=1:size(closed_loops,2)
%        constraints=[constraints, P{i}-mum{i}*eye(size(P{j}))*P{j}<= 0,mum{i}>1];
        constraints_di=[constraints_di, P_di{i}-mum_di{i}*eye(size(P_di{j}))*P_di{j}<= 0,mum_di{i}-1>=slack];
%         mu(j)=(det(P{i}/P{j})^(1/size(P{i},1)));        
   end
end
%%%%%%%%%%%%--Solving LMI to find out Mu--%%%%%%%%%%%%%%
% optimize(constraints,[],ops);
optimize(constraints_di,[],ops);
p=1;q=1;
gammaUnstable=[];
%%%%%%%%%%%--Gamma,Taud calculation--%%%%%%%%%%%%%
for i=1:size(closed_loops,2)
        Mu(i)=double(mum_di{i});
%         Taud(i)= -(log(Mu(i))/log(Alpha_diBar(i)));      %%%%-dwell time for ith state
        Taud(i)= log(Mu(i)/abs(log(Alpha_diBar(i))));      %%%%-dwell time for ith state
        Count(i)= (ceil(Taud(i)/closed_loops{i}.Ts));     %%%--Permissible self loop count
        gamma(i)= (Alpha_diBar(i)*((Mu(i))^(1/Taud(i))));
%        gamma(i)= Alpha_di(i)+(log(Mu(i))/Taud(i))

        if(isStable(i))
           gammaStable(p)= gamma(i);      %%--gamma_plus
 %          gammaStable(p)= (gamma_eig(i));
            p=p+1;
        else
           gammaUnstable(q)=gamma(i);    %%%--gamma_minus
 %          gammaUnstable(q)= (gamma_eig(i));
            q=q+1;
        end
end  % end for
%%%%%%%%%%%%%%--Values derived--%%%%%%%%%%%%%%%
if(~isempty(gammaUnstable))
    DwellingRatio= ceil((log(max(gammaUnstable))+(exp_decay))/(-log(max(gammaStable))+(exp_decay)));
    % DwellingRatio= ceil((((max(gammaUnstable)))+(exp_decay))/(((max(gammaStable))))-(exp_decay));
    % T_minus (dwelling time in stable sys)/T_plus (dwelling time in unstable sys)
end
% %%%%%%%%%%%%%%%%%%%%%%%%--drop rate calculation--%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% beta_1 = max(abs(eig(A1)))^2
% beta_0 = max(abs(eig(Ap1)))^2
%  
% alpha=exp_decay;
%     
% r_min=(2*log(alpha)+log(beta_0))/(log(beta_0)-log(beta_1));
%     
%     %%========= lmi variables are r and P ===========%%
% 
%     setlmis([]);
%     r=lmivar(1,[1,0]);
%     Pr=lmivar(1,[size(Am,1),1]);
% 
%     lmiterm([-1 1 1 r],1,1);                    % LMI # 1: r > r_min
%     lmiterm([1 1 1 0],r_min);                      
% 
%     lmiterm([2 1 1 Pr],A1',A1);                 % LMI # 2 : A1'*Pr*A1 <= beta_1*Pr
%     lmiterm([2 1 1 Pr],-beta_1,1);        
% 
%     lmiterm([3 1 1 Pr],Ap1',Ap1);               % LMI # 3 : Ap1'*Pr*Ap1 <= beta_0*Pr
%     lmiterm([3 1 1 Pr],-beta_0,1);        
% 
%     lmiterm([4 1 1 r],1,1);                     % LMI # 4 : r < 1
%     lmiterm([4 1 1 0],-1);   
% 
%     lmiterm([-5 1 1 r],1,1);                    % LMI # 5 : r > 0
% 
%     lmiterm([-6 1 1 Pr],1,1);                   % LMI # 6  Pr > 0    
% 
%     lmisys = getlmis;                           % Create the LMI system
% 
%     [tmin,rP_feas] = feasp(lmisys) ;
%     Pmat = dec2mat(lmisys,rP_feas,Pr);             % display Pr matrix
%     r = dec2mat(lmisys,rP_feas,r);               % display r
%     
%     decay=epsilon;
%     rate=r;
%     alpha=log(1/epsilon)/l;
% 
% %%%%%%%%%%%%%%%-- application --%%%%%%%%%%%%%%%%%%%%
% %rt=0.56;
% V=[x0'*P{count(exec_pattern2(1),'0')+1}*x0];                   %%%--V for initial state
% for k=1:floor(horizon/l)
% for i=1:size(exec_pattern2)
%    if(count(exec_pattern2(i),'1')==1)
%        if((size(exec_pattern2,1)/length(exec_pattern)) >= rate)
%          m=count(exec_pattern2(i),'0')+1;
%          Ts=sampling_time(m);
%          per_step=Ts/1;
%          time_horizon=Ts;           %%--time limit for this pattern
%          [ym,tm,xm]=initial(closed_loops{m},x0,time_horizon);%%--running till time, based on pattern
%          x=[x'; xm]';                                        %%--appending x values for m X h
%          t=[t';tm+t0]';                                      %%--appending t values for m X h
%          n=[n'; (Ts/h)*ones(size(tm))]';                     %%--appending switching signal
% %         y=[y;ym];
%          for j=1:size(tm,1)
%             Vm= [xm(:,j)'*P{m}*xm(:,j)]';            
%             V=[V Vm];
%          end
%          x0=x(:,size(t,2))';                    %%--last value of x as initial value for next
%          t0=t(1,size(t,2));                     %%--last timestamp as initial time for next
%          shouldPlot=1;
%        else           
%            shouldPlot=0;
%        end
%    end
% end   
% end
% 
% %%%%%%%%%--plot--%%%%%%%%%%
%    if(shouldPlot==1)
%      figure(1);
%      [AX,H1,H2] = plotyy(t,x(1,:),t,n,'plot');              %%% state vs time on switching modes
%      set(get(AX(1),'Ylabel'),'String','x');
%      set(get(AX(2),'Ylabel'),'String','x_dot');
%      title(sprintf('State vs time for pattern: %s',exec_pattern));
%      grid on;   
%      figure(2);
%      [AX_dot,H1_dot,H2_dot] = plotyy(t,x(2,:),t,n,'plot');              %%% state vs time on switching modes
%      set(get(AX_dot(1),'Ylabel'),'String','x_{dot}');
%      set(get(AX_dot(2),'Ylabel'),'String','n');     
%      title(sprintf('x_{dot},n vs time for pattern: %s',exec_pattern));
%      grid on; 
%      figure(3);
%      plot(t,(V));
%      grid on;
%     end
% %%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:size(closed_loops,2)
    Sampling_Time(i)=closed_loops{i}.Ts;
end
Self_Loop_Count=Count';
Min_Dwell_Time=Taud';
Mu_Val=Mu';
Alpha_val=Alpha_di';
% Gamma=log(gamma_eig');
Gamma=log(gamma');
T=table(Sampling_Time',Self_Loop_Count,Min_Dwell_Time,Mu_Val,Alpha_val,Gamma)

%DwellingRatio
%rate
exp_decay