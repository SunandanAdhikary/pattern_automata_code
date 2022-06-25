clc;
clear all;
ops = sdpsettings('verbose',0);
system = "fuel_injection"
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
exp_decay= (log(1/epsilon))/l;  % desired decay rate from (l,epsilon)
exp_decay_dt = exp(-exp_decay);%epsilon^(1/l);  % desired decay rate in discrete domain
stab=1;                        % while loop first run
i=1;
cnt=6;
horizon=100;
%%%%%%%%%%% plant %%%%%%%%%%%
if system=="plant"
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
    sys_ss = ss(A,B,C,D,Ts);
    eig_open= eig(A);                   
    eig_openZOH=eig(Ap1);   
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
    QWV = blkdiag(R_1c_1,R_2_1)
    % QWV=[R_1c_1 zeros(dimension,1);zeros(1,dimension) R_2_1]
end
% fuel_injection(Modeling and Control of an Engine Fuel Injection System)--in ECM ECU
if system=="fuel_injection"
    % states: 
    % inputs: inlet valve air flow, combustion torque
    % output: AFR
    Ts = 0.1;
    A = [0.18734,0.13306,0.10468;
        0.08183,0.78614,-0.54529;
        -0.00054 0.10877,0.26882];
    B = [0.00516,-0.0172;
        -0.00073,0.09841;
        -0.00011,0.13589];
    C = [158.16,8.4277,-0.44246];
    D = [0,0];
    open_loop = ss(A,B,C,D,Ts);
    Q = blkdiag(0.1,0.1,1);%blkdiag(1,2.25,25);
    R = blkdiag(1,500);
    Pr = care(A,B,Q);
    K = -inv(R)*B'*Pr;
    qweight= 1;
    rweight = 1;
    proc_dev= 0.001; meas_dev=0.0001;
    QN = eye(size(B,1));%proc_dev^2*(B*B');
    RN = 1;%meas_dev^2;
    L=[0.00150311177912917;
        0.00200806159683866;
        0.000355570756765841];
    safex = [-0.22,-1.5,-5;0.22,1.5,5].*3;
    % initial region of this system to safely start from
    ini = 0.7.*safex;
    perf = 0.1.*safex;
    % for central chi2 FAR < 0.05
    sensor_limit = 80;  % columnwise range of each y
    actuator_limit = [20;20];   % columnwise range of each u
    threshold = 4.35;
    uatkon=[1];   % attack on which u
    yatkon=[1];   % attack on which y
    rate =0.5;
    QXU = blkdiag(Q,R);
    QWV = blkdiag(QN,RN);
end
if system=="trajectory"
    Ts=0.1;
    A = [1.0000    0.1000; 0    1.0000];
    B = [0.0050; 0.1000];
    C = [1 0];
    D = [0];
    open_loop = ss(A,B,C,D,Ts);
    Q= eye(size(A,2));
    R= eye(size(B,2));
    [K,S,E] = dlqr(A,B,Q,R);
    QN = 1500;
    RN = eye(1);
    sys_ss = ss(A,B,C,D,Ts);
    [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    safex = [-25,-30;25,30];
    safer = 0.6;
    sensor_limit = 30;
    actuator_limit = 36; %36;
    perf_region_depth = 0.3;
    perf = perf_region_depth.*safex;
    threshold = 4.35;
    t=20;
    rate = 0.5;
    QXU = blkdiag(Q,R);
    QWV = blkdiag(QN,RN,1); 
end
ZOH_open= open_loop;
[Ap1,Bp1,Cp1,Dp1] = ssdata(ZOH_open);
lqg_reg=lqg(ZOH_open,QXU,QWV);
[Acd,Bcd,Ccd,Dcd]=ssdata(lqg_reg);  %K = -Ccd;


A1=[Ap1 Bp1*Ccd;Bcd*Cp1 Acd]; 
A0=[Ap1 Bp1*Ccd;0.*Bcd*Cp1 eye(size(Acd))]; 
eig_closed= eig(Ap1);
constraints= [];
constraints_di = [];
isCtrb = [];
isStable =[];
slack = 0.001;
ctrbl = 1 ;
unstable_count = 0;
h = Ts;
%%%%%%%%%%%%%%%%%5%%%%--plant with sampling period=m*h --%%%%%%%%%%%%%%%%%%%%%
while ctrbl
    Ts=i*h;        
    Am = A1*A0^(i-1);
    eig_state{i} = eig(Am);
    isCtrb(i) = rank(Am)>=rank(ctrb(Am,blkdiag(B,B)));
    ctrbl = isCtrb(i);
    max_eig = max(abs(eig_state{i}));
    isStable(i) = max_eig<1;
    howStabler(i) = max_eig/exp_decay_dt;
    isStabler(i) = howStabler(i) >= 1;
%     Alpha_di(i) = -0.4;
%     Alpha_diBar(i)=1+Alpha_di(i);
    alpha(i) = -0.893;%sdpvar(1,1);
%     assign(alpha(i),0.9);
    if(isCtrb(i))
        Bm = zeros(2.*size(B));
        Cm = blkdiag(Cp1,zeros(size(Ccd)));
        Dm = zeros(size(Cm,1),size(Bm,2));
        closed_loops{i}=ss(Am,Bm,Cm,Dm,Ts);
        if isStable(i)
            fprintf("\n stable for %dh sampling time\n",i);
            if isStabler(i)
                fprintf("\n %f times stabler than desired for %dh sampling time\n",howStabler(i),i);
            end
        else
            fprintf("\n unstable for %dh sampling time",i);
            unstable_count = unstable_count + 1;
        end
%         Pm_di{i} = dlyap(Am,slack);
        if unstable_count > 5
            i = i-1;
            break;
        end
    else
        fprintf("\n not stabilizable for %dh sampling time",i);
        if unstable_count > 5
            i = i-1;
            break;
        end
    end
    %%%--LMi solution to find out Lyapunov func Slow switching--%%%
    solved = 1;
    min = 1;
    while solved ~= 0 && alpha(i) < -slack && -1 < alpha(i)
%     while ~min
        fprintf("solving for alpha = "+num2str(alpha(i))+ " for the system with "+num2str(Ts)+ " sampling period\n")
        Pm_di{i}=sdpvar(size(Am,1),size(Am,1));
        constraint_di=[Am'*Pm_di{i}*Am-(1+alpha(i))*eye(size(Pm_di{i}))*Pm_di{i} <= slack,...
                       Pm_di{i} >= slack];
    %     constraint_di=[Am'*Pm_di{i}*Am-(1+Alpha_di(i))*eye(size(Pm_di{i}))*Pm_di{i} <= slack,...
    %                             -Pm_di{i} <= slack];
    %     constraint_di=[Am'*Pm_di{i}*Am-Pm_di{i}+blkdiag(Q,Q) <= slack,...
    %                            -Pm_di{i} <= slack];
        sol=optimize(constraint_di,[],ops);
        solved = sol.problem
        alpha(i) = alpha(i) + 0.01;
    end
    P_di{i}=value(Pm_di{i});           %%--P values for discrete sys  
    Alpha_di(i) =value(alpha(i));
%     Alpha_di(i) = -min(eig(blkdiag(Q,Q)))/max(eig(P_di{i}));% Q+K'*B'*A+A'*B*K
    Alpha_diBar(i)=1+Alpha_di(i);
    i=i+1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% p=1;q=1;
for i=1:size(P_di,2)
    mum_di{i}=sdpvar(1,1);
    for j=1:size(P_di,2)
%        constraints=[constraints, P{i}-mum{i}*eye(size(P{j}))*P{j}<= 0,mum{i}>1];
        constraints_di=[constraints_di,...
                        P_di{i}-mum_di{i}*eye(size(P_di{j}))*P_di{j}<= 0,...
                                                            mum_di{i}-1>=slack];
%         mu(j)=(det(P{i}/P{j})^(1/size(P{i},1)));        
   end
end
%%%%%%%%%%%%--Solving LMI to find out Mu--%%%%%%%%%%%%%%
fprintf("mu derivation by cs...")
check(constraints_di);
sol = optimize(constraints_di,[],ops);
solved = sol.problem
p=1;q=1;
gammaUnstable=[];
%%%%%%%%%%%--Gamma,Taud calculation--%%%%%%%%%%%%%
for i=1:size(closed_loops,2)
        Mu(i)=double(mum_di{i});
        Taud(i)= -log(Mu(i))/log(Alpha_diBar(i));      %%%%-dwell time for ith state
%         Taud(i)= log(Mu(i))/abs(log(Alpha_diBar(i)));
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
nn =size(closed_loops,2);
for i=1:nn
    Sampling_Time(i)=closed_loops{i}.Ts;
end
Self_Loop_Count=Count';
Min_Dwell_Time=Taud';
Mu_Val=Mu(:,1:nn)';
Alpha_val=Alpha_di(:,1:nn)';
% Gamma=log(gamma_eig');
Gamma=log(gamma(:,1:nn)');
T=table(Sampling_Time',Self_Loop_Count,Min_Dwell_Time,Mu_Val,Alpha_val,Gamma)
exp_decay_dt
exp_decay
DwellingRatio
%rate
