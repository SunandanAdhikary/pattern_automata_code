clc;
clear all;
diary dbl_int.out;
%%%%%%%%%%%%%%--Given l,epsolon,sampling period--%%%%%%%%%%%%%
exec_pattern='1001001';
exec_pattern1= strrep(exec_pattern, '1','.1');
exec_pattern2= split(exec_pattern1,'.');
exec_pattern2(1)=[];
exec_pattern_states=unique(exec_pattern2);
l_pattern=length(exec_pattern);
l=100;
l=l_pattern;
unit_decay = 0.005; 
epsilon = 0.005;
exp_decay= (log(1/epsilon))/l;              % desired decay rate from (l,epsilon)
stab=1;                                     % while loop first run
i=1;
cnt=15;

gammaStable =zeros(size(exec_pattern_states));
gammaUnstable =zeros(size(exec_pattern_states));

    %%%%%%%%%%% plant %%%%%%%%%%%
    A = [0 1;
      0 0];
    B = [0;
      1];
    C = [1 0];
    D = 0;
    x0 = [1;1];                         % ini state
    x = x0;         
    t0 = [0];                               
    t = t0;
    k=0;
    n=0;
    h = 0.1;                              % sampling rate 
    states=['x','x_dot'];
    input=['u'];
    output=['y'];
    reference=['r'];
    open_loop = ss(A,B,C,D);
    eig_open= eig(A);                   
    Ts=h;
    ZOH_open= c2d(open_loop,Ts,'zoh');
    [Ap1,Bp1,Cp1,Dp1] = ssdata(ZOH_open);
    eig_openZOH=eig(Ap1);               
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%---closed loop---%%%%%%%%%%%%%%%%%%
   
    p=100;
    K=dlqr(Ap1,Bp1,p*(Cp1'*Cp1),[1]);
    K=[132 16];
    A1= (Ap1-Bp1*K);
    B1=zeros(size(A1,1),1);             %%%--Becomes zero
    C1=Cp1;                             %%%--remains same,outputs
    D1=Dp1;                             %%%--remains same i.e. zero
    closed_loop = ss(A1,B1,C1,D1,Ts);
    eig_closed= eig(A1);
    %%%--LMi solution to find out Lyapunov func--%%%
        
 
%%%%%%%%%%%%%%%%%5%%%%--plant with sampling period=m*h --%%%%%%%%%%%%%%%%%%%%%
    while stab==1
        m=i;
        Ts=m*h;        
        per_step=Ts/10;
%        time_horizon=30;
        ZOHm= c2d(open_loop,Ts,'zoh');
        [Apm,Bpm,Cpm,Dpm] = ssdata(ZOHm);

        Am= (Apm-Bpm*K);
        Bm=zeros(size(Am,1),1);             %%%--Becomes zero
        Cm=Cpm;                             %%%--remains same,outputs
        Dm=Dpm;                             %%%--remains same i.e. zero
        closed_loop = ss(Am,Bm,Cm,Dm,Ts);
        eig_state{i}=eig(Am);
        %%%%%%%%%%%%%%%%%--Contrallability check--%%%%%%%%%%%%%%%%%%
         Km=place(Apm,Bpm,eig_closed);       %%--if the controller was designed for mh sampling time        
%        Km=lqr(Apm,Bpm,Cpm*Cpm',[1]);
        closed_m_loop=ss(Apm-Bpm*Km,Bpm,Cpm,Dpm,Ts);  %%--would there be any K possible for stability?
        if(((abs(eig(Apm-Bpm*Km))<1)) & cnt>0)
            ifStabilizable(i)=1;                                %%--stabilizable frequency?
            stab=1;
%           fprintf("\n stabilizable for %dh sampling time",m);
            closed_loops{i}=closed_loop;
            w=step(closed_loops{i});
            ws=stepinfo(w)
            set_time(i)=ws.SettlingTime;
            
            closed_m_loops{i}=closed_m_loop;
%            figure(m)
%           step(closed_m_loops{i});
            w=step(closed_m_loops{i});
            ws=stepinfo(w);
            set_m_time(i)=ws.SettlingTime;
            cnt=cnt-1;
        else
            stab=0;
            fprintf("\n not stabilizable for %dh sampling time",m);
            eig_state{i}=[];
            break;
        end
         
        %%%--deciding alpha_m for the state--%%%
        if((abs(eig_state{i})<1))
            ifStable(i)= 1;
            alpham=-2*eye(size(Am));
            fprintf("\n stable for %dh sampling time",m);
            Alpha(i)=-1*det(alpham)^(1/size(alpham,1));        %%--alpha values
        else
            ifStable(i)=0;
            alpham=2*eye(size(Am));
            fprintf("\n not stable for %dh sampling time",m);
            Alpha(i)=det(alpham)^(1/size(alpham,1));           %%--alpha values
        end
        
        %%%--LMi solution to find out Lyapunov func--%%%
        Pm=sdpvar(size(A1,1),size(A1,1));
        Probm=[Am'*Pm+Pm'*Am-alpham*Pm zeros(size(Am));
                zeros(size(Am)) -Pm];   
        constraint= [Probm <= 0];

        solvesdp(constraint);
        Pm= double(Pm);
        P{i}=Pm;                                                %%--P values
        sampling_time(i)=Ts;            
        i=i+1;
    
%          %%%%%%%%--plot--%%%%%%%%%%
%          figure(i);
%          [y,t,x]= initial(closed_loop,x0);
%          [AX,H1,H2] = plotyy(t,y(:,1),t,y(:,2),'plot');       %%%--state vs time on switching modes
%          set(get(AX(1),'Ylabel'),'String','theta');
%          set(get(AX(2),'Ylabel'),'String','theta dot');
%          title(sprintf('State vs time for %d x sampling time: %s',m));
%          grid on;   
%          %%%%%%%%%%%%%%%%%%%%%%%%%%%
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p=1;q=1;
%%%%%%%%%%%--Mu,Taud calculation--%%%%%%%%%%%%%
        
for i=1:size(sampling_time,2)
    for j=1:size(sampling_time,2)
        mu(j)=(det(P{i}/P{j})^(1/size(P{i},1)));
%         mu(j)=det(P{i}*inv(P{j}));
    end
        
%        constraints= Probm <= 0;
%        solvesdp(constraint);
%        Pm= double(Pm);
     mu
    Mu(i)=(max(mu)*2);                                          %%%--MU_i for ith state
    Taud(i)= (log(Mu(i))/Alpha(i));                      %%%%-dwell time for ith state
    Count(i)= abs(floor(Taud(i)/sampling_time(i)));         %%%--Permissible self loop count
    gamma(i)= abs(Alpha(i)+(log(Mu(i)))/Taud(i));
    if(ifStable(i))
%        gammaStable(i)= Alpha(i)+(log(Mu(i)))/Taud(i);      %%--gamma_plus
        gammaStable(p)= abs(gamma(i));      %%--gamma_plus
        p=p+1;
    else
 %       gammaUnstable(i)= Alpha(i)+(log(Mu(i)))/Taud(i);    %%%--gamma_minus
        gammaUnstable(q)=abs(gamma(i));    %%%--gamma_minus
        q=q+1;
    end
end

%%%%%%%%%%%%%%--Values derived--%%%%%%%%%%%%%%%
DwellingRatio= ceil((abs(max(gammaUnstable))+exp_decay)/(abs(max(gammaStable))-exp_decay));
% T_minus (dwelling time in stable sys)/T_plus (dwelling time in unstable sys)

%%%%%%%%%%%%%%%%%%%%%%%%--drop rate calculation--%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
beta_1 = max(abs(eig(A1)))^2
beta_0 = max(abs(eig(Ap1)))^2
 
total_points=(1/unit_decay)-1;
decay=zeros(1,total_points);
rate=zeros(1,total_points);

% alpha=(1-epsilon)^(1/l1);  % damping rate- smaller epsilon -> higher alpha
alpha=exp_decay;

    
    r_min=(2*log(alpha)+log(beta_0))/(log(beta_0)-log(beta_1));
    % r_min=(log(beta_0))/(log(beta_0)-log(beta_1))

    %%========= lmi variables are r and P ===========%%

    setlmis([]);
    r=lmivar(1,[1,0]);
    Pr=lmivar(1,[size(Am,1),1]);

    lmiterm([-1 1 1 r],1,1);                    % LMI # 1: r > r_min
    lmiterm([1 1 1 0],r_min);                      

    lmiterm([2 1 1 Pr],A1',A1);                 % LMI # 2 : A1'*Pr*A1 <= beta_1*Pr
    lmiterm([2 1 1 Pr],-beta_1,1);        

    lmiterm([3 1 1 Pr],Ap1',Ap1);               % LMI # 3 : Ap1'*Pr*Ap1 <= beta_0*Pr
    lmiterm([3 1 1 Pr],-beta_0,1);        

    lmiterm([4 1 1 r],1,1);                     % LMI # 4 : r < 1
    lmiterm([4 1 1 0],-1);   

    lmiterm([-5 1 1 r],1,1);                    % LMI # 5 : r > 0

    lmiterm([-6 1 1 Pr],1,1);                   % LMI # 6  Pr > 0
    % lmiterm([6 1 1 0],0);                    

    lmisys = getlmis;                           % Create the LMI system

    [tmin,rP_feas] = feasp(lmisys) ;
    Pmat = dec2mat(lmisys,rP_feas,Pr);             % display Pr matrix
    r = dec2mat(lmisys,rP_feas,r);               % display r
    
    decay=epsilon;
    rate=r; 
    alpha=log(1/epsilon)/l;
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%-- application --%%%%%%%%%%%%%%%%%%%%

%rt=0.56;
V=x0'*P{count(exec_pattern2(1),'0')+1}*x0;        %%%--V for initial state
horizon=100;
for k=1:floor(horizon/l)
for i=1:size(exec_pattern2)
   if(count(exec_pattern2(i),'1')==1)
       if((size(exec_pattern2,1)/length(exec_pattern)) >= rate)
         m=count(exec_pattern2(i),'0')+1;
         Ts=sampling_time(m);
         sampling_time_pattern(i)=Ts;
         per_step=Ts/1;
         P_pattern{i}=P{m};
         Alpha_pattern(i)=Alpha(m);
         ifStabilizablePattern(i)=ifStabilizable(m);
         time_horizon=Ts;           %%--time limit for this pattern
         
%         tm = t0 : per_step : t0+time_horizon;
%          x = (closed_loops{m}.A-K*closed_loops{m}.B)*x;
%          y = closed_loops{m}.C*x;
         [ym,tm,xm]=initial(closed_loops{m},x0,time_horizon);%%--running till time, based on pattern
         x=[x'; xm]';                                        %%--appending x values for m X h
         t=[t';tm+t0]';                                      %%--appending t values for m X h
         n=[n'; (Ts/h)*ones(size(tm))]';                     %%--appending switching signal
         %     y=[y;ym];       
         figure(3);
         plot(t,V);
         hold on;
         for j=1:size(tm,1)
            Vm= [xm(:,j)'*P{m}*xm(:,j)]';            
            V=[V Vm]
         end
         x0=x(:,size(t,2))';                    %%--last value of x as initial value for next
         t0=t(1,size(t,2));                     %%--last timestamp as initial time for next
         shouldPlot=1;
       else           
           shouldPlot=0;
       end
   end
end   
end

%%%%%%%%%--plot--%%%%%%%%%%
   if(shouldPlot==1)
     figure(1);
     [AX,H1,H2] = plotyy(t,x(1,:),t,n,'plot');              %%% state vs time on switching modes
     set(get(AX(1),'Ylabel'),'String','x');
     set(get(AX(2),'Ylabel'),'String','n');    
     title(sprintf('x,n vs time for pattern: %s',exec_pattern));
     grid on;   
     figure(2);
     [AX_dot,H1_dot,H2_dot] = plotyy(t,x(2,:),t,n,'plot');              %%% state vs time on switching modes
     set(get(AX_dot(1),'Ylabel'),'String','x_{dot}');
     set(get(AX_dot(2),'Ylabel'),'String','n');     
     title(sprintf('x_{dot},n vs time for pattern: %s',exec_pattern));
     grid on;
     figure(3);
     plot(t,V);
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%
samplingTIme=sampling_time';
selfLoopCount=Count';
minDwellTime=Taud';
muVal=Mu';
alphaVal=Alpha';
Gamma=gamma';
settlingTime=set_time';
settlingTImeStabContr=set_m_time';
T=table(samplingTIme,selfLoopCount,minDwellTime,muVal,alphaVal,settlingTime,settlingTImeStabContr,Gamma)
%T2=table(rate',decay')
DwellingRatio
rate