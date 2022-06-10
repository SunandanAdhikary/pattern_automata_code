clc;
clear all;
diary power_plant.out;
%%%%%%%%%%%%%%--Given l,epsolon,sampling period--%%%%%%%%%%%%%
exec_pattern='101';
exec_pattern1= strrep(exec_pattern, '1','.1');
exec_pattern2= split(exec_pattern1,'.');
exec_pattern2(1)=[];
exec_pattern_states=unique(exec_pattern2);
l=length(exec_pattern);

unit_decay = 0.005; epsilon = 0.0005;
exp_decay= (log(1/epsilon))/l;              % desired decay rate from (l,epsilon)
plot=0;        

% a = -2.5;
 a = -2;
% a = -1.5;
         x0 = [10;20];                         % ini state
         x = x0;         
         t0 = [0];                               
         t = t0;
         n=0;                                   %switching 1 or 2
        A1= [2 2;1 3];
        B1=zeros(size(A1,1),1);                         %%%--Becomes zero
        C1=eye(size(A1,2));                             %%%--remains same,outputs
        D1=zeros(size(A1,1),1);                         %%%--remains same i.e. zero
        sys1 = ss(A1,B1,C1,D1);
        eig_state{1}=eig(A1);
%        f1 = bandwidth(sys1);
        
        A2= [a 1;1 -2];
        B2=zeros(size(A2,1),1);             %%%--Becomes zero
        C2=eye(size(A2,2));                         %%%--remains same,outputs
        D2=zeros(size(A2,1),1);                               %%%--remains same i.e. zero
        sys2 = ss(A2,B2,C2,D2);
        eig_state{2}=eig(A2);
%        f2 = bandwidth(sys2);
        
        sys{1} = sys1;
        sys{2} = sys2;
        constrs=[];
        for i =1:1:2

            Am=sys{i}.A;
            Bm=sys{i}.B;
            Cm=sys{i}.C;
            Dm=sys{i}.D;
            
            if(isstable(sys{i}))
                ifStable(i)=1;
                alpham=-2.5*eye(size(Am));
                fprintf("\n stable for %d state",i);
                Alpha(i)=-1*det(alpham)^(1/size(alpham,1));        %%--alpha values
            else
                ifStable(i)=0;
                alpham=3.5*eye(size(Am));
                fprintf("\n not stable for %d state",i);
                Alpha(i)=det(alpham)^(1/size(alpham,1));           %%--alpha values
            end

            %%%--LMi solution to find out Lyapunov func--%%%
            Pm{i}=sdpvar(size(Am,1),size(Am,1));
            Probm=[Am'*Pm{i}+Pm{i}'*Am-alpham*Pm{i} zeros(size(Am));
                    zeros(size(Am)) -Pm{i}];
%                constrs=[constrs, Probm <= 0];
            constraint= [Probm <= 0];
            solvesdp(constraint);
%            Pm= double(Pm{i});
            P{i}=double(Pm{i});                                                %%--P values
        end
%%%%%%%%%%%--Mu,Taud calculation--%%%%%%%%%%%%%
for i=1:size(sys,2)
    mum{i}=sdpvar(1,1);
    for j=1:size(sys,2)
%        mu(j)=det(P{i}/P{j})^(1/size(P{i},1));
        constrs=[constrs, P{i}-mum{i}*eye(size(P{j}))*P{j}<= 0, mum{i}>1]
    end
%    mu
%    Mu(i)=abs(max(mu))*2;                                      %%%--MU_i for ith state
 %   td_min(i)= abs(log(Mu(i))/Alpha(i));                     %%%%-dwell time for ith state
%    Count(i)= abs(floor(Taud(i)/sampling_time(i)));         %%%--Permissible self loop count
%    gamma(i)= Alpha(i)+(log(Mu(i)))/td_min(i);

%    if(isstable(sys{i}))
%        gammaS= gamma(i);                     %%--gamma_plus
  %      gammaS(i)= max(eig_state{i})         %%--gamma_plus
%    else
%        gammaU= gamma(i);    %%%--gamma_minus
  %      gammaU(i)= max(eig_state{i})
%    end
end
solvesdp(constrs);
p=1;q=1;
for i=1:size(sys,2)
    Mu(i)=double(mum{i})
%    P{i}=double(Pm{i})
    td_min(i)= abs(log(Mu(i))/Alpha(i));                      %%%%-dwell time for ith state
%    Count(i)= (floor(Taud(i)/sampling_time(i)));         %%%--Permissible self loop count
    gamma(i)= abs(Alpha(i)+(log(Mu(i)))/td_min(i));
    if(ifStable(i))
        gammaS(p)= abs(gamma(i));      %%--gamma_plus
        p=p+1;
    else
        gammaU(q)=abs(gamma(i));    %%%--gamma_minus
        q=q+1;
    end
end
%%%%%%%%%%%%%%--Values derived--%%%%%%%%%%%%%%%
DwellingRatio= (abs(max(gammaU))+exp_decay)/(abs(max(gammaS))-exp_decay);

% V=x0'*P{count(exec_pattern2(1),'0')+1}*x0;                   %%%--V for initial state

%%%%%%%%%%%%%%%%%%%%%%--for alternted switching mode--%%%%%%%%%%%%%%%
for i=1:size(exec_pattern2)
   if(count(exec_pattern2(i),'1')==1)
         m=count(exec_pattern2(i),'0')+1;
         [ym,tm,xm]=initial(sys{m},x0);
         x=[x'; xm]';                                        %%--appending x values for m X h
         t=[t';tm+t0]';                                      %%--appending t values for m X h
         n=[n'; (m)*ones(size(tm))]';                        %%--appending switching signal
         %     y=[y;ym];
         Vm= [xm*P{m}*xm']';            
   %      V=[V;Vm]';
         x0=x(:,size(t,2))';                    %%--last value of x as initial value for next
         t0=t(1,size(t,2));                     %%--last timestamp as initial time for next
         plot=1;                                %%%%%%%%%%--if you don't want plot--%%%%%%%%%%%%
       end
end


%%%%%%%%%--plot--%%%%%%%%%%
   if(plot==1)
     figure(1);
     [AX,H1,H2] = plotyy(t,n,t,x(2,:),'plot');              %%% state vs time on switching modes
     set(get(AX(1),'Ylabel'),'String','theta');
     set(get(AX(2),'Ylabel'),'String','theta_{dot}');
     title(sprintf('State vs time for pattern: %s',exec_pattern));
     grid on; 
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%
MU=Mu';
ALPHA=Alpha';
minDwellTime=td_min';

%table(t',n',x')
table(MU,ALPHA,minDwellTime)
DwellingRatio
gamma