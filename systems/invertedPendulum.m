clear all;
exec_pattern='1001011';
h=0.04;
exec_pattern1= strrep(exec_pattern, '1','.1');
exec_pattern2= split(exec_pattern1,'.');
exec_pattern2(1)=[];
exec_pattern_states=unique(exec_pattern2);

%%%%%%%-- open loop system design --%%%%%%%%%
M = .5;
m = 0.2;
b = 0.1;
I = 0.006;
g = 9.8;
l = 0.3;

p = I*(M+m)+M*m*l^2;                        %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];
states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x'; 'phi'};
open_loop = ss(A,B,C,D,'statename',states,'inputname',inputs,'outputname',outputs);
eig_open= abs(eig(A))                               %%%eigen values of open loop system
%h=bandwidth(open_loop)                             %%%for SISO,need to check
%frequency response for MIMO
%%%%%%%%%% deriving zero order hold of open loop %%%%%%%%%%%
Ts=h;
ZOH_open= c2d(open_loop,Ts,'zoh');
[Ap1,Bp1,Cp1,Dp1] = ssdata(ZOH_open);
eig_openZOH=abs(eig(Ap1))                           %%%eigen values of open loop ZOH

%%%%%%%---lqr---%%%%%%%%%%%
Q = Cp1'*Cp1;
Q(1,1) = 5000;
%Q(3,3) = 1;
R = 1;
[K,S,pc] = lqr(ZOH_open,Q,R);
K

%%%% closed loop system definition %%%%
states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'r'};
outputs = {'x'; 'phi'};
%A1= [Ap1,Bp1*Cc;Bc*Cp1,Ac];
A1= (Ap1-Bp1*K);
B1=zeros(size(A1,1),1);             %%%--Becomes zero
C1=Cp1;                             %%%--remains same,outputs
D1=Dp1;                             %%%--remains same i.e. zero
closed_loop = ss(A1,B1,C1,D1,'statename',states,'inputname',inputs,'outputname',outputs);
eig_closed= abs(eig(A1))
stability_lqrClosed= isstable(closed_loop)
 %%%%%%--pole placement in case of duplicate poles for better understanding--%%%%%%%
 p= eig_closed;
 sp=size(p);
 p_unique=unique(p)
 spu=size(p_unique);
 if(sp(1)>spu(1))
     for i=1:(sp(1)-spu(1))
         p_unique(spu(1)+1)=min(p_unique)-0.11
         spu=size(p_unique);
     end
     Knew = place(Ap1,Bp1,p_unique);
     K=Knew;
%    A1= [Ap1,Bp1*Knew;Bc*Cp1,Ac];
     A1=Ap1-Bp1*Knew;
     closed_loop = ss(A1,B1,C1,D1,'statename',states,'inputname',inputs,'outputname',outputs);
     eig_poleplacemnt= (eig(Ap1-Bp1*Knew))
%    pole(closed_loop)
     state_stability= isstable(closed_loop);
     
 end
 
%%%%%--plant with sampling period=m*h according to the pattern--%%%%%
for i=1:size(exec_pattern_states)
    if(count(exec_pattern_states(i),'1')==1)
        m=count(exec_pattern_states(i),'0')+1;
        Ts=m*h;
        sampling_time(i)=Ts;
        ZOHm= c2d(open_loop,Ts,'zoh');
        [Apm,Bpm,Cpm,Dpm] = ssdata(ZOHm);

        %%%%--closed loop/openloop--%%%%
        
%       Am=[Apm,Bpm*Cc;Bc*Cpm,Ac];
%       B1=zeros(size(A1,1),1);
        Am= (Apm-Bpm*K);
        Bm=zeros(size(Am,1),1);
        Cm=Cpm;
        Dm=Dpm;
        closed_loop = ss(Am,Bm,Cm,Dm,'statename',states,'inputname',inputs,'outputname',outputs);
        fprintf('for %dh sampling period:\n',m);
        eig_state=eig(Am)
        
        %%%--deciding alpha_m for the state--%%%
        if(isstable(closed_loop))
            alpham=-2*eye(size(Am));
        else
            alpham=2*eye(size(Am));
        end
        
        %%%%%%%%--Contrallability check--%%%%%%%%%%%%
        Km=place(Apm,Bpm,p_unique);               %if the controller was designed for mh sampling time        
        closed_m_loop=ss(Apm-Bpm*Km,Bpm,Cpm,Dpm);    %would there be any K possible for stability?
        if(isstable(closed_m_loop))
            fprintf("stabilizable for %dh sampling time",m);
        else
            fprintf("not stabilizable for %dh sampling time",m);
        end
   
        %%%--LMi solution to find out Lyapunov func--%%%
        Pm=sdpvar(size(Am,1),size(Am,1));
        Probm=[Am'*Pm+Pm'*Am-alpham*Pm zeros(size(Am));
                zeros(size(Am)) -Pm];
        constraint= Probm < 0;
        solvesdp(constraint);
        Pm= double(Pm)
        P{i}=Pm;                %P values
        Alpha(i)=det(alpham)^(1/size(alpham,1));        %alpha values
        
%         setlmis([]);
%         Pm= lmivar(1, [size(Am,1) 1]);
%         lmiterm([1 1 1 Pm],Am',1,'s');
%         lmiterm([1 1 1 Pm'],1,Am);
%         lmieqn=getlmis;
        %%%%%-- plot---%%%%%
        t = 0:0.01:5;
        r =0.2*ones(size(t));
        x0=[0 0 0 0];
        [y,t,x]=initial(closed_loop,x0);
%        [y,t,x]=lsim(closed_loop,r,t);
        Vm= x*Pm*x';            
        V{i}=Vm;
        figure(i);
        [AX,H1,H2] = plotyy(t,x(:,2),t,x(:,1),'plot');  
        title(sprintf('State vs time with %dh sampling time',m));
        grid on;
    else
        disp(exec_pattern2(i));
        disp('is a wrong pattern');
     end
end
%%%%%%%%%%%--Mu,Taud calculation--%%%%%%%%%%%%%
for i=1:size(exec_pattern_states)
    for j=1:size(exec_pattern_states)
        mu(j)=det(P{i}/P{j})^(1/size(P{i},1));
    end
    Mu(i)=min(mu);
    Taud(i)=log(Mu(i))/Alpha(i);
    Count(i)=floor(Taud(i)/sampling_time(i));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  