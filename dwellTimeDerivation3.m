clc;
clear all;
ops = sdpsettings('verbose',1);
ops1 = sdpsettings('verbose',1);
% { systems
% %LKAS%, dcmotor_speed, suspension_control, esp,
% cruise_control, dcmotor_pos, trajectory, fuel_injection,
% }
system = "trajectory"
mlfonly = 1;
%% Given l,epsolon,sampling period
exec_pattern='1';
exec_pattern1= strrep(exec_pattern, '1','.1');
exec_pattern2= split(exec_pattern1,'.');
exec_pattern2(1)=[];
exec_pattern_states=unique(exec_pattern2);
l_pattern=length(exec_pattern);
% l=l_pattern;
l=5;
epsilon = 0.364;
exp_decay= (log(1/epsilon))/l;  % desired decay rate from (l,epsilon)
exp_decay_dt = epsilon^(1/l);   % exp(-exp_decay); % desired decay rate in discrete domain
stab=1;                         % while loop first run
i=1;
cnt=6;
horizon=100;
%%  plants 
% fuel_injection(Modeling and Control of an Engine Fuel Injection System)--in ECM ECU
if system == "fuel_injection"
    % states:
    % inputs: inlet valve air flow, combustion torque
    % output: AFR
    Ts = 0.005;
    Ts_new = 0.01;
    A = [0.18734,0.13306,0.10468;
        0.08183,0.78614,-0.54529;
        -0.00054 0.10877,0.26882];
    B = [0.00516,-0.0172;
        -0.00073,0.09841;
        -0.00011,0.13589];
    C = [158.16,8.4277,-0.44246];
    D = [0,0];
    open_loop = ss(A,B,C,D);
%     open_loop1 = d2d(open_loop,Ts_new);
%     [A,B,C,D] = ssdata(open_loop1);
    open_loop_dt = c2d(open_loop,Ts);
    [A,B,C,D] = ssdata(open_loop_dt);
    Ts = Ts_new;
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
    x0 = ini(2,:)';
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
    Ts = 0.1;
    A = [1.0000    0.1000; 0    1.0000];
    B = [0.0050; 0.1000];
    C = [1 0];
    D = [0];
    open_loop_dt = ss(A,B,C,D,Ts);
    Ts_new = 0.01;
    open_loop_dt = d2d(open_loop_dt,Ts_new);
    [A,B,C,D,Ts] = ssdata(open_loop_dt);
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
    rate = 0.6;
    QXU = blkdiag(Q,R);
    QWV = blkdiag(QN,RN,1);
    x0 = [10;30];
end
if system=="dcmotor_pos"
    % states: rotational ang., angular vel., armature current
    % output rotational angle
    % input armature voltage
    Ts = 0.02;
    % ctms
    J = 3.2284E-6;
    b = 3.5077E-6;
    KK = 0.0274;
    RR = 4;
    LL = 2.75E-6;
    %     J = 0.01;
    %     b = 0.1;
    %     KK = 0.01;
    %     RR
    %     LL = 0.5;
    Ac = [0 1 0;
        0 -b/J KK/J;
        0 -KK/LL -RR/LL];
    Bc = [0; 0; 1/LL];
    Cc = [1 0 0];
    Dc = [0];
    x0 = [0;10;10];
    % discretize
    [A,B,C,D]=ssdata(c2d(ss(Ac,Bc,Cc,Dc),Ts));
    open_loop = ss(A,B,C,D,Ts);
    Ts_new = 0.2;
    open_loop1 = d2d(open_loop,Ts_new);
    [A,B,C,D] = ssdata(open_loop1);
    Q= 10*eye(size(A,2));
    R= 0.01*eye(size(B,2));
    [K,S,E] = dlqr(A,B,Q,R);
    K
    proc_dev= 0.01;
    meas_dev=0.001;
    QN = proc_dev*proc_dev*eye(size(B,1));
    RN = meas_dev*meas_dev*eye(size(C,1));
    sys_ss = ss(A-B*K,B,C,D,Ts);

    %     [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    safex = 10.*[-4,-20,0;4,20,20];
    % from perfReg.py with this system
    perf = 0.5.*[-1.67,-1.67,-1.67;-1.47,-1.47,-1.47];
    % safer region of this system to start from
    ini = perf;
    % for central chi2 FAR < 0.05
    th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensor_limit = 0.5*4;  % columnwise range of each y
    actuator_limit = 0.5*5;   % columnwise range of each u
    settlingTime = 13;
    rate =0.5;
    QXU = blkdiag(Q,R);
    QWV = blkdiag(QN,RN);
end
if system=="cruise_control"
    % emsoft sumana ghosh 17
    % states: speed, position,?
    % output speed
    % input throttle angle
%     Ts = 0.04;
    Ts = 0.005;
    A = [0 1 0;
        0 0 1;
        -6.0476 -5.2856 -0.238];
    B = [0; 0; 2.4767];
    C = [1 0 0];
    D = [0];
    x0 = [0;10;10];
    open_loop = ss(A,B,C,D);
    open_loop_dt = c2d(open_loop,Ts);
    [A,B,C,D] = ssdata(open_loop_dt);
    q=0.000000011;r=0.000000001;
    %     Q=q*(C')*C;
    Q=q*eye(size(A,2));
    R=r*eye(size(B,2));
    [K,S,E] = dlqr(A,B,Q,R);
    K
    K = acker(A,B,[0.2,0.4,-0.4])
    proc_dev= 0.01;
    meas_dev=0.001;
    QN = proc_dev*proc_dev*eye(size(B,1));
    RN = meas_dev*meas_dev*eye(size(C,1));
    sys_ss = ss(A-B*K,B,C,D,Ts);
    %     [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    %     safex = [];
    %     % from perfReg.py with this system
    %     perf = 0.5.*[-1.67,-1.67,-1.67;-1.47,-1.47,-1.47];
    %     % safer region of this system to start from
    %     ini = perf;
    %     % for central chi2 FAR < 0.05
    %     th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    %     sensor_limit = 0.5*4;  % columnwise range of each y
    %     actuator_limit = 0.5*5;   % columnwise range of each u
    settlingTime = 13;
    rate = 0.55;
    QXU = blkdiag(Q,R);
    QWV = blkdiag(QN,RN);
end
if system=="suspension_control"
    % emsoft sumana ghosh 17
    % states: position, speed of vehicle, suspended mass
    % output speed
    % input force applied on the body
    Ts = 0.04;
%     Ts = 0.08;
    A = [0 1 0 0;
        -8 -4 8 4;
        0 0 0 1;
        80 40 -160 -60];
    B = [0; 80; 20; -1120];
    C = [1 0 0 0];
    D = [0];
    x0 = [1;0.1;1;0.02];
    open_loop = ss(A,B,C,D);
    open_loop_dt = c2d(open_loop,Ts);
    [A,B,C,D] = ssdata(open_loop_dt);
    q=0.1;r=0.005;
    Q=q*(C')*C;
    R=r*eye(size(B,2));
    [K,S,E] = dlqr(A,B,Q,R);
    K
    proc_dev= 0.01;
    meas_dev=0.01;
    QN = proc_dev*proc_dev*eye(size(B,1));
    RN = meas_dev*meas_dev*eye(size(C,1));
    sys_ss = ss(A-B*K,B,C,D,Ts);
    %     [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    %     safex = [];
    %     % from perfReg.py with this system
    %     perf = 0.5.*[-1.67,-1.67,-1.67;-1.47,-1.47,-1.47];
    %     % safer region of this system to start from
    %     ini = perf;
    %     % for central chi2 FAR < 0.05
    %     th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    %     sensor_limit = 0.5*4;  % columnwise range of each y
    %     actuator_limit = 0.5*5;   % columnwise range of each u
    settlingTime = 13;
    rate =0.55;
    QXU = blkdiag(Q,R);
    QWV = blkdiag(QN,RN);
end
if system=="dcmotor_speed"
    % emsoft sumana ghosh 17
    % states: angular vel., armature current
    % output rotational angle
    % input armature voltage
    Ts = 0.05;
    A = [-10 1;
        -0.02 -2];
    B = [0; 2];
    C = [1 0];
    D = [0];
    x0 = [0.2;0.1];
    open_loop = ss(A,B,C,D);
    open_loop_dt = c2d(open_loop,Ts);
    [A,B,C,D] = ssdata(open_loop_dt);
    q=0.01;r=0.001;
    Q=q*(C')*C;
    R=r*eye(size(B,2));
    [K,S,E] = dlqr(A,B,Q,R);
    K
    proc_dev= 0.01;
    meas_dev=0.001;
    QN = proc_dev*proc_dev*eye(size(B,1));
    RN = meas_dev*meas_dev*eye(size(C,1));
    sys_ss = ss(A-B*K,B,C,D,Ts);
    %     [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    %     safex = [];
    %     % from perfReg.py with this system
    %     perf = 0.5.*[-1.67,-1.67,-1.67;-1.47,-1.47,-1.47];
    %     % safer region of this system to start from
    %     ini = perf;
    %     % for central chi2 FAR < 0.05
    %     th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    %     sensor_limit = 0.5*4;  % columnwise range of each y
    %     actuator_limit = 0.5*5;   % columnwise range of each u
    settlingTime = 13;
    rate = 0.55;
    QXU = blkdiag(Q,R);
    QWV = blkdiag(QN,RN);
end
%% electronic stability program/vehicle lateral dynamic control(skip2secure,same as early)-- in Electronic Stability Program ECU
if system=="esp"
    Ts=0.04;
    A = [0.4450 -0.0458;1.2939 0.4402];
    B = [0.0550;4.5607];
    C = [1 0];%[0 1];
    D = [0];
    open_loop_dt = ss(A,B,C,D,Ts);
    open_loop = d2c(open_loop_dt);
    Ts_new = 0.01;
    open_loop_dt = d2d(open_loop_dt,Ts_new);
    [A,B,C,D,Ts] = ssdata(open_loop_dt);
    p = 0.00001;
    Q = p*(C'*C);
    R = 0.000001;
    [K,S,E] = dlqr(A,B,Q,R)
    sys_ss =ss(A-B*K,B,C,D,Ts_new);
    qweight= 1;
    rweight = 1;
    proc_dev= 0.001; meas_dev=0.00001;
    QN = 5000000*eye(size(B,1));
    RN = 10*eye(1);
%     [kalmf,L,P,M] = kalman(sys_ss,QN,RN,Ts_new);
    % K = [-0.0987 0.1420];
    L = [-0.0390;0.4339];
    settling_time = 6;
    safex=[-1,-2;1,2];
    ini= 0.2.*safex;
    perf=0.1.*safex;
    sensorRange = [-2.5;2.5] ;     % columnwise range of each y
    actuatorRange = [-0.8125;0.8125]; % columnwise range of each u
    x0 = ini(1,:)';
    QXU = blkdiag(Q,R);
    QWV = blkdiag(QN,RN);
end

if exist("open_loop_dt")
    [Ap1,Bp1,Cp1,Dp1] = ssdata(open_loop_dt);
else
    if ~exist("open_loop")
        open_loop = d2c(open_loop_dt);
    end
    Ts =0.05;
    open_loop_dt = c2d(open_loop,Ts);
    [Ap1,Bp1,Cp1,Dp1] = ssdata(open_loop_dt);
end
open_loops{i} = d2d(open_loop_dt, Ts);
lqg_reg = lqg(open_loop_dt,QXU,QWV);
[Acd,Bcd,Ccd,Dcd] = ssdata(lqg_reg);  % 
K = -Ccd;
L = Bcd;
%% creating A1, A0
% Ccd = -K
A1 = [Ap1 Bp1*Ccd;Bcd*Cp1 Acd];
A0 = [Ap1 Bp1*Ccd;0.*Bcd*Cp1 eye(size(Acd))];
eig_closed = eig(Ap1);
constraints = [];
constraints_di = [];
isCtrb = [];
isStable = [];
slack = 0.0001;
unstable_count = 0;
h = Ts;
%% Plant design with sampling period=m*h
P_var = sdpvar(size(A1,1),size(A1,1));
constraints = [P_var >= slack];
clfsolved(1) = 1;
mlfsolved(1) = 1;
i = 1;
Ts=i*h;
Am = A0^(i-1)*A1;
eig_state{i} = eig(Am);
isCtrb(i) = rank(Am) >= rank(ctrb(Am,blkdiag(B,B)));
ctrbl = isCtrb(i);
max_eig(i) = max(abs(eig_state{i}));
min_eig = min(abs(eig_state{i}));
isStable(i) = max_eig(i) < 1;
% hold on;
% while ctrbl
while isStable(i)%isCtrb(i)%
    max_eig(i) = max(abs(eig_state{i}));
    min_eig = min(abs(eig_state{i}));
    isStable(i) = max_eig(i) < 1;
    alpha(i) = max_eig(i)^2-1;
    %     assign(alpha(i),0.9);
    Bm = [Bp1;Bp1];%zeros(2.*size(B));
    Cm = blkdiag(Cp1,Cp1);%zeros(size(Ccd)));
    Dm = zeros(size(Cm,1),size(Bm,2));
    closed_loops{i}=ss(Am,Bm,Cm,Dm,Ts);
    %     [Y,T,X] = step(closed_loops{i});
    [Y,T,X] = initial(closed_loops{i},[x0;x0]);
    figure("Name","Outputs for pattern 1(0)^"+num2str(i-1)+" periodicity= "+num2str(Ts));
    plot(Y(:,1:2));
    legend(["y1","y2","y3","y4"]);
%%%%%%%%%%%%
    if i ==1
        [n,d] = ss2tf(Am,Bm,Cm,Dm);
        closed_loop_tfs{1} = tf(n(1,:),d(1,:));
        [AA,BB,CC,DD] = tf2ss(closed_loop_tfs{i}.num{1}, closed_loop_tfs{i}.den{1});
        closed_loop_tf2sss{i} = ss(AA,BB,CC,DD,Ts);
    else
        closed_loop_tfs{i} = tf(closed_loop_tfs{1},'IODelay', i-1);
        [AA,BB,CC,DD] = tf2ss(closed_loop_tfs{i}.num{1}, closed_loop_tfs{i}.den{1});
        closed_loop_tf2sss{i} = ss(AA,BB,CC,DD,Ts);
    end
    figure("Name","Poles and zeroes for 1(0)^"+num2str(i-1));
    subplot(2,1,1);
    pzplot(closed_loops{i});
    title('p-z for augmented closed loops with ')
    subplot(2,1,2);
    pzplot(closed_loop_tfs{i});
    title('p- z for delayed tfs with ');
    %     P_di{i} = dlyap(Am,alpha(i)*eye(size(Am)));
    Pm_di{i}=sdpvar(size(Am,1),size(Am,1));
    constraint_di= [Am'*Pm_di{i}*Am-(1+alpha(i))*eye(size(Pm_di{i}))*Pm_di{i} <= slack,...
                                                                        Pm_di{i} >= slack];
    if isStable(i)
        fprintf("\n stable for %dh sampling time\n",i);
        howStabler(i) = max_eig(i)/exp_decay_dt;
        isStabler(i) = howStabler(i) >= 1;
        if isStabler(i)
            fprintf("\n %f times LESS stabler than desired for %dh sampling time\n",howStabler(i),i);
        end
        constraints = [constraints, Am'*P_var*Am-(1+alpha(i))*eye(size(Am))*P_var <= slack];
    else
        fprintf("\n unstable for %dh sampling time\n",i);
        howStabler(i) = 0;
        isStabler(i) = 0;
        unstable_count = unstable_count + 1;
    end
    if unstable_count > 5
        i = i-1;
        break;
    end
    %%%--LMi solution to find out Lyapunov func Fast switching--%%%
    fprintf("CLF : solving for alpha = "+num2str(alpha(i))+ " for the system with sampling period from "+ num2str(h)+"s to "+num2str(Ts)+ "s\n");
    sol=optimize(constraints,[],ops);
    sol.info
    clfsolved(i) = sol.problem
    if clfsolved(i) == 0 && isStable(i)  %&& mlfonly == 0
        fprintf("CLF..\n");
        P = value(P_var)
        P_di{i} = P;
    else
        fprintf("MLF : solving for alpha = "+num2str(alpha(i))+" for the system with "+num2str(Ts)+ " sampling period\n");
%         sol=optimize(constraint_di,[],ops);
%         sol.info
%         mlfsolved(i) = sol.problem
%         %         alpha(i) = alpha(i) + 0.01;
%         P_di{i}=value(Pm_di{i})           %%--P values for discrete sys
    end
    Alpha_di(i) = value(alpha(i));
    %     Alpha_di(i) = -min(eig(blkdiag(Q,Q)))/max(eig(P_di{i}));% Q+K'*B'*A+A'*B*K
    Alpha_diBar(i)=1+Alpha_di(i);
    i=i+1;
    Ts=i*h;
% --------------update feedback gains?-------------------------- %
%     open_loops{i} = d2d(open_loop_dt, Ts);
%     [Ap1,Bp1,Cp1,Dp1] = ssdata(open_loops{i});
%     lqg_reg = lqg(open_loops{i},QXU,QWV);
%     [Acd,Bcd,Ccd,Dcd] = ssdata(lqg_reg);  % K = -Ccd;
%     %% creating A1, A0
%     % Ccd = -K
%     A1 = [Ap1 Bp1*Ccd;Bcd*Cp1 Acd];
%     A0 = [Ap1 Bp1*Ccd;0.*Bcd*Cp1 eye(size(Acd))];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Am = A0^(i-1)*A1;
    eig_state{i} = eig(Am);
    isCtrb(i) = rank(Am) >= rank(ctrb(Am,blkdiag(B,B)));
%     ctrbl = isCtrb(i);
    max_eig(i) = max(abs(eig_state{i}));
    min_eig = min(abs(eig_state{i}));
    isStable(i) = max_eig(i) < 1;
    if i > 10
        break;
    end
end
% legend([legendstring]);
% hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% p=1;q=1;
for i=1:size(Pm_di,2)
%     if howStabler(i)>1
%         mum_di{i}= howStabler(i)*0.9; %sdpvar(1,1);
%     else
        mum_di{i} =1+slack;
%     end
    for j=1:size(Pm_di,2)
        if clfsolved(i) == 1  || mlfonly == 1
            constraints_di = [constraints_di,...
                Pm_di{i}-mum_di{i}*eye(size(Pm_di{j}))*Pm_di{j} <= slack];%,...
%                                                                mum_di{i} >= 1] ;
        end
    end
end
%%%%%%%%%%%%--Solving LMI to find out P--%%%%%%%%%%%%%%
fprintf("mlf derivation by cs...")
% check(constraints_di);
% sol1 = optimize(constraints_di,[],ops);
% musolved = sol1.problem
sol = optimize(constraints_di,[],ops);
sol.info
for i=1:size(Pm_di,2)
    if clfsolved(i) == 1  || mlfonly == 1
        mlfsolved(i) = sol.problem
        %         alpha(i) = alpha(i) + 0.01;
        P_di{i}=value(Pm_di{i})           %%--P values for discrete sys
    end
end
p=1;q=1;
gammaStable = [];
gammaLessstable = [];
gammaUnstable=[];
%%%%%%%%%%%--Gamma,Taud calculation--%%%%%%%%%%%%%
for i=1:size(P_di,2)
    Mu(i) = mum_di{i};
    Taud(i)= -log(Mu(i))/log(Alpha_diBar(i));  % considering gamma_i <= 1
    %         Taud(i)= log(Mu(i))/abs(log(Alpha_diBar(i)));
    gamma(i)= log(Alpha_diBar(i)*((Mu(i))^(1/Taud(i))));
    %        gamma(i)= Alpha_di(i)+(log(Mu(i))/Taud(i))
%     if(isStabler(i))
    if(howStabler(i) >= 1)
        gammaStable(p) = gamma(i);      %%--gamma_plus
        %          gammaStable(p)= (gamma_eig(i));
        p=p+1;
    else
        gammaLessstable(q) = gamma(i);   %%%--gamma_minus
        %          gammaUnstable(q)= (gamma_eig(i));
        q=q+1;
    end

end  % end for
%%%%%%%%%%%%%%--Values derived--%%%%%%%%%%%%%%%
DwellingRatio = 1;
if(~isempty(gammaLessstable)) && (~isempty(gammaStable))
%     DwellingRatio= ceil((log(max(gammaLessstable))+log(exp_decay_dt))/(-log(max(gammaStable))+log(exp_decay_dt)));
    DwellingRatio= ceil((max(gammaLessstable)+log(exp_decay_dt))/(-(max(gammaStable))+log(exp_decay_dt)));
    % DwellingRatio= ceil((((max(gammaUnstable)))+(exp_decay))/(((max(gammaStable))))-(exp_decay));
    % T_minus (dwelling time in stable sys)/T_plus (dwelling time in unstable sys)
else
    if isempty(gammaStable)
        fprintf("All are less stable modes than desired.\n")
    end
    if isempty(gammaLessstable)
        fprintf("All are more stable modes than desired.\n")
    end
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
%%%%%%%%%--plot--%%%%%%%%%%
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

%% Printing Data Tables
nn = size(P_di,2);
for i=1:nn
    Sampling_Time(i)=closed_loops{i}.Ts;
    Count(i)= (ceil(Taud(i)/closed_loops{i}.Ts));%%%--Permissible self loop count
end
Self_Loop_Count=Count';
Min_Dwell_Time=Taud';
% Dwell_Time = dwell_durations(:,1:nn)';
Mu_Val= Mu(:,1:nn)';
Alpha_val=Alpha_diBar(:,1:nn)';
HowStable=howStabler(:,1:nn)';
% Gamma=log(gamma_eig');
Gamma=log(gamma(:,1:nn)');
T=table(Sampling_Time',Self_Loop_Count,Min_Dwell_Time,Mu_Val,Alpha_val,HowStable)
exp_decay_dt
exp_decay
DwellingRatio
%rate
