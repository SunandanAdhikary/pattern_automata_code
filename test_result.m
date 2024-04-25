% clc;
% clear all;
%% pattern
n = 100;
pat = ones(1,n);
% subseq0 ='10010'
subseq = [1 ];%[1 1 1 1 0 1 0 0 1 0 0 0 1 0 0 0 0]
zeroct = size(subseq,2)-1
% subseq1= strrep(subseq0, '1','.1');
% subseq= split(subseq1,'.');
% subseq(1)=[];
repeat = ceil(size(pat,2)/size(subseq,2));
offset = 0;
for i=1:repeat
    for j=1:size(subseq,2)
        if offset+j< size(pat,2)
            pat(offset+j) = subseq(j);
        end
    end
    offset = offset + size(subseq,2);
end

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
if system=="trajectory"
    Ts = 0.1;
    A = [1.0000    0.1000; 0    1.0000];
    B = [0.0050; 0.1000];
    C = [1 0];
    D = [0];
    open_loop_dt = ss(A,B,C,D,Ts);
    Ts_new = 2*0.05;
    open_loop_dt = d2d(open_loop_dt,Ts_new);
    [A,B,C,D,Ts] = ssdata(open_loop_dt);
    q = 100;
    r =0.0000001;
    Q= q*eye(size(A,2));
    R= r*eye(size(B,2));
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
    x0 = [1;30];
end
if system=="cruise_control"
    % emsoft sumana ghosh 17
    % states: speed, position,?
    % output speed
    % input throttle angle
    Ts = 0.02;
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
    q=0.011;r=0.0001;
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
    Ts = 0.03;
%     Ts = 0.08;
    A = [0 1 0 0;
        -8 -4 8 4;
        0 0 0 1;
        80 40 -160 -60];
    B = [0; 80; 20; -1120];
    C = [1 0 0 0];
    D = [0];
    x0 = [0.03;0.3;0.12;0.8];
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
%% electronic stability program/vehicle lateral dynamic control(skip2secure,same as early)-- in Electronic Stability Program ECU
if system=="esp"
    Ts=0.04;
    A = [0.4450 -0.0458;1.2939 0.4402];
    B = [0.0550;4.5607];
    C = [1 0];%[0 1];
    D = [0];
    open_loop_dt = ss(A,B,C,D,0.04);
    open_loop = d2c(open_loop_dt);
    Ts_new = 0.02;
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
max_zero_ol(i) = max(abs(zero(open_loops{i})));
lqg_reg = lqg(open_loop_dt,QXU,QWV);
[Acd,Bcd,Ccd,Dcd] = ssdata(lqg_reg);  % 
K = -Ccd;
L = Bcd;
sys_sss{i} =ss(A-B*K,B,C,D,Ts);
max_zero_cl(i) = max(abs(zero(sys_sss{i})));
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
    closed_loops{i}=ss(Am,Bm,Cm,Dm,h);%Ts);
    %     [Y,T,X] = step(closed_loops{i});
    [Y,T,X] = initial(closed_loops{i},[x0;x0]);
    % figure("Name","Outputs for pattern 1(0)^"+num2str(i-1)+" periodicity= "+num2str(Ts));
    % plot(Y(:,1:2));
    % legend(["y1","y2","y3","y4"]);
%%%%%%%%%%%%
    if i == 1
        [n,d] = ss2tf(Am,Bm,Cm,Dm);
        closed_loop_tfs{1} = tf(n(1,:),d(1,:));
        [AA,BB,CC,DD] = tf2ss(closed_loop_tfs{i}.num{1}, closed_loop_tfs{i}.den{1});
        closed_loop_tf2sss{i} = ss(AA,BB,CC,DD,Ts);
    else
        closed_loop_tfs{i} = tf(closed_loop_tfs{1},'IODelay', i-1);
        [AA,BB,CC,DD] = tf2ss(closed_loop_tfs{i}.num{1}, closed_loop_tfs{i}.den{1});
        closed_loop_tf2sss{i} = ss(AA,BB,CC,DD,Ts);
    end
    max_zero_cltfs_d(i) = max(abs(zero(closed_loop_tfs{i})));
    max_eig_d(i) = max(abs(pole(closed_loop_tfs{i})));
    max_eig_i(i) = max(abs(pole(closed_loops{i})));                                                                        
    if isStable(i)
        fprintf("\n stable for %dh sampling time\n",i);
        howStabler(i) = max_eig(i)/exp_decay_dt;
        isStabler(i) = howStabler(i) >= 1;
        if isStabler(i)
            fprintf("\n %f times LESS stabler than desired for %dh sampling time\n",howStabler(i),i);
        end
        % constraints = [constraints, Am'*P_var*Am-(1+alpha(i))*eye(size(Am))*P_var <= slack];
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
    i=i+1;
    Ts=i*h;
% --------------drop-------------------------- %
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
    % --------------update feedback gains-------------------------- %
    open_loops{i} = d2d(open_loop_dt, Ts);
    [AAp1, BBp1, CCp1, DDp1] = ssdata(open_loops{i});
    max_zero_mr_ol(i) = max(abs(zero(open_loops{i})));
    max_eig_mr_ol(i) = max(abs(pole(open_loops{i})));
    lqg_reg1 = lqg(open_loops{i},QXU,QWV);
    [AAcd,BBcd,CCcd,DDcd] = ssdata(lqg_reg1);  % 
    KK1 = -CCcd;
    AA1 = [AAp1 BBp1*CCcd;BBcd*CCp1 AAcd];
    BBm = [BBp1;BBp1];%zeros(2.*size(B));
    CCm = blkdiag(CCp1,CCp1);%zeros(size(Ccd)));
    DDm = zeros(size(CCm,1),size(BBm,2));
    multirate_cloops{i} = ss(AA1,BBm,CCm,DDm, Ts)
    % max_zero_mr_cl(i) = max(abs(zero(multirate_cloops{i})));
    max_eig_mr_cl(i) = max(abs(pole(multirate_cloops{i})));
    sys_sss{i} =ss(A-B*K,B,C,D,Ts);
    % max_zero_cl(i) = max(abs(zero(sys_sss{i})));
    % max_eig_mr(i) = max(abs(pole(sys_sss{i})));
    % subplot(3,1,3);
    % pzplot(sys_sss{i});
    % title("p- z for multirate with per = "+ num2str(Ts));
    %%%%%%%%%%%%%%%%%%
end
%%%%%%%%% comparison plot %%%%%%%%
multirate_cloops{1} = closed_loops{1};
% for j11 = 1:6
% figure("Name","rate vs CSS change")
% hold on;
% for i11 =j11:j11
% [Y1,T1,X1] =initial(multirate_cloops{i11},[x0;x0]);
% plot(Y1(:,1:1));
% lg(i11) = "y for h x"+num2str(i11);
% [Y11,T11,X11] =initial(closed_loops{i11},[x0;x0]);
% plot(Y11(:,1:1));
% lg1(i11) = "y for 10^"+num2str(i11);
% end
% hold off;
% legend([lg(j11) ,lg1(j11)])
% end
% %% simulate
% xdim = size(A,1);
% udim = size(B,2);
% ydim = size(C,1)
% x = x0
% xhat = x;
% xx = [x;xhat];
% u = -K*x0;
% y = C*x;
% yhat = C*xhat;
% r = y-yhat;
% e = x-xhat;
% ki = 1;
% xi = [value(x0)];
% xhati = [value(x0)];
% ui = [value(u)];
% yi = [value(y)];
% yhati = [value(yhat)];
% ri = [value(r)];
% ei = [value(e)];
% flag = 1;
% trunc = @(x,x_limit)(max(min(x,x_limit),-x_limit));
% pat_func = @(pat,ifexec,ifskip)(pat*(ifexec)+(1-pat)*ifskip);
% for i = 1:n-1
%     %     r = y - Cp1*xhat;
%     x = Ap1*x + Bp1*u ; % +normrnd(0,1/24,[2 1]);
%     %     if pat(i)
%     y = Cp1*x ; % +normrnd(0,1/12,[1 1]);
%     %     end
%     if pat(i)
%         xhat = Ap1*xhat + Bp1*u + L*r;
%     end
%     %     xhat = pat_func(pat(i),Ap1*xhat + Bp1*u + L*r,xhat);
%     %     e = x - xhat;
%     %        if pat(i+1)
%     %             u = pat_func(pat(i+1),-K*x,u)
%     u = -K*xhat;
%     %     xx = Am*[x;xhat] + Bm*[u;u];
%     %     x = xx(1:xdim,:);
%     %     xhat = xx(xdim+1:end,:);
%     %     u = -K*xhat;
%     %     y = Cm*xx;
%     %        end
%     xi = [xi,value(x)];
%     xhati = [xhati,value(xhat)];
%     ui = [ui,value(u)];
%     yi = [yi,value(y(1:ydim,:))];
%     yhati = [yhati,value(y(ydim+1:end,:))];
%     %     ei = [ei,value(e)];
%     %     ri = [ri,norm(r,1)];
% end
% % th = threshold*ones(1,n);
% figure('Name','Simulation Code')
% hold on;
% yb = max(max(yi),-min(yi));
% lgnd = [];
% linewidth = 1.5;
% for j = 1:size(xi,1)
%     plot(1:n,xi(j,:), ColorMode='auto',LineStyleMode='auto');
%     plot(1:n,xhati(j,:), ColorMode='auto',LineStyleMode='auto');
%     lgnd = [lgnd, strcat({'x','xhat'},{num2str(j),num2str(j)})];
% end
% for j = 1:size(ui,1)
%     plot(1:n,ui,ColorMode='auto',LineStyle='-.',LineWidth=linewidth);
%     lgnd = [lgnd, strcat({'u'},{num2str(j)})];
% end
% bar(1:n, yb*pat,'k', FaceAlpha=0.08);
% legend([lgnd,'pattern']);
% ylim('padded');
% xlim([1,n]);
% hold off;