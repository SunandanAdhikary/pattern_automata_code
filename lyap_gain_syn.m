
clc;
clf;
clear all;
system = "trajectory"

%% systems
if system=="esp"
    Ts = 0.04;
    A = [0.4450 -0.0458;1.2939 0.4402];
    B = [0.0550;4.5607];
    C = [0 1];
    K = [0.2826    0.0960];
    L = [-0.0390;0.4339];
    safex = [-1, -2;1, 2];
    safer = 1;
    sensor_limit = 2.5;
    actuator_limit = 0.8125;
    perf_region_depth = 0.1;
    perf= perf_region_depth.*safex;
    threshold = 4.35;
    nonatk_cov = 0.1169;
    safer_center = [-0.7258    1.7258   -0.7250    1.5611]';
    safer_scale = 0.2742;
    t=8;
end
if system=="trajectory"
    Ts = 0.1;
    Ts_2=0.01;
    A = [1.0000    0.1000; 0    1.0000];
    B = [0.0050; 0.1000];
    C = [1 0];
    D = [0];
    sys = ss(A,B,C,D,Ts);
    sys_2 = d2d(sys,Ts_2);
    [A,B,C,D,Ts] = ssdata(sys_2)
    Q= 0.001*eye(size(A,2));
    R= 0.0000001*eye(size(B,2));
    [K,S,E] = dlqr(A,B,Q,R);
    QN = 1500;
    RN = eye(1);
    sys_ss = ss(A,B,C,D,0.1);
    [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
    safex = [-25,-30;25,30];
    safer = 0.6;
    sensor_limit = 30;
    actuator_limit = 36; %36;
    perf_region_depth = 0.3;
    perf = perf_region_depth.*safex;
    threshold = 4.35;
    nonatk_cov = 12.6;
    safer_center = [18.1468   -9.5400   17.3949   -12.6147]';
    safer_scale = 3.3258;
    t=20;
end
if system=="aircraft"
    % states: attack angle, pitch rate, pitch angle
    % output pitch angle
    % input angle of deflection
    A = [-0.313 56.7 0;-0.0139 -0.426 0;0 56.7 0];
    B = [0.232;0.0203;0];
    C = [0 0 1];
    D = zeros(size(C,1),size(B,2));
    Ts = 0.02;  
        % safery constraints
    safex = [0 0 0; 2 2 2];
    % performance region of this system
    perf = [0.68 0.68 0.68;0.72 0.72 0.72];
    % safer region of this system to start from
    ini = 0.5*safex;
    % for central chi2 FAR < 0.05
    th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
    sensorRange = [-3;3];  % columnwise range of each y
    % columnwise range of each u
    actuatorRange = [-0.4363 ;0.4363 ];   % from ctms control tab
    Q= blkdiag(1/2,1/2,1/2);%6.8*eye(size(C,1));
    R= 1/min(abs(actuatorRange))^2;
%     [K,S,E] = dlqr(A,B,Q,R);
%     K
    proc_dev= 0.001;
    meas_dev= 0.0001;
    QN = 1/(proc_dev*proc_dev)*(B*B');
    RN = 1/(meas_dev*meas_dev);
%     sys_ss = ss(A-B*K,B,C,D,Ts);
%     [kalmf,L,P,M] = kalman(sys_ss,QN,RN);

    settling_time = 13; 
    noisy_zvar= 0.14;
    noisy_zmean= 0.52;
    noisy_delta= 1.86;
    nonatk_zvar= 12.6041;%15.8507
    nonatk_zmean= 0.6064;
    nonatk_delta= 0.0292;
    uatkon=[1];   % attack on which u
    yatkon=[1];   % attack on which y     
end

%% extended state-space
% AA= [A zeros(size(A));L*C (A-L*C)] ;
% BB= [B;B];
% C1 = [C zeros(1,size(A,1))];
% C2 = [zeros(1,size(A,1)) C];
% KK = [zeros(size(K)) K];
% AA_cl = AA-BB*KK;
% lower_safety = safer_center - safer_scale;
% upper_safety = safer_center + safer_scale;


x0 = 0.8.*safex(1,:).';
ref = 0.5*sum(perf,1).';
%% Optimization Problem
cons= [];
k = 2;
isCovered = false;
P = sdpvar(size(A,1)*2,size(A,2)*2);
Px = sdpvar(size(A,1),size(A,2));
Ac = [A, -B*K;L*C*A, A-L*C*A-B*K];
slack = 0.01;
goal = perf(2,:);
sour = safex(2,:);
while isCovered == false & sour(1)- goal(1)>1
    if k>10
        goal = goal + (sour-goal)/2;
        k = 2;
    end
    desired_delay = (norm(goal,inf)/norm(sour,inf))^(1/k)
    lyap_decay = (desired_delay*desired_delay) - 1
    cons=[Ac'*P*Ac-(1+lyap_decay)*eye(size(A,1)*2)*P<=0, P>=slack];
%     consx=[(A+B*inv(R)*B'*Px)'*Px*(A+B*inv(R)*B'*Px)-(1+lyap_decay)*eye(size(A,1))*Px<=0, Px>=0];
%     consx=[(A-B*inv(R+B'*Px*B)*B'*P*A)'*Px*(A-B*inv(R+B'*Px*B)*B'*Px*A)-(1+lyap_decay)*eye(size(A,1))*Px<=0, Px>=0];
    sol = optimize(cons);
%     solx = optimize(consx);
    if sol.problem == 0
        isCovered = true;
    else
        k = k+1;
    end
end

% Pmat = value(Px)
% value(k)
% K_new = inv(R+B'*Pmat*B)*B'*Pmat*A;%-inv(Rc)*Bc'*Pmat;
% newsys = ss(A+B*K,B,C,D,Ts);
% x0 = [safex(2,:)'];
% initial(sys, x0);

Pmat = value(P)
Rc = blkdiag(R,R)
Bc = blkdiag(B,B)
Cc = blkdiag(C,C);
Dc = blkdiag(D,D);
K_new = inv(Rc+Bc'*Pmat*Bc)*Bc'*Pmat*Ac;%-inv(Rc)*Bc'*Pmat;
x0 = [safex(2,:)';safex(2,:)'];
sys = ss(Ac-Bc*K_new,Bc,Cc,Dc,Ts);
initial(sys, x0)
% simulate 
trunc = @(x,x_limit)(max(min(x,x_limit),-x_limit));
xdim=size(A,1);
udim=size(B,2);
ydim =size(C,1);
N =200;
xi=[];
xhati=[];
x= x0;
u = -K_new*x;
y= Cc*x;
for i=1:N
%     r = y - C*(A*xhat + B*u);
    x = Ac*x + Bc*u;
    y = Cc*x;%trunc(Cc*x,[sensor_limit);
%     xhat = A*xhat + B*u + L*r;
%     e = x - xhat;
    u = -K_new*x;%trunc(-K_new*xhat,actuator_limit);
    xi = [xi,x(1:xdim,:)];
    xhati = [xhati,x(xdim+1:end,:)];
%     ui = [ui,value(ua)];
%     yi = [yi,value(ya)];
end

% x= x0;
% xhat = x0;
% e = x - xhat;
% r = C*e;
% u = -K_new*x;
% y = C*x;
% for i=1:N
%     r = y - C*(A*xhat + B*u);
%     x = A*x + B*u;
%     y = C*x;%trunc(Cc*x,[sensor_limit);
%     xhat = A*xhat + B*u + L*r;
%     e = x - xhat;
%     u = -K_new*x;%trunc(-K_new*xhat,actuator_limit);
%     xi = [xi,x(1:xdim,:)];
%     xhati = [xhati,x(xdim+1:end,:)];
%     ui = [ui,value(ua)];
%     yi = [yi,value(ya)];
% end
figure;
hold on;
plot(xi');
plot((safex(1,:)'*ones(1,N))');
plot((safex(2,:)'*ones(1,N))');
plot((perf(1,:)'*ones(1,N))');
plot((perf(2,:)'*ones(1,N))');
hold off;



