clc;
% clear all;
% yalmip("clear");
format long g
% { systems
% batch_reactor, water_treatment, temp_ctrl, acc_platoon_pf,
% acc_platoon_tplf, %LKAS%, dcmotor_speed, suspension_control,
% cruise_control, dcmotor_pos, trajectory, fuel_injection,
% power_sys,four_car_platoon
% }
system = "dcmotor_speed"
%% pattern
n = 20;
pat = ones(1,n);
% subseq0 ='10010'
subseq = [1 0 0 0 0 0 0 0]
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

%%  plants
if system=="batch_reactor"
    A = [1.38 0.2077 6.715 5.676;
        0.5814 4.29 0 0.675;
        1.067 4.273 6.654 5.893;
        0.048 4.273 1.343 -2.104];
    dim = size(A,1);
    B = [0 0;5.679 0;1.136 -3.146;1.136 0];
    C = [1 0 1 -1;0 1 0 0];
    D = zeros(size(C,1),size(B,2));
    Ts = 0.1;
    %     Ts_new =0.05;
    sys = ss(A,B,C,D);
    sys_d = c2d(sys,Ts);
    %     sysd2 = d2d(sys_d,Ts_new);
    [A,B,C,D,Ts] = ssdata(sys_d);
    open_loop = ss(A,B,C,D,Ts);
    [A,B,C,D] = ssdata(open_loop);
    eig_open= eig(A);
    Q= eye(size(A,2));
    R= 0.05*eye(size(B,2));
    proc_dev= 0.001; meas_dev=0.0001;
    QN = eye(size(B,1)).*proc_dev^2*(B*B');
    RN = eye(size(C,1)).*meas_dev^2;
    QXU = blkdiag(Q,R);
    % QXU=[Q_1c_1 Q_12c_1;Q_12c_1' Q_2c_1];
    QWV = blkdiag(QN,RN);
    x0 = [10;0;0.1;0];
end
if system=="water_treatment"
    A = [-0.138   0         0;
        0.006664 -0.006664  0;
        0         0.002216 -0.002216];
    dim = size(A,1);
    B = [0.1328 0 0;0 0 0;0 0 0];
    C = [0 0 1];
    D = zeros(size(C,1),size(B,2));
    Ts = 0.1;
    Ts_new =0.05;
    sys = ss(A,B,C,D);
    sys_d = c2d(sys,Ts);
    sysd2 = d2d(sys_d,Ts_new);
    [A,B,C,D,Ts] = ssdata(sysd2);
    open_loop = ss(A,B,C,D,Ts);
    [A,B,C,D] = ssdata(open_loop);
    eig_open= eig(A);
    Q= eye(size(A,2));
    R= 0.05*eye(size(B,2));
    proc_dev= 0.001; meas_dev=0.0001;
    QN = eye(size(B,1)).*proc_dev^2*(B*B');
    RN = eye(size(C,1)).*meas_dev^2;
    QXU = blkdiag(Q,R);
    % QXU=[Q_1c_1 Q_12c_1;Q_12c_1' Q_2c_1];
    QWV = blkdiag(QN,RN);
    x0 = [10;0;0.1];
end
if system=="temp_ctrl"
    A=[-0.11 0.1;0 -50];
    B=[0;-50];
    C=[1 0];
    D=[0];
    Ts=0.5;
    Ts_new =0.05;
    sys = ss(A,B,C,D);
    sys_d = c2d(sys,Ts);
    sysd2 = d2d(sys_d,Ts_new);
    [A,B,C,D,Ts] = ssdata(sysd2);
    open_loop = ss(A,B,C,D,Ts);
    [A,B,C,D] = ssdata(open_loop);
    eig_open= eig(A);
    Q= eye(size(A,2));
    R= 0.05*eye(size(B,2));
    proc_dev= 0.001; meas_dev=0.0001;
    QN = eye(size(B,1)).*proc_dev^2*(B*B');
    RN = eye(size(C,1)).*meas_dev^2;
    QXU = blkdiag(Q,R);
    % QXU=[Q_1c_1 Q_12c_1;Q_12c_1' Q_2c_1];
    QWV = blkdiag(QN,RN);
    x0 = [10;0];
    safex = [-30,-30;30,30];
end
if system=="acc_platoon_pf"
    N = 3; %excluding leader
    tau = 0.5; %inertial delay of vehicle longitudinal dynamics
    d = 20; %desired spacing d_{i,i-1} in m
    veh_len = 4; %in m
    A = [0 1 0;
        0 0 1;
        0 0 -1/tau];
    dim = size(A,1);
    B = [0;0;1/tau];
    C = eye(dim);
    D = zeros(size(C,1),size(B,2));
    Ts = 0.1;
    Ts_new =0.05;
    sys = ss(A,B,C,D);
    sys_d = c2d(sys,Ts);
    sysd2 = d2d(sys_d,Ts_new);
    [A,B,C,D,Ts] = ssdata(sysd2);
    L = [0	 0	0;
        -1	 1	0;
        0	-1	1];
    P = [1	 0	0;
        0	 0	0;
        0	 0	0];
    Ac = kron(eye(N),A);
    Bc = kron(L+P,B);
    Cc = zeros(N,dim*N);
    for ii = 1:N
        Cc(ii,(ii-1)*dim+1) = 1;
    end
    Dc = zeros(size(Cc,1),size(Bc,2));
    x0=[];
    for jj=1:N
        x0 = [x0;(N-jj+1)*veh_len+(N-jj+1)*d;20;0];
    end
    open_loop = ss(Ac,Bc,Cc,Dc,Ts);
    [A,B,C,D] = ssdata(open_loop);
    eig_open= eig(Ac);
    Q= eye(size(A,2));
    R= 0.05*eye(size(B,2));
    proc_dev= 0.001; meas_dev=0.0001;
    QN = eye(size(B,1)).*proc_dev^2*(B*B');
    RN = eye(size(C,1)).*meas_dev^2;
    QXU = blkdiag(Q,R);
    % QXU=[Q_1c_1 Q_12c_1;Q_12c_1' Q_2c_1];
    QWV = blkdiag(QN,RN);
end
if system=="acc_platoon_tplf"
    N = 3; % excluding leader
    tau = 0.5; % inertial delay of vehicle longitudinal dynamics
    d = 20; % desired spacing d_{i,i-1} in m
    veh_len = 4; % in m
    A = [0 1 0;
        0 0 1;
        0 0 -1/tau];
    dim = size(A,1);
    B = [0;0;1/tau];
    C = eye(dim);
    D = zeros(size(C,1),size(B,2));
    Ts = 0.1;
    Ts_new =0.05;
    sys = ss(A,B,C,D);
    sys_d = c2d(sys,Ts);
    sysd2 = d2d(sys_d,Ts_new);
    [A,B,C,D,Ts] = ssdata(sysd2);
    L = [0	 0	0;
        -1	 1	0;
        -1	-1	2];
    P = [1	0	0;
        0	1	0;
        0	0	1];
    Ac = kron(eye(N),A);
    Bc = kron(L+P,B);
    Cc = zeros(N,dim*N);
    for ii = 1:N
        Cc(ii,(ii-1)*dim+1) = 1;
    end
    Dc = zeros(size(Cc,1),size(Bc,2));
    x0=[];
    for jj=1:N
        x0 = [x0;(N-jj+1)*veh_len+(N-jj+1)*d;20;0];
    end
    open_loop = ss(Ac,Bc,Cc,Dc,Ts);
    [A,B,C,D] = ssdata(open_loop);
    eig_open= eig(Ac);
    Q= eye(size(A,2));
    R= 0.0000000005*eye(size(B,2));
    proc_dev= 0.1; meas_dev=0.1;
    QN = eye(size(B,1)).*proc_dev^2*(B*B');
    RN = eye(size(C,1)).*meas_dev^2;
    QXU = blkdiag(Q,R);
    % QXU=[Q_1c_1 Q_12c_1;Q_12c_1' Q_2c_1];
    QWV = blkdiag(QN,RN);
end
if system=="four_car_platoon"
    % states: position and velocities of 4 vehicles
    % inputs: velocities
    % output: positions
    Ts = 0.1;
    A =[1.10517091807565,0.0110517091807565,0,0,0,0,0,0;0,1.10517091807565,0,0,0,0,0,0;0,0,1.10517091807565,0.0110517091807565,0,0,0,0;0,0,0,1.10517091807565,0,0,0,0;0,0,0,0,1.10517091807565,0.0110517091807565,0,0;0,0,0,0,0,1.10517091807565,0,0;0,0,0,0,0,0,1.10517091807565,0.0110517091807565;0,0,0,0,0,0,0,1.10517091807565];
    B =[5.34617373191714e-05,0,0,0;0.0105170918075648,0,0,0;5.34617373191714e-05,-5.34617373191714e-05,0,0;0.0105170918075648,-0.0105170918075648,0,0;0,5.34617373191714e-05,-5.34617373191714e-05,0;0,0.0105170918075648,-0.0105170918075648,0;0,0,5.34617373191714e-05,-5.34617373191714e-05;0,0,0.0105170918075648,-0.0105170918075648];
    C =[1,0,0,0,0,0,0,0;0,0,1,0,0,0,0,0;0,0,0,0,1,0,0,0;0,0,0,0,0,0,1,0];
    D =zeros(size(C,1),size(B,2));
    actuator_limit = [10;10;10;10];
    sensor_limit = [20;20;20;20];
    safex = [-20,-10,-20,-10,-20,-10,-20,-10;20,10,20,10,20,10,20,10];
    Q = C'*C;
    R = B'*B;
    open_loop = ss(A,B,C,D,Ts);
    Ts_new =0.03;
    open_loop = d2d(open_loop,Ts_new);
    [A,B,C,D,Ts] = ssdata(open_loop);
    x0 = [0;10;20;20;30;30;40;40];
    %     [K,S,E]=dlqr(A,B,Q,R);
    %     sys_d =ss(A-B*K,B,C,D,Ts);
    %     L = [7.38472324606530e-09	1.10767378478111e-08	-3.69178068993179e-09	-2.35766404514692e-13;...
    %         -8.06607123878028e-08	-1.20987484468706e-07	4.03243462966293e-08	2.45231890733761e-12;...
    %         1.10767378620785e-08	2.21532418194634e-08	-1.47682827951273e-08	3.69154491818929e-09;...
    %         -1.20987484656173e-07	-2.41972543487752e-07	1.61309378729129e-07	-4.03218939403918e-08;...
    %         -3.69178073241593e-09	-1.47682827566559e-08	2.21530060221851e-08	-1.47685185520782e-08;...
    %         4.03243467777148e-08	1.61309378236735e-07	-2.41970090967650e-07	1.61311830983905e-07;...
    %         -2.35746720559148e-13	3.69154491801782e-09	-1.47685185240281e-08	1.84614611047725e-08;...
    %         2.45206180642458e-12	-4.03218938420706e-08	1.61311830610115e-07	-2.01648197004208e-07];
    perf = 0.1.*safex;
    threshold =4.35;
    uatkon=[1];   % attack on which u
    yatkon=[1];   % attack on which y
    rate =0.5;
    proc_dev= 0.1; meas_dev=0.1;
    QN = eye(size(B,1)).*proc_dev^2*(B*B');
    RN = eye(size(C,1)).*meas_dev^2;
    QXU = blkdiag(Q,R);
    % QXU=[Q_1c_1 Q_12c_1;Q_12c_1' Q_2c_1];
    QWV = blkdiag(QN,RN);
end
if system=="power_sys"
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
    %     eig_openZOH=eig(Ap1);
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
    Ts = 0.01;
    Ts_new = 0.01;
    A = [0.18734,0.13306,0.10468;
        0.08183,0.78614,-0.54529;
        -0.00054 0.10877,0.26882];
    B = [0.00516,-0.0172;
        -0.00073,0.09841;
        -0.00011,0.13589];
    C = [158.16,8.4277,-0.44246];
    D = [0,0];
    open_loop = ss(A,B,C,D,Ts);
    open_loop1 = d2d(open_loop,Ts_new);
    [A,B,C,D] = ssdata(open_loop1);
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
    open_loop = ss(A,B,C,D,Ts);
    Ts_new = 0.2;
    open_loop1 = d2d(open_loop,Ts_new);
    [A,B,C,D,Ts] = ssdata(open_loop1);
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
    Ts = 0.02
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
    Ts = 0.04;
    A = [0 1 0;
        0 0 1;
        -6.0476 -5.2856 -0.238];
    B = [0; 0; 2.4767];
    C = [1 0 0];
    D = [0];
    x0 = [0;10;10];
    open_loop = ss(A,B,C,D,Ts);
    open_loop_dt = d2d(open_loop,Ts);
    [A,B,C,D] = ssdata(open_loop_dt)
    q=100;r=1;
    %     Q=q*(C')*C;
    Q=q*eye(size(A,2));
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
if system=="suspension_control"
    % emsoft sumana ghosh 17
    % states: position, speed of vehicle, suspended mass
    % output speed
    % input force applied on the body
    Ts = 0.08;
    A = [0 1 0 0;
        -8 -4 8 4;
        0 0 0 1;
        80 40 -160 -60];
    B = [0; 80; 20; -1120];
    C = [1 0 0 0];
    D = [0];
    x0 = [10;10;10;0];
    open_loop = ss(A,B,C,D);
    open_loop_dt = c2d(open_loop,Ts);
    [A,B,C,D] = ssdata(open_loop_dt);
    q=1;r=1;
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
    rate =0.55;
    QXU = blkdiag(Q,R);
    QWV = blkdiag(QN,RN);
end
if system=="dcmotor_speed"
    % emsoft sumana ghosh 17
    % states: angular vel., armature current
    % output rotational angle
    % input armature voltage
    Ts = 0.1;
    A = [-10 1;
        -0.02 -2];
    B = [0; 2];
    C = [1 0];
    D = [0];
    x0 = [2;0.1];
    open_loop = ss(A,B,C,D);
    open_loop_dt = c2d(open_loop,Ts);
    [A,B,C,D] = ssdata(open_loop_dt);
    q=1;r=1;
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
% if system=="LKAS"
%     % emsoft sumana ghosh 17
%     % states: angular vel., armature current
%     % output rotational angle
%     % input armature voltage
%     Ts = 0.1;
%     A = [-10 1;
%         -0.02 -2];
%     B = [0; 2];
%     C = [1 0];
%     D = [0];
%     x0 = [10;10;10;0];
%     open_loop = ss(A,B,C,D);
%     open_loop_dt = c2d(open_loop,Ts);
%     [A,B,C,D] = ssdata(open_loop_dt);
%     q=1;r=1;
%     Q=q*(C')*C;
%     R=r*eye(size(B,2));
%     [K,S,E] = dlqr(A,B,Q,R);
%     K
%     proc_dev= 0.01;
%     meas_dev=0.001;
%     QN = proc_dev*proc_dev*eye(size(B,1));
%     RN = meas_dev*meas_dev*eye(size(C,1));
%     sys_ss = ss(A-B*K,B,C,D,Ts)
%     %     [kalmf,L,P,M] = kalman(sys_ss,QN,RN);
%     %     safex = [];
%     %     % from perfReg.py with this system
%     %     perf = 0.5.*[-1.67,-1.67,-1.67;-1.47,-1.47,-1.47];
%     %     % safer region of this system to start from
%     %     ini = perf;
%     %     % for central chi2 FAR < 0.05
%     %     th = 4.35;              % new: changed it a bit so that without attack, the residue remains below threshold.
%     %     sensor_limit = 0.5*4;  % columnwise range of each y
%     %     actuator_limit = 0.5*5;   % columnwise range of each u
%     settlingTime = 13;
%     rate = 0.55;
%     QXU = blkdiag(Q,R);
%     QWV = blkdiag(QN,RN);
% end
if exist("open_loop_dt")
    [Ap1,Bp1,Cp1,Dp1] = ssdata(open_loop_dt);
else
    [Ap1,Bp1,Cp1,Dp1] = ssdata(open_loop);
end
[lqg_reg,gains] = lqg(open_loop_dt,QXU,QWV);
[Acd,Bcd,Ccd,Dcd]=ssdata(lqg_reg);
K = -Ccd; % gains.Kx;
L = Bcd; % gains.L;
A1 = [Ap1 Bp1*Ccd;Bcd*Cp1 Acd];
A0 = [Ap1 Bp1*Ccd;0.*Bcd*Cp1 eye(size(Acd))];
Am = A0^zeroct*A1;
Bm = zeros(2.*size(B));
Cm = blkdiag(Cp1,zeros(size(Ccd)));
Dm = zeros(size(Cm,1),size(Bm,2));
eig_closed = eig(Ap1)
% sys_ss = ss(A-B*K,zeros(size(B)),C,D,Ts);
figure('Name','system linear sim')
initial(sys_ss,x0)
sys_aug_ss = ss(Am,Bm,Cm,Dm,Ts);
figure('Name','Augmented system linear sim')
initial(sys_aug_ss,[x0;x0])
%% simulate
x = x0
xhat = x;
u = -K*x0;
y = C*x;
r = y-C*xhat;
e = x-xhat;

ki = 1;
xi = [value(x0)];
xhati = [value(x0)];
ui = [value(u)];
yi = [value(y)];
ri = [value(r)];
ei = [value(e)];
flag = 1;
trunc = @(x,x_limit)(max(min(x,x_limit),-x_limit));
pat_func = @(pat,ifexec,ifskip)(pat*(ifexec)+(1-pat)*ifskip);
for i=1:n-1
    %     r = y - Cp1*xhat;
    x = Ap1*x + Bp1*u;%+normrnd(0,1/24,[2 1]);
    %     if pat(i)
    y = Cp1*x;%+normrnd(0,1/12,[1 1]);
    %     end
    xhat = pat_func(pat(i),Ap1*xhat + Bp1*u + L*r,xhat);
    %     e = x - xhat;
    %        if pat(i+1)
    %             u = pat_func(pat(i+1),-K*x,u)
    u = -K*xhat;
    %        end
    xi = [xi,value(x)];
    xhati = [xhati,value(xhat)];
    ui = [ui,value(u)];
    yi = [yi,value(y)];
    %     ei = [ei,value(e)];
    %     ri = [ri,norm(r,1)];

end
% th = threshold*ones(1,n);
figure('Name','Simulation Code')
hold on;
yb = max(max(yi),-min(yi));
lgnd = [];
linewidth = 1.5;
for j = 1:size(xi,1)
    plot(1:n,xi(j,:), ColorMode='auto',LineStyleMode='auto');
    plot(1:n,xhati(j,:), ColorMode='auto',LineStyleMode='auto');
    lgnd = [lgnd, strcat({'x','xhat'},{num2str(j),num2str(j)})];
end
for j = 1:size(ui,1)
    plot(1:n,ui,ColorMode='auto',LineStyle='-.',LineWidth=linewidth);
    lgnd = [lgnd, strcat({'u'},{num2str(j)})];
end
bar(1:n, yb*pat,'k', FaceAlpha=0.08);
legend([lgnd,'pattern']);
ylim('padded');
xlim([1,n]);
hold off;