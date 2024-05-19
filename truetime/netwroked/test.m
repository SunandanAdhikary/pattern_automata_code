%%%%% sharedmem %%%%%%%%%
global sharedmem;
sharedmem.P = {};                             % plants
sharedmem.C = {};                             % controllers
sharedmem.O = {};                             % observers
sharedmem.Q = {};                             % costs
sharedmem.R = {};                             % noisevar
sharedmem.Pd1 = {};                           % discrete plants
sharedmem.Od1 = {};                           % observers
sharedmem.C1 = {};                            % controllers
sharedmem.Pd2 = {};                           % discrete plants
sharedmem.Od2 = {};                           % observers
sharedmem.C2 = {};                            % controllers
sharedmem.Pd3 = {};                           % discrete plants
sharedmem.Od3 = {};                           % observers
sharedmem.C3 = {};                            % controllers
sharedmem.Pd4 = {};                           % discrete plants
sharedmem.Od4 = {};                           % observers
sharedmem.C4 = {};                            % controllers
sharedmem.ct = [3,2,3,3];                     % controller counts
sharedmem.x0 = {};                            % initial states
sharedmem.h = [2, 2, 2, 2];
sharedmem.hset = {};
snsids = {'0x11', '0x21', '0x31', '0x41'};
actids = {'0xa1', '0x21', '0x31', '0x41'};

% -- esp -- %%%%%%%%%%%%%%%%%%%%%
A_esp = [-16.95,  -2.365;
                 66.82,   -17.2];
B_esp = [8.478;
             153.1];
C_esp = [0,1];
D_esp = [0];
C_esp1 = eye(size(A_esp,1));
D_esp1 = zeros(size(A_esp,1),size(B_esp,2));
sharedmem.x0{1} = [0;0.2];
esp = ss(A_esp,B_esp,C_esp,D_esp);
p_esp = 0.00001;
Q_esp = p_esp*(C_esp'*C_esp);
R_esp = 0.000001;
Qesp = blkdiag(Q_esp,R_esp);
proc_esp = 0.001; meas_esp = 0.00001;
QN_esp = 5000000*eye(size(B_esp,1));
RN_esp = 10*eye(1);
h_esp = {0.01, 0.02, 0.03};
sharedmem.hset{1} = h_esp;
sharedmem.P{1} = esp;
sharedmem.Q{1} = Qesp;
sharedmem.R{1} = blkdiag(QN_esp, RN_esp);

for i = 1:sharedmem.ct(1)
%     [lqg,K,~,~,L, pd] = lqgdesign(esp,Qesp,QN_esp,RN_esp,h_esp{i});
    Pd = c2d(sharedmem.P{1},h_esp{i});
    [lq, gains] = lqg(Pd,sharedmem.Q{1},sharedmem.R{1});
    sharedmem.Pd1{i} =  Pd;
    sharedmem.O1{i} =  gains.L;
    sharedmem.C1{i} =  gains.Kx;
    sharedmem.Od1{i} = lq;
end
sharedmem.C{1} = sharedmem.C1{sharedmem.h(1)};
sharedmem.O{1} = sharedmem.O1{sharedmem.h(1)};
sharedmem.Od{1} = sharedmem.Od1{sharedmem.h(1)};
sharedmem.Pd{1} = sharedmem.Pd1{sharedmem.h(1)};

% -- ttc-- %%%%%%%%%%%%%%%%%%%%%
A_ttc = [0,1;0,0];
B_ttc = [0;1];
C_ttc = [1,0];
D_ttc = [0];
C_ttc1 = eye(size(A_ttc,1));
D_ttc1 = zeros(size(A_ttc,1),size(B_ttc,2));
sharedmem.x0{2} = [10;0];
ttc = ss(A_ttc,B_ttc,C_ttc,D_ttc);
Q_ttc = [0.16;0.04].*eye(size(A_ttc,2));
R_ttc = eye(size(B_ttc,2));
Qttc = blkdiag(Q_ttc,R_ttc);
QN_ttc = 1500*eye(size(B_ttc,1));
RN_ttc = eye(1);
h_ttc = {0.025, 0.05};
sharedmem.hset{2} = h_ttc;
sharedmem.P{2} = ttc;
sharedmem.Q{2} = Qttc;
sharedmem.R{2} = blkdiag(QN_ttc, RN_ttc);

for i = 1:sharedmem.ct(2)
%     [lqg,K,~,~,L, pd] = lqgdesign(ttc,Qttc,QN_ttc,RN_ttc,h_ttc{i});
    Pd = c2d(sharedmem.P{2},h_ttc{i});
    [lq, gains] = lqg(Pd,sharedmem.Q{2},sharedmem.R{2});
    sharedmem.Pd2{i} =  Pd;
    sharedmem.O2{i} =  gains.L;
    sharedmem.C2{i} =  gains.Kx;
    sharedmem.Od2{i} = lq;
end
sharedmem.C{2} = sharedmem.C2{sharedmem.h(2)};
sharedmem.O{2} = sharedmem.O2{sharedmem.h(2)};
sharedmem.Od{2} = sharedmem.Od2{sharedmem.h(2)};
sharedmem.Pd{2} = sharedmem.Pd2{sharedmem.h(2)};

% -- cc -- %%%%%%%%%%%%
A_cc = [0 1 0;
            0 0 1;
          -6.0476 -5.2856 -0.238];
B_cc = [0; 0; 2.4767];
C_cc = [1 0 0];
D_cc = [0];
C_cc1 = eye(size(A_cc,1));
D_cc1 = zeros(size(A_cc,1),size(B_cc,2));
sharedmem.x0{3} = [0;10;10];
cc = ss(A_cc,B_cc,C_cc,D_cc);
q_cc=0.000000011; r_cc=0.000000001;
Q_cc=q_cc*eye(size(A_cc,2));
R_cc=r_cc*eye(size(B_cc,2));
Qcc=blkdiag(Q_cc,R_cc);
proc_cc= 0.01; meas_cc=0.001;
QN_cc = proc_cc*eye(size(B_cc,1));
RN_cc = meas_cc*eye(size(C_cc,1));
h_cc = {0.01, 0.02, 0.04};
sharedmem.hset{3} = h_cc;
sharedmem.P{3} = cc;
sharedmem.Q{3} = Qcc;
sharedmem.R{3} = blkdiag(QN_cc, RN_cc);

for i = 1:sharedmem.ct(3)
%     [lqg,K,~,~,L, pd] = lqgdesign(cc,Qcc,QN_cc,RN_cc,h_cc{i});
    Pd = c2d(sharedmem.P{3},h_cc{i});
    [lq, gains] = lqg(Pd,sharedmem.Q{3},sharedmem.R{3});
    sharedmem.Pd3{i} =  Pd;
    sharedmem.O3{i} =  gains.L;
    sharedmem.C3{i} =  gains.Kx;
    sharedmem.Od3{i} = lq;
end
sharedmem.C{3} = sharedmem.C3{sharedmem.h(3)};
sharedmem.O{3} = sharedmem.O3{sharedmem.h(3)};
sharedmem.Od{3} = sharedmem.Od3{sharedmem.h(3)};
sharedmem.Pd{3} = sharedmem.Pd3{sharedmem.h(3)};

% --sc-- %%%%%%%%%%%%
A_sc = [3.957e-14  1  1.351e-14   1.193e-16;
        -8             -4                8            4;
        1.96e-14   6.397e-15   1.908e-14  1;
        80             40             -160         -60];
B_sc = [9.628e-15; 80; 20; -1120];
C_sc = [1 0 0 0];
D_sc = [0];
C_sc1 = eye(size(A_sc,1));
D_sc1 = zeros(size(A_sc,1),size(B_sc,2));
sharedmem.x0{4} = [1;0.1;1;0.02];
sc = ss(A_sc,B_sc,C_sc,D_sc);
q_sc=0.1; r_sc=0.005;
Q_sc=q_sc*(C_sc')*C_sc;
R_sc=r_sc*eye(size(B_sc,2));
proc_sc= 0.01; meas_sc=0.01;
QN_sc = proc_sc*eye(size(B_sc,1));
RN_sc = meas_sc*eye(size(C_sc,1));
Qsc = blkdiag(Q_sc,R_sc);
h_sc = {0.02, 0.04, 0.05};
sharedmem.hset{4} = h_sc;
sharedmem.P{4} = sc;
sharedmem.Q{4} = Qsc;
sharedmem.R{4} = blkdiag(QN_sc, RN_sc);

for i = 1:sharedmem.ct(4)
%     [lqg, K, Obs, ~, L, pd] = lqgdesign(sc,Qsc,QN_sc,RN_sc,h_sc{i})
    Pd = c2d(sharedmem.P{4},h_sc{i});
    [lq, gains] = lqg(Pd,sharedmem.Q{4},sharedmem.R{4});
    sharedmem.Pd4{i} =  Pd;
    sharedmem.O4{i} =  gains.L;
    sharedmem.C4{i} =  gains.Kx;
    sharedmem.Od4{i} = lq;
end
sharedmem.C{4} = sharedmem.C4{sharedmem.h(4)};
sharedmem.O{4} = sharedmem.O4{sharedmem.h(4)};
sharedmem.Od{4} = sharedmem.Od4{sharedmem.h(4)};
sharedmem.Pd{4} = sharedmem.Pd4{sharedmem.h(4)};

%
sharedmem.wcet_ctrl = 0.005 ;
sharedmem.rchan = [1,2,3,4];
%
z = sym('z');
Act = ss(1);%tf([1],[1],'Variable','z^-1');                         % Sampler system
Sns = ss(1);%tf([0,1],[1],'Variable','z^-1');                       % Actuator system 

global Systems;
j = 4;
for i = 1 : 4
    N = jtInit;                                                     % Initialize Jittersim
    idx = j*(i-1);
    N = jtAddContSys(N, 1, sharedmem.P{i}, 4, sharedmem.R{i}(1:size(sharedmem.P{i}.A,1),1:size(sharedmem.P{i}.A,1)), sharedmem.Q{i});   % Add sys 1 (Plant), input from sys 4
    N = jtAddDiscSys(N, 2, Sns, 1);                                 % Add sys 2 (Sens), input from 1
%     N = jtAddDiscSys(N, 3, ss(-sharedmem.C{i}), idx+2);%, sharedmem.R{idx+1}, sharedmem.Q{idx+1}) % Add sys 3 (Ctrl), input from 2
    N = jtAddDiscSys(N, 3, sharedmem.Od{i}, 2);%, sharedmem.R{i}, sharedmem.Q{i})      % Add sys 3 (Ctrl), input from 2
    N = jtAddDiscSys(N, 4, Act, 3);                                 % Add sys 4 (Act), input from 3
    N = jtCalcDynamics(N);                                          % Calculate the internal dynamics
    N.samp = 0;
    x = sharedmem.x0{i};
    xhat = 0.*x;
    u = -sharedmem.C{i}*xhat;
    y = sharedmem.P{i}.C*x;
    r = y - sharedmem.P{i}.C*xhat;
    N.tvec = []; %pvec = []; Jvec = []; Jvec1 = [];
    N.xvec = {}; N.yvec = {}; 
    N.xhatvec = {}; N.uvec = {};
    N.tvec(1) = N.Tsim; %pvec(l) = N.P(1,1);  Jvec(1) = N.J;
    % Jvec1(l) = [x;u]'*sharedmem.Q{i}*[x;u];
    N.xvec{1} = x; N.yvec{1} = y; 
    N.xhatvec{1} = xhat; N.uvec{1} = u;
    N.h = sharedmem.Pd{i}.Ts;
    N.rx = actids{i};
    N.tx = snsids{i};
    Systems{i} = N;
end
disp("init done")
%{
% Simulate the system and log the results
Nsteps = 12;                    % Large time steps (control periods)                    % Small time steps (for plotting)
j = 4;
j1 = 4;
for i = 1:j1
    idx = j*(i-1);
%     idx1 = j1*(i-1);
    dt = sharedmem.Pd{i}.Ts;
    l = 1;
    N = Systems{i};
    x = sharedmem.x0{i};
    xhat = 0.*x;
    u = -sharedmem.C{i}*xhat;
    y = sharedmem.P{i}.C*x;
    r = y - sharedmem.P{i}.C*xhat;
    tvec = []; pvec = []; Jvec = []; Jvec1 = [];
    xvec = {}; yvec = {}; 
    xhatvec = {}; uvec = {};
    tvec(l) = N.Tsim; pvec(l) = N.P(1,1);  Jvec(1) = N.J;
    Jvec1(l) = [x;u]'*sharedmem.Q{i}*[x;u];
    xvec{1} = x; yvec{1} = y; 
    xhatvec{1} = xhat; uvec{1} = u;
    plant = sharedmem.Pd{i};
    process = sharedmem.P{i};
    process.C = eye(size(process.A,1));
    process.D = zeros(size(process.A,1),size(process.B,2));
    for k = 1:dt:Nsteps
        l = l+1;
%         if l == 1
            Jvec(l) = N.J;
%         else
%             Jvec(l) = N.J - Jvec(l-1);
%         end
        tvec(l) = N.Tsim; pvec(l) = N.P(1,1);
        tspan = tvec(l-1):0.01:tvec(l)+dt;
        uspan = u.*ones(size(tspan));
        xspan = lsim(process, uspan, tspan, x);
        x = xspan(end,:)';
        Jvec1(l+1)= [x;u]'*sharedmem.Q{i}*[x;u];
        y = plant.C*x;
        xpred = plant.A*xhat+plant.B*u;
        r = y - plant.C*xpred;
        xhat = xpred+sharedmem.O{i}*r;
        u = -sharedmem.C{i}*xhat;
        xvec{l+1} = x;
        yvec{l+1} = y;
        xhatvec{l+1} = xhat;
        uvec{l+1} = u;
        N = jtPassTime(N, dt);
        N = jtExecSys(N,2);
        N = jtExecSys(N,3);
        N = jtExecSys(N,4);
        N.samp = l;
    end
    % Plot the results
%     title('system'+num2str(i));
    subplot(j1,j,idx+1);
    plot(tvec,pvec)
    xlabel('Time')
    ylabel('Process variance')
    subplot(j1,j,idx+2);
    plot(tvec,Jvec1(:,1:end-1))
    xlabel('Time')
    ylabel('cost')
    subplot(j1,j,idx+3);
    plot(tvec,cell2mat(xvec))
    xlabel('Time')
    ylabel('states')
    subplot(j1,j,idx+4);
    plot(tvec,cell2mat(yvec))
    xlabel('Time')
    ylabel('states')
end

function dxdt = odefun1(t,x,u)
    dxdt = sharedmem.P{1}.A*x + sharedmem.P{1}.B*u;
    % sharedmem.odefun{1} = odefun1;
end
function dxdt = odefun2(t,x,u)
    dxdt = sharedmem.P{2}.A*x + sharedmem.P{2}.B*u;
    % sharedmem.odefun{2} = odefun2;
end
function dxdt = odefun3(t,x,u)
    dxdt = sharedmem.P{3}.A*x + sharedmem.P{3}.B*u;
    % sharedmem.odefun{3} = odefun3;
end
function dxdt = odefun4(t,x,u)
    dxdt = sharedmem.P{4}.A*x + sharedmem.P{4}.B*u;
    % sharedmem.odefun{4} = odefun4;
end
%}