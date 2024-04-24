%%%%% data %%%%%%%%%
global data;
data.P = {};                             % plants
data.Q = {};                             % costs
data.R = {};                             % noisevar
data.Pd1 = {};                           % discrete plants
data.Od1 = {};                           % observers
data.C1 = {};                            % controllers
data.Pd2 = {};                           % discrete plants
data.Od2 = {};                           % observers
data.C2 = {};                            % controllers
data.Pd3 = {};                           % discrete plants
data.Od3 = {};                           % observers
data.C3 = {};                            % controllers
data.Pd4 = {};                           % discrete plants
data.Od4 = {};                           % observers
data.C4 = {};                            % controllers
data.ct = {3,2,3,3};                     % controller counts
data.x0 = {};                            % initial states
data.h = [2, 2, 2, 2];

% -- esp -- %%%%%%%%%%%%%%%%%%%%%
A_esp = [-16.95,  -2.365;
                 66.82,   -17.2];
B_esp = [8.478;
             153.1];
C_esp = [0,1];
D_esp = [0];
data.x0{1} = [0;0.2];
esp = ss(A_esp,B_esp,C_esp,D_esp);
p_esp = 0.00001;
Q_esp = p_esp*(C_esp'*C_esp);
R_esp = 0.000001;
Qesp = blkdiag(Q_esp,R_esp);
proc_esp = 0.001; meas_esp = 0.00001;
QN_esp = 5000000*eye(size(B_esp,1));
RN_esp = 10*eye(1);
h_esp = {0.01, 0.02, 0.03};

data.P{1} = esp;
data.Q{1} = Qesp;
data.R{1} = blkdiag(QN_esp, RN_esp);
for i = 1:data.ct{1}
%     [lqg,K,~,~,L, pd] = lqgdesign(esp,Qesp,QN_esp,RN_esp,h_esp{i});
    Pd = c2d(data.P{1},h_esp{i});
    [lq, gains] = lqg(Pd,data.Q{1},data.R{1});
    data.Pd1{i} =  Pd;
    data.O1{i} =  gains.L;
    data.C1{i} =  gains.Kx;
    data.Od1{i} = lq;
end
data.C{1} = data.C1{data.h(1)};
data.Od{1} = data.Od1{data.h(1)};
data.Pd{1} = data.Pd1{data.h(1)};

% -- ttc-- %%%%%%%%%%%%%%%%%%%%%
A_ttc = [0,1;0,0];
B_ttc = [0;1];
C_ttc = [1,0];
D_ttc = [0];
data.x0{2} = [10;0];
ttc = ss(A_ttc,B_ttc,C_ttc,D_ttc);
Q_ttc = [0.16;0.04].*eye(size(A_ttc,2));
R_ttc = eye(size(B_ttc,2));
Qttc = blkdiag(Q_ttc,R_ttc);
QN_ttc = 1500*eye(size(B_ttc,1));
RN_ttc = eye(1);
h_ttc = {0.025, 0.05};

data.P{2} = ttc;
data.Q{2} = Qttc;
data.R{2} = blkdiag(QN_ttc, RN_ttc);
for i = 1:data.ct{2}
%     [lqg,K,~,~,L, pd] = lqgdesign(ttc,Qttc,QN_ttc,RN_ttc,h_ttc{i});
    Pd = c2d(data.P{2},h_ttc{i});
    [lq, gains] = lqg(Pd,data.Q{2},data.R{2});
    data.Pd2{i} =  Pd;
    data.O2{i} =  gains.L;
    data.C2{i} =  gains.Kx;
    data.Od2{i} = lq;
end
data.C{2} = data.C2{data.h(2)};
data.Od{2} = data.Od2{data.h(2)};
data.Pd{2} = data.Pd2{data.h(2)};

% -- cc -- %%%%%%%%%%%%
A_cc = [0 1 0;
            0 0 1;
          -6.0476 -5.2856 -0.238];
B_cc = [0; 0; 2.4767];
C_cc = [1 0 0];
D_cc = [0];
data.x0{3} = [0;10;10];
cc = ss(A_cc,B_cc,C_cc,D_cc);
q_cc=0.000000011; r_cc=0.000000001;
Q_cc=q_cc*eye(size(A_cc,2));
R_cc=r_cc*eye(size(B_cc,2));
Qcc=blkdiag(Q_cc,R_cc);
proc_cc= 0.01; meas_cc=0.001;
QN_cc = proc_cc*eye(size(B_cc,1));
RN_cc = meas_cc*eye(size(C_cc,1));
h_cc = {0.01, 0.02, 0.04};

data.P{3} = cc;
data.Q{3} = Qcc;
data.R{3} = blkdiag(QN_cc, RN_cc);

for i = 1:data.ct{3}
%     [lqg,K,~,~,L, pd] = lqgdesign(cc,Qcc,QN_cc,RN_cc,h_cc{i});
    Pd = c2d(data.P{3},h_cc{i});
    [lq, gains] = lqg(Pd,data.Q{3},data.R{3});
    data.Pd3{i} =  Pd;
    data.O3{i} =  gains.L;
    data.C3{i} =  gains.Kx;
    data.Od3{i} = lq;
end
data.C{3} = data.C3{data.h(3)};
data.Od{3} = data.Od3{data.h(3)};
data.Pd{3} = data.Pd3{data.h(3)};

% --sc-- %%%%%%%%%%%%
A_sc = [3.957e-14  1  1.351e-14   1.193e-16;
        -8             -4                8            4;
        1.96e-14   6.397e-15   1.908e-14  1;
        80             40             -160         -60];
B_sc = [9.628e-15; 80; 20; -1120];
C_sc = [1 0 0 0];
D_sc = [0];
data.x0{4} = [1;0.1;1;0.02];
sc = ss(A_sc,B_sc,C_sc,D_sc);
q_sc=0.1; r_sc=0.005;
Q_sc=q_sc*(C_sc')*C_sc;
R_sc=r_sc*eye(size(B_sc,2));
proc_sc= 0.01; meas_sc=0.01;
QN_sc = proc_sc*eye(size(B_sc,1));
RN_sc = meas_sc*eye(size(C_sc,1));
Qsc = blkdiag(Q_sc,R_sc);
h_sc = {0.02, 0.04, 0.05};

data.P{4} = sc;
data.Q{4} = Qsc;
data.R{4} = blkdiag(QN_sc, RN_sc);
for i = 1:data.ct{4}
%     [lqg, K, Obs, ~, L, pd] = lqgdesign(sc,Qsc,QN_sc,RN_sc,h_sc{i})
    Pd = c2d(data.P{4},h_sc{i});
    [lq, gains] = lqg(Pd,data.Q{4},data.R{4});
    data.Pd4{i} =  Pd;
    data.O4{i} =  gains.L;
    data.C4{i} =  gains.Kx;
    data.Od4{i} = lq;
end
data.C{4} = data.C4{data.h(4)};
data.Od{4} = data.Od4{data.h(4)};
data.Pd{4} = data.Pd4{data.h(4)};

%
sym z;
Sns = tf(1);                         % Sampler system
Act = tf(1/z);                         % Actuator system 

global Systems;
j = 4;
for i = 1 : 4
    N = jtInit;                          % Initialize Jittersim
    idx = j*(i-1);
    N = jtAddContSys(N, 1, data.P{i}, 4, data.R{i}(1:size(data.P{i}.A,1),1:size(data.P{i}.A,1)), data.Q{i});   % Add sys 1 (Plant), input from sys 4
    N = jtAddDiscSys(N, 2, Sns, 1);                                     % Add sys 2 (Sens), input from 1
%     N = jtAddDiscSys(N, 3, ss(-data.C{i}), idx+2);%, data.R{idx+1}, data.Q{idx+1}) % Add sys 3 (Ctrl), input from 2
    N = jtAddDiscSys(N, 3, data.Od{i}, 2);%, data.R{i}, data.Q{i})      % Add sys 3 (Ctrl), input from 2
    N = jtAddDiscSys(N, 4, Act, 3);                                     % Add sys 4 (Act), input from 3
    N = jtCalcDynamics(N);                                      % Calculate the internal dynamics
    N.samp = 0;
    Systems{i} = N;
end


% Simulate the system and log the results
Nsteps = 10;                    % Large time steps (control periods)                    % Small time steps (for plotting)
j = 3;
% j1 = 4;
for i = 1:4
    idx = j*(i-1);
%     idx1 = j1*(i-1);
    dt = data.Pd{i}.Ts;
    l = 0;
    x = data.x0{i};
    tvec = []; pvec = []; Jvec = []; xvec = [x];
    N = Systems{i};
    for k = 1:dt:Nsteps
        l = l+1;
%         if l == 1
            Jvec(l) = N.J;
%         else
%             Jvec(l) = N.J - Jvec(l-1);
%         end
        tvec(l) = N.Tsim; pvec(l) = N.P(1,1); 
        xvec(l+1) = N.Ac*x;
        N = jtPassTime(N, dt);
        N = jtExecSys(N,2);
        N = jtExecSys(N,3);
        N = jtExecSys(N,4);
        N.samp = l;
    end
    xvec = xvec
    % Plot the results
%     title('system'+num2str(i));
    subplot(4,3,idx+1);
    plot(tvec,pvec)
    xlabel('Time')
    ylabel('Process variance')
    subplot(4,3,idx+2);
    plot(tvec,Jvec)
    xlabel('Time')
    ylabel('Accumulated cost')
    subplot(4,3,idx+3);
    plot(tvec,xvec)
    xlabel('Time')
    ylabel('states')
end
%% 
function dxdt = odefun1(t,x,u)
    dxdt = data.P{1}.A*x + data.P{1}.B*u;
    data.odefun{1} = odefun1;
end
function dxdt = odefun2(t,x,u)
    dxdt = data.P{2}.A*x + data.P{2}.B*u;
    data.odefun{2} = odefun2;
end
function dxdt = odefun3(t,x,u)
    dxdt = data.P{3}.A*x + data.P{3}.B*u;
    data.odefun{3} = odefun3;
end
function dxdt = odefun4(t,x,u)
    dxdt = data.P{4}.A*x + data.P{4}.B*u;
    data.odefun{4} = odefun4;
end