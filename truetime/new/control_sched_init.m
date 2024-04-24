function control_sched_init()

ttInitKernel('prioFP')
% ttInitKernel('prioDM')
% ttInitKernel('prioEDF')

tasknames = {'control_task_esp' , 'control_task_ttc', 'control_task_cc', 'control_task_sc', ...
                'rx_task_esp' , 'rx_task_ttc', 'rx_task_cc', 'rx_task_sc',...
                'dynsched'};
taskexecs = {'control_code_esp' , 'control_code_ttc', 'control_code_cc', 'control_code_sc', ...
                'dynsched_code'};
%                 'rx_code_esp' , 'rx_code_ttc', 'rx_code_cc', 'rx_code_sc',...
% Create dynamic scheduler
data.wcet_ctrl = 0.005 ;
data.h = [1, 2, 2, 2];
data.rchan = [1,2,3,4];
for i = 1 : 4
    ttCreatePeriodicTask(tasknames{i}, 0, data1.h(i), taskexecs{i}, data1);
    ttSetPriority(i, tasknames{i});
end

%{
% Create low-priority controller tasks
data.E = 0.15 ;
ttCreateTask( tasknames{2}, 0.4 , 'scheduling_ctrlcode1', data)
ttSetPriority(2,tasknames{2});
data.E = 0.25 ;
ttCreateTask( tasknames{3}, 0.6, 'scheduling_ctrlcode2', data)
ttSetPriority(3,tasknames{3});
% % Create periodic timer and sampling handler
h1 = 0.4;
ttCreateHandler('samp_handler1', 1, 'scheduling_sampcode1')
%  ttAttachDLHandler('control_task1', 'samp_handler1')
ttCreatePeriodicTimer('samp_timer1', 0, h1, 'samp_handler1')

h2 = 0.6;              % h = 0.6,Tsamp ;  % Sampling period from init argument 
ttCreateHandler('samp_handler2', 1, 'scheduling_sampcode2')
%       ttAttachDLHandler('control_task2', 'samp_handler2')
ttCreatePeriodicTimer('samp_timer', 0, h2, 'samp_handler2')

% low-priority controller tasks design
data1 = [];

s = zpk('s');
% P1 = (0.08*s + 0.08)/(s^2 - 2*s + 1); discrete 
% P2 = (0.2035*s+0.1116)/(1.0000*s^2-0.8471*s+0.1653); discrete
A = [0 1; 1 0];                     % The process (Inverted pendulum system)continuous         
B = [0; 1];
C = [1 0];
D = [0];
[b,a] = ss2tf(A,B,C,D);
P1=tf(b,a);
% P1 = 1/ (s^2 - 1); continuous
% Q1 = diag([1 0.1]);
% R1 = diag([0.1]);
% Qc1=blkdiag(Q1,R1);              % Continuous cost J 
Qc1 = diag([1 0.1]);
tau1 = 0.15;                       % Assumed input-output delay (= Execution time)continuous 
S1 = ss(1);                        % Sampler system
A1 = ss(1);                        % Actuator system

A =[-0.9999    0.9998; -0.0200   -1.9998];  % The process (dc motor)
B =[0.0002;1.9999];
C =[1     0];
D =[0];
[b,a] = ss2tf(A,B,C,D);  
P2 = tf(b,a);
% P2 =  ( 0.0002* s + 2)/ (s^2 + 3* s + 2.02);

R1c = 1;                           % Continuous-time input noise
R2 = 0.01;                         % Discrete-time measurement noise
% Q2 = diag([1 0]);
% R2 = diag([0.001]);
% Qc2=blkdiag(Q2,R2);                   % Continuous cost J 
Qc2 = diag([1 0.1]);
tau2 = 0.25;                       % Assumed input-output delay (= Execution time)
S2 = ss(1);                        % Sampler system
A2 = ss(1);                        % Actuator system 

% Design state feedback 
h1 = 0.4;
C1 = zpk(ss(lqgdesign(P1,Qc1,R1c,R2,h1,tau1),'min'));
% [~,L,~,~,K1,sysd1] = lqgdesign(P1,Qc,R1c,R21,h1);
h2 = 0.6;
C2 = zpk(ss(lqgdesign(P2,Qc2,R1c,R2,h2,tau2),'min'));
% [~,~,~,~,K2,sysd2] = lqgdesign(P2,Qc,R1c,R22,h2bar);


global N

N = jtInit;                          % Initialize Jittersim
N = jtAddContSys(N,1,P1,7,R1c,Qc1);   % Add sys 1 (P), input from sys 7
% N = jtAddContSys(N,2,P2,8,R1c,Qc2); % Add sys 2 (P), input from sys 8
N = jtAddDiscSys(N,3,S1,1,R2);       % Add sys 3 (S), input from 1
% N = jtAddDiscSys(N,4,S2,2,R2);     % Add sys 4 (S), input from 2
% N = jtAddDiscSys(N,3,S,[1,2],R2);  % Add sys 3 (S), input from 1,2
N = jtAddDiscSys(N,5,C1,3);          % Add sys 5 (C), input from 3
% N = jtAddDiscSys(N,6,C2,4);        % Add sys 6 (C), input from 4
N = jtAddDiscSys(N,7,A1,5);          % Add sys 7 (A), input from 5
% N = jtAddDiscSys(N,8,A2,6);        % Add sys 8 (A), input from 6
N = jtCalcDynamics(N);               % Calculate the internal dynamics
N.samp = 0;


N = jtInit;                            % Initialize Jittersim
% N = jtAddContSys(N,1,P1,7,R1c,Qc1);   % Add sys 1 (P), input from sys 7
N = jtAddContSys(N,2,P2,8,R1c,Qc2);     % Add sys 2 (P), input from sys 8
% N = jtAddDiscSys(N,3,S1,1,R2);       % Add sys 3 (S), input from 1
N = jtAddDiscSys(N,4,S2,2,R2);         % Add sys 4 (S), input from 2
% N = jtAddDiscSys(N,3,S,[1,2],R2);    % Add sys 3 (S), input from 1,2
% N = jtAddDiscSys(N,5,C1,3);          % Add sys 5 (C), input from 3
N = jtAddDiscSys(N,6,C2,4);            % Add sys 6 (C), input from 4
% N = jtAddDiscSys(N,7,A1,5);          % Add sys 7 (A), input from 5
N = jtAddDiscSys(N,8,A2,6);            % Add sys 8 (A), input from 6
N = jtCalcDynamics(N);                 % Calculate the internal dynamics
N.samp = 0;

%}

