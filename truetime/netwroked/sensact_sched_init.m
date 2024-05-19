function sensact_sched_init(Systems)

ttInitKernel('prioFP')
% ttInitKernel('prioDM')
% ttInitKernel('prioEDF')
taskct = length(Systems);
tasknames = {'esp_senstx_task', 'ttc_senstx_task' , 'cc_senstx_task', 'sc_senstx_task', ...
            'rxact_task'}
taskexecs = {'esp_senstx_code', 'ttc_senstx_code' , 'cc_senstx_code', 'sc_senstx_code', ...
            'rxact_code'}
% Create periodic tasks for repeated sensing, transmission, reception, actuation

for i=1:taskct
    ttCreatePeriodicTask(tasknames{i}, 0, Systems{i}.h, taskexecs{i}, Systems{i});
    ttSetPriority(i, tasknames{i}); 
end
ttCreateTask('rxact_task', 0.01, 'rxact_code');
ttAttachNetworkHandler('rxact_task');
disp("init sensact")
%{
data = [];
s = zpk('s');
% --inv pend-- %%%%%%%%%%%%
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

% --dc motor-- %%%%%%%%%%%%
A =[-0.9999    0.9998; -0.0200   -1.9998];  % The process (dc motor)
B =[0.0002; 1.9999];
C =[1  0];
D =[0];
[b,a] = ss2tf(A,B,C,D);  
P2 = tf(b,a);
% P2 =  ( 0.0002* s + 2)/ (s^2 + 3* s + 2.02);

% low-priority controller task design
data = [];
s = zpk('s');
P = (0.2035*s+0.1116)/(1.0000*s^2-0.8471*s+0.1653);                        % The process (dc motor)
R1c = 1;                           % Continuous-time input noise
R2 = 0.01;                         % Discrete-time measurement noise
Qc = diag([1 0]);                  % Continuous cost J 
tau = 0.25;                        % Assumed input-output delay (= E)
S = ss(1);                         % Sampler system
A = ss(1);                         % Actuator system
C = zpk(ss(lqgdesign(P,Qc,R1c,R2,h,tau),'min'));
%}

% data.odefun{1} = odefun1;
% data.odefun{2} = odefun2;
% data.odefun{3} = odefun3;
% data.odefun{4} = odefun4;
