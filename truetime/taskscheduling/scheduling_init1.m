function scheduling_init1()

% ttInitKernel('prioFP')
ttInitKernel('prioDM')
% ttInitKernel('prioEDF')

tasknames = {'security_task', 'control_task1' , 'ctrl_task'};
% Create one high-priority security task
data.E = 0.05 - sqrt(eps);
ttCreatePeriodicTask( tasknames{1}, 0, 0.300, 'scheduling_dummycode1', data);
ttSetPriority(1,tasknames{1});
data.E = 0.15 - sqrt(eps);
ttCreatePeriodicTask(tasknames{2}, 0, 0.400, 'scheduling_dummycode1', data);
ttSetPriority(2,tasknames{2});
% Create low-priority controller task
data.E = 0.25 - sqrt(eps);
ttCreateTask( tasknames{3}, 0.6, 'scheduling_ctrlcode1', data)
ttSetPriority(3,tasknames{3});
% % Create periodic timer and sampling handler
h = 0.6;              % h = 0.6,Tsamp ;  % Sampling period from init argument 
ttCreateHandler('samp_handler', 1, 'scheduling_sampcode3')
%     for i = 1:3
%       ttAttachDLHandler('ctrl_task', 'samp_handler')
%     end
ttCreatePeriodicTimer('samp_timer', 0, h, 'samp_handler')

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

global N
N = jtInit;                          % Initialize Jittersim
N = jtAddContSys(N,1,P,4,R1c,Qc);    % Add sys 1 (P), input from sys 4
N = jtAddDiscSys(N,2,S,1,R2);        % Add sys 2 (S), input from 1
N = jtAddDiscSys(N,3,C,2);           % Add sys 3 (C), input from 2
N = jtAddDiscSys(N,4,A,3);           % Add sys 4 (A), input from 3
N = jtCalcDynamics(N);               % Calculate the internal dynamics
N.samp = 0;




% J = 0.386