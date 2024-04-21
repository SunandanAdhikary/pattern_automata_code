function sched_init(arg)

% Task scheduling and control.
%
% The case of three tasks running concurrently on the same CPU controlling 
% two different control systems. The effect of the scheduling policy on the
%  global control performance is demonstrated.
%
% Initialize TrueTime kernel
switch arg
 case 1   % FP scheduling with no deadline handling
  ttInitKernel('prioFP')
  codefcn = {'other_task','control_task'};
  sched_scheme = "RM";
  data.dlms = 'none';
 case 2   % DM scheduling with no deadline handling
  ttInitKernel('prioDM')
  codefcn = {'other_task','control_task'};
  sched_scheme = "DM";
  data.dlms = 'none';
  case 3   % EDF scheduling with no deadline handling
  ttInitKernel('prioEDF')
  codefcn = {'other_task','control_task'};
  sched_scheme = "EDF";
  data.dlms = 'none';
 case 11   % FP scheduling with killnow deadline handling
  ttInitKernel('prioFP')
  codefcn = {'other_task','control_task'};
  sched_scheme = "RM";
  data.dlms = 'killnow';
 case 21   % DM scheduling with killnow deadline handling
  ttInitKernel('prioDM')
  codefcn = {'other_task','control_task'};
  sched_scheme = "DM";
  data.dlms = 'killnow';
 case 31   % EDF scheduling with killnow deadline handling
  ttInitKernel('prioEDF')
  codefcn = {'other_task','control_task'};
  sched_scheme = "EDF";
  data.dlms = 'killnow';
%  case 12   % FP scheduling with skipnxt deadline handling
%   ttInitKernel('prioFP')
%   codefcn = {'other_task','control_task'};
%   sched_scheme = "RM";
%   data.dlms = 'skipnxt';
%  case 22   % DM scheduling with skipnxt deadline handling
%   ttInitKernel('prioDM')
%   codefcn = {'other_task','control_task'};
%   sched_scheme = "DM";
%   data.dlms = 'skipnxt';
%  case 32   % EDF scheduling with skipnxt deadline handling
%   ttInitKernel('prioEDF')
%   codefcn = {'other_task','control_task'};
%   sched_scheme = "EDF";
%   data.dlms = 'skipnxt';
 otherwise
  error('Illegal init argument')
end

% common data 


% Task parameters
starttimes = [0 0 0];
periods = [0.3 0.4 0.6];
executime = [0.05 0.15 0.25];
tasknames = {'security_task', 'acc' , 'dcmotorControl'};
priorities = [1,2,3];

%% high-priority security task
data.E = executime(1) ;
ttCreatePeriodicTask(tasknames{1}, starttimes(1), periods(1), codefcn{1}, data);
ttCreateLog(tasknames{1},1,['response' num2str(1)],1000);
ttSetPriority(priorities(1),tasknames{1});

%% deadline miss interrupt handler
ttCreateHandler('dl_miss_handler', 1, 'dl_miss_code');
ttAttachDLHandler(tasknames{1}, 'dl_miss_handler');
%       ttNoSchedule('dl_miss_handler')

%% ACC plant       
A = [0 1; 1 0];                     
B = [0; 1];
C = [1 0];
D = [0];
sys{1} = ss(A,B,C,D);
Q{1} = diag([1 0.1]);
R{1} = diag([0.1]);
x0{1} = zeros(size(A,1),1);

%% DC motor plant
A = [-0.9999    0.9998; -0.0200   -1.9998]; 
B = [0.0002;1.9999];
C = [1 0];
D = [0];
sys{2} = ss(A,B,C,D);
Q{2} = diag([1 0]);
R{2} = diag([0.001]);
x0{2} = zeros(size(A,1),1);

%% discrete control tasks for plants
for i=1:2
    sys_d = c2d(sys{i},periods(i+1));
    [A_d,B_d,C_d,D_d] = ssdata(sys_d);
    [KEST,L,P] = kalman(sys_d,1,1);
    [K,S,CLP] = dlqr(A_d,B_d,Q{i},R{i});
% system specific data
    data.A_d = A_d;
    data.B_d = B_d;
    data.C_d = C_d;
    data.D_d = D_d;
    data.L = L;
    data.Q = Q{i};
    data.R = R{i};
    data.K = K;
    data.F = -inv(C_d*((eye(size(A_d))-(A_d-B_d*K))\B_d));
    data.J = 0;
% init
    data.x_hat = x0{i};
    data.u = K*x0{i};
% %% deadline miss handler
%     ttCreateHandler('dl_miss_handler', 1, 'dl_miss_code');
%     for i = 1:3
%       ttAttachDLHandler(tasknames{i}, 'dl_miss_handler');
% %       ttNoSchedule('dl_miss_handler')
%     end
    data.E = executime (i+1);
%  input/output channels
    data.rChan = i;
    data.yChan = i+2;
    data.uChan = i;
    data.JChan = i+2;
    % ttCreateTask( tasknames{2}, periods(i+1) , codefcn{2}, data);
    ttCreatePeriodicTask(tasknames{i+1}, starttimes(i+1), periods(i+1), codefcn{2}, data);
    ttCreateLog(tasknames{i+1},1,['response' num2str(i+1)],1000);
    if sched_scheme == "RM"
        ttSetPriority(priorities(i+1),tasknames{i+1});
    end
    ttAttachDLHandler(tasknames{i+1}, 'dl_miss_handler');
end