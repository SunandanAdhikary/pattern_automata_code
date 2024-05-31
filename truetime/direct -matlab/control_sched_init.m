function control_sched_init(Systems)

ttInitKernel('prioFP')
starttime = 0.0;
taskct = length(Systems);
ttCreateHandler('dl_miss_handler', 1, 'dl_miss_code');
% Periodic dummy task with higher priority
period = 0.01;
arg = 0.05;
wcet = period*arg;
ttCreatePeriodicTask('dummy_task', starttime, period, 'dummy_code', wcet);
ttSetPriority(1, 'dummy_task');
ttAttachDLHandler('dummy_task', 'dl_miss_handler');
%% control tasks
tasknames = {'control_task_esp' , 'control_task_ttc', 'control_task_cc', 'control_task_sc', ...
                'dynsched'};
taskexecs = {'esp_controller_code' , 'ttc_controller_code', 'cc_controller_code', 'sc_controller_code', ...
                'dynsched_code'};
%                 'rx_code_esp' , 'rx_code_ttc', 'rx_code_cc', 'rx_code_sc',...
% Create dynamic scheduler
hyp = period*1000;
for i = 1 : taskct
    sys = Systems{i};
    sys.task = tasknames{i};
    sys.h*1000;
    hyp = lcm(hyp, sys.h*1000);
    sys.samp = 0;
end
hyp = hyp/1000
for i = 1 : taskct
    sys = Systems{i};
    sys.ct = hyp/sys.h;
    sys.acess = ones(1, sys.ct);
    sys.util = getUtil(sys);
    sys.task
%     tasknames{i}
    ttCreatePeriodicTask(sys.task, starttime, sys.h, taskexecs{i}, sys);
%     ttCreateTask(sys.task, starttime, taskexecs{i}, sys);
%     ttSetPeriod(sys.h, sys.task);
    ttSetPriority(i+1, sys.task);
    ttAttachDLHandler(sys.task, 'dl_miss_handler');
end
% ttCreatePeriodicTask('dynsched', hyp-0.005, hyp, taskexecs{taskct+1}, Systems);
% ttCreateTask('dynsched', hyp, taskexecs{taskct+1}, Systems);
% ttCreateJob('dynsched', hyp);
% ttAttachDLHandler('dynsched', 'dl_miss_handler');
disp("init control")

