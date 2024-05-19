function [exectime, data] = dl_miss_code(seg, data)


time = ttCurrentTime();
task = sscanf(ttGetInvoker,'DLtimer:%s');
releasetime= ttGetRelease(task);
taskdata = ttGetData(task);
exectime = ttGetData(task).E;%ttGetWCET(task);
deadline = ttGetDeadline(task);
absdl = releasetime + deadline;%ttGetAbsDeadline(task);
timeleft= ttGetBudget(task);
executed= deadline-timeleft;
execleft = exectime - executed;
strategy = ttGetData(task).dlms;

disp("time: "+time+", Invoked "+ttCurrentTask()+" for "+task+" that started at "+releasetime+" with rel(abs) deadline= "+deadline+"("+absdl+") and remaining execution time "+ execleft);
if strategy == "killnow"
    if ~isempty(task) && releasetime < time %&& (execleft > 0 && execleft < exectime) %&& (releasetime < absdl)
        disp("----- Killing the job instance of "+task+" under dl miss handling strategy "+ strategy);
        ttKillJob(task)
    elseif ~isempty(task) && releasetime >= time
        disp("----- But not to kill the instance of "+task+" that just started now, under dl miss handling strategy "+ strategy);
        ttCreateJob(task, releasetime)
    end
elseif strategy == "none"
   if ~isempty(task)
       disp("----- Deadline miss Ignored for this job instance of "+task+" under dl miss handling strategy "+ strategy);
   end
else
    if ~isempty(task)
       disp("----- Deadline miss Ignored for "+task);
    end
end

exectime = -1;
