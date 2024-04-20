function [exectime, data] = control_task(seg, data)

time = ttCurrentTime();
task = ttCurrentTask();
releasetime = ttGetRelease(task);
exectime = ttGetData(task).E;%ttGetWCET(task);
deadline = ttGetDeadline(task);
absdl = releasetime+deadline;%ttGetAbsDeadline(task);
timeleft = ttGetBudget(task);
executed= deadline-timeleft;
execleft = exectime - executed;
strategy = ttGetData(task).dlms;
switch seg
 case 1
%      lateness = 
%      if lateness == 0
          r = ttAnalogIn(data.rChan); % Read reference
          y = ttAnalogIn(data.yChan); % Read process output
          data = controller_lqr(data,y,r); % Calculate LQR action
          exectime = data.E;
          disp("time: "+ttCurrentTime()+", Executing "+ttCurrentTask()+" that started at "+ttGetRelease()+" with abs deadline= "+ttGetAbsDeadline()+" and remaining execution time budget "+ ttGetBudget());    
%       end
 case 2
      ttAnalogOut(data.uChan, data.u); % Output control signal
      exectime = -1;
end