function [exectime, Systems] = dynsched_code(seg, Systems)

persistent hyp ;
taskct = length(Systems);
hyp = 0.01*1000;
switch seg
 case 1
    for i=1:taskct
    syss{i} = ttGetData(Systems{i}.task);
    end
%      while util >= 1
     for i=1:taskct
        sys = syss{i};
%         Systems{i};
        J = sys.Jvec1(end);
        if J > sys.Jth
            disp("-------more-------");
            if sys.hidx > 1
%                 sys.hidx = sys.hidx-1;
%                 sys.h = sys.hset(sys.hidx);
%                 ttSetPeriod(sys.h);
                sys.util = getUtil(sys);
                sys.acess = getAcess(sys, sys.util, 0);
            else
                disp("no way");
            end
        else
            disp("ok");
        end
        sys.h*1000
        hyp = lcm(hyp, sys.h*1000)
     end  
     hyp = hyp/1000
     util = 0;
     disp("update");
     for i = 1: taskct
         sys = syss{i};
%          sys = ttGetData(Systems{i}.task);
         sys.h*1000
         sys.ct = hyp/sys.h;
         sys.acess = ones(1, sys.ct);
         sys.util = getUtil(sys);
         util = util + sys.util;
%          ttSetData(sys.task, sys);
     end
%      %{
     if util > 1
        disp("needs reduction");
        red_ratio = 1/util;
        if J < sys.Jth*0.3
          disp("--------less-------");
          for i = 1: taskct
            sys = syss{i};
            if sys.hidx < length(sys.hset)
%                 sys.hidx = sys.hidx+1;
%                 sys.h = sys.hset(sys.hidx);
%                 ttSetPeriod(sys.h);
                sys.util = getUtil(sys);
                sys.acess = getAcess(sys, sys.util*red_ratio, 1);
            else
                disp("no way");
            end
          end
        end
     else
% %          }
        disp("======== final update ========");
         for i = 1: taskct
            sys = syss{i};
            ttSetPeriod(sys.h);
            sys.acess = getAcess(sys, sys.util, 1);
            sys.util = getUtil(sys);
            ttSetData(sys.task, sys);
            Systems{i} = sys;
            ttSetData(sys.task, sys);
         end
     end
     exectime = 0.0001;
 case 2
%     ttAnalogOut(sysid, unew);
%   ttAnalogOut(2*(sysid-1)+1,unew);
%   ttAnalogOut(2*(sysid-1)+2,xhatnew);
%   ttSendMsg(1, unew, 64, System.tx{sysid});      % Send 80 bits to node 2 (actuator)
    ttSetData('dynsched', Systems);
    disp("======== data set ========");
    exectime = 0.0005;
 case 3
  exectime = -1; % finished
  now = ttCurrentTime()
  ttCreateJob('dynsched', now+hyp);
end
    
