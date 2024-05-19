function [exectime, System] = ttc_controller_code(seg, System)

persistent unew xhatnew;% msgtx;

sysid = 2;                       % ttc

xhat = System.xhatvec{end};
u = System.uvec{end};
switch seg
 case 1
 if System.acess(rem(System.samp+1,System.ct)+1)
    %   msg = ttGetMsg(1);             % Obtain sensor value
      y   = ttAnalogIn(sysid);         % msg.data;
      if isempty(y)
        disp('Error in controller: no message received!');
        y = System.yvec{System.samp};%0.0;
      end
    %   ref = ttAnalogIn(sysid);      % Read reference value
      now = ttCurrentTime;%ttGetCPUTime("control_task_ttc");
      disp("ttc control")
      System = controller_lqr(System, y, xhat, u);
      unew = System.uvec{end};
    %   msgtx.data =unew;
    %   msgtx.id = System.tx{sysid};
      then = ttCurrentTime;%ttGetCPUTime(control_task_ttc);
      exectime = then-now;
      System.wcet = exectime;
 else
      disp("ttc control killed")
      ttKillJob(System.task);
      exectime = 0.00;
  end
 case 2
    ttAnalogOut(sysid,unew);
%   ttAnalogOut(2*(sysid-1)+1,unew);
%   ttAnalogOut(2*(sysid-1)+2,xhatnew);
%   ttSendMsg(1, unew, 64, System.tx{sysid});    % Send 80 bits to node 2 (actuator)
  exectime = System.wcet;
 case 3
  exectime = -1; % finished
end
