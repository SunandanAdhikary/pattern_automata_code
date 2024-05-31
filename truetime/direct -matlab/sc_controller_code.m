function [exectime, System] = sc_controller_code(seg, System)

persistent unew xhatnew Jnew; % msgtx;

sysid = 4;                     % esp

xhat = System.xhatvec{end};
u = System.uvec{end};
switch seg
 case 1
  if System.acess(rem(System.samp+1,System.ct)+1)
    %   msg = ttGetMsg(1);             % Obtain sensor value
      y   = ttAnalogIn(sysid);%msg.data;
      if isempty(y)
        disp('Error in controller: no message received!');
        y = System.yvec{end};%0.0;
      end
    %   ref = ttAnalogIn(sysid);      % Read reference value
      now = ttCurrentTime;%ttGetCPUTime(control_task_sc);
%       disp("sc control")
      System = controller_lqr(System, y, xhat, u);
      unew = System.uvec{end};
      Jnew = System.Jvec(end);
      %------------------------%
      System = updateParams(System,Jnew);
      %------------------------%
    %   msgtx.data =unew;
    %   msgtx.id = System.tx{sysid};
      then = ttCurrentTime;%ttGetCPUTime(control_task_sc);
      exectime = then-now+System.wcet;
%       System.wcet = exectime;
  else
      disp("sc control killed")
      ttKillJob(System.task);
      exectime = 0.00;
  end
 case 2
    ttAnalogOut(sysid, unew);
    ttAnalogOut(4+sysid,Jnew);
%   ttAnalogOut(2*(sysid-1)+2,xhatnew);
%   ttSendMsg(1, unew, 64, System.tx{sysid});    % Send 80 bits to node 2 (actuator)
  exectime = 0.0005;
 case 3
  exectime = -1; % finished
end
