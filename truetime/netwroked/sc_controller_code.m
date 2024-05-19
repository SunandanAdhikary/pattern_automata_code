function [exectime, System] = sc_controller_code(seg, System)

persistent unew xhatnew msgtx;
disp("sc control")
sysid = 4;                  % esp
N = System;
xhat = N.xhatvec{end};
u = N.uvec{end};
switch seg
 case 1
  msg = ttGetMsg(1);             % Obtain sensor value
  y   = msg.data;
  if isempty(y)
    disp('Error in controller: no message received!');
    y = N.yvec{end};%0.0;
  end
  ref = ttAnalogIn(sysid);      % Read reference value
  now = ttGetCPUTime();
  [unew, xhatnew] = controler_lqr(sharedmem, y, xhat, u, sysid);
  N = jtExecSys(N,2);
  N = jtExecSys(N,3);
  N = jtExecSys(N,4);
  N.samp = N.samp+1;
  N.xhatvec{N.samp}= xhatnew;
  N.uvec{N.samp}= unew;
  msgtx.data =unew;
  msgtx.id = System.tx{sysid};
  then = ttGetCPUTime();
  exectime = then-now;
 case 2
  ttAnalogOut(2*(sysid-1)+1,unew);
  ttAnalogOut(2*(sysid-1)+2,xhatnew);
  ttSendMsg(1, unew, 64, System.tx{sysid});    % Send 80 bits to node 2 (actuator)
  exectime = 0.0005;
 case 3
  exectime = -1; % finished
end
