function [exectime, data] = esp_controller_code(seg, data)

sysid = 1; % esp
N = Systems{sysid};
xhat = N.xhatvec{end};
u = N.uvec{end};
switch seg
 case 1
  y = ttGetMsg;         % Obtain sensor value
  if isempty(y)
    disp('Error in controller: no message received!');
    y = 0.0;
  end
  ref = ttAnalogIn(1);    % Read reference value
  [unew, xhatnew] = controler_lqr(data, y, xhat, u, sysid);
  N = jtExecSys(N,2);
    N = jtExecSys(N,3);
    N = jtExecSys(N,4);
  N.samp = N.samp+1;
  N.xhatvec{N.samp}= xhatnew;
  N.uvec{N.samp}= unew;
  ttAnalogOut(1,unew);
  ttAnalogOut(2,xhatnew);
  ttSendMsg([1 1], unew, 64, data.rxids{sysid});    % Send 80 bits to node 2 (actuator)
  exectime = 0.0005;
 case 2
  % ttSendMsg(2, data.u, 80);    % Send 80 bits to node 2 (actuator)
  exectime = -1; % finished
end
