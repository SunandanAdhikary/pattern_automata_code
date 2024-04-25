function [exectime, data] = esp_senstx_code(seg, data)

persistent y

%{
switch seg
	case 1
		% Pass time since last execution
		N = jtPassTimeUntil(N,ttCurrentTime);
 		ttAnalogOut(1,N.J);
        ttAnalogOut(2,trace(N.P));
		

 		% Execute sampler system
% 		N = jtExecSys(N,2);
% 		N.samp = N.samp + 1; % Count nbr of samples in buffer
		
		% Invoke control task             
        ttCreateJob('control_task1' );               % from scheduling_init2, '0.4'
		
		exectime = 1e-6;     % Just to make it show up in schedule     
%             task = sscanf(ttGetInvoker,'control_task1');
%                 if ~isempty(task)
%                 ttKillJob(task)
%                 end

	case 2
		exectime = -1;
end
%}
sysid = 1; % esp
N = Systems{sysid};
x = N.xvec{end};
switch seg
 case 1
    now = ttGetCPUTime;
    u = ttGetMsg(1);
    [y,xnew, Jnew] = plant(data, x, u, sysid);
    N = jtPassTimeUntil(N,ttCurrentTime);
    N.samp = N.samp+1;
    N.xvec{N.samp}= xnew;
    N.yvec{N.samp}= y;
    ttAnalogOut(1,Jnew);%N.J);
    ttAnalogOut(2,y);
    % ttAnalogOut(2,trace(N.P{1}));
    ttSendMsg([1 2], y, 64, data.txids{sysid}); % Send message (64 bits) to node 3 (controller)
    then = ttGetCPUTime;
    exectime = then-now;%data.wcet_snac;%0.0005;
 case 2
%   ttSendMsg(3, y, 80); % Send message (80 bits) to node 3 (controller)
%   exectime = 0.0004;
%  case 3
  exectime = -1; % finished
end
