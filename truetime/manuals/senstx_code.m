function [exectime, data] = senstx_code(seg, data)

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

switch seg
 case 1
%     N = jtPassTime(N, data.h(1))
    N = jtPassTimeUntil(N,ttCurrentTime);
    ttAnalogOut(1,N.J{1});
    ttAnalogOut(2,trace(N.P{1}));
    odefun = data.odefun{1};
    tspan = 
    [t,x] = ode45(@odefun,tspan, data.x0{1});
    exectime = data.wcet_snac;%0.0005;
    ttSendMsg(1, y, 80); % Send message (80 bits) to node 3 (controller)
 case 2
%   ttSendMsg(3, y, 80); % Send message (80 bits) to node 3 (controller)
%   exectime = 0.0004;
%  case 3
  exectime = -1; % finished
end
