function [exectime,data] = scheduling_sampcode1(seg,data)

% Emulate sampling
global N

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
