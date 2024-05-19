function [exectime, Systems] = rxact_code(seg, Systems)

persistent u msg;
disp("act")
switch seg
 case 1
    msg = ttGetMsg(1)
    if ~isempty(msg)
        u = msg.data;
    else
        disp('Error: actuator received empty message!')
    end
%     [y,xnew, Jnew] = plant(data, x, u, sysid);
%     N = jtPassTimeUntil(N,ttCurrentTime);
%     N.samp = N.samp+1;
%     N.xvec{N.samp}= xnew;
%     N.yvec{N.samp}= y;
%     then = ttGetCPUTime;
    exectime = sharedmem.wcet_snac
 otherwise
%     ttAnalogOut(1,Jnew);%N.J);
    if msg.id == Systems{1}.rx
        ttAnalogOut(1,u);
    elseif msg.id == Systems{2}.rx
        ttAnalogOut(2,u);
    elseif msg.id == Systems{3}.rx
        ttAnalogOut(3,u);
    elseif msg.id == Systems{4}.rx
        ttAnalogOut(4,u);
    else
        pass;
    end
    exectime = -1; % finished
end
