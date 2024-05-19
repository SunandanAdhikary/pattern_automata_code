function [exectime, System] = cc_senstx_code(seg, System)

persistent y3;

switch seg
 case 1
    y3 = ttAnalogIn(3);
    exectime = sharedmem.wcet_snac;
 case 2
    msg3.data =y3;
    msg3.id = System.tx;
    msg3.timestamp = ttCurrentTime();
    ttSendMsg(2, msg3, 64, System.tx); % Send message (64 bits) to node 2 (controller)
    
    exectime = sharedmem.wcet_snac;
 case 3
    exectime = -1;                         % finished
end
