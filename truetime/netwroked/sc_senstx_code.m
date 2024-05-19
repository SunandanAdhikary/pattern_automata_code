function [exectime, System] = sc_senstx_code(seg, System)

persistent y4;

switch seg
 case 1
    y4 = ttAnalogIn(4);
    exectime = sharedmem.wcet_snac;
 case 2
    msg4.data =y4;
    msg4.id = System.tx;
    msg4.timestamp = ttCurrentTime();
    ttSendMsg(2, msg4, 64, System.tx); % Send message (64 bits) to node 2 (controller)
    
    exectime = sharedmem.wcet_snac;
 case 3
    exectime = -1;                         % finished
end
