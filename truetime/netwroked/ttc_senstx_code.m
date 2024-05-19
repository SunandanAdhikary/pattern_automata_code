function [exectime, System] = ttc_senstx_code(seg, System)

persistent y2;
disp("ttc sense")
switch seg
 case 1
    y2 = ttAnalogIn(2);
    exectime = sharedmem.wcet_snac;
 case 2
    msg2.data =y2;
    msg2.id = System.tx;
    msg2.timestamp = ttCurrentTime();
    ttSendMsg(2, msg2, 64, System.tx); % Send message (64 bits) to node 2 (controller)
    
    exectime = sharedmem.wcet_snac;
 case 3
    exectime = -1;                         % finished
end
