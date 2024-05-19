function [exectime, System] = esp_senstx_code(seg, System)

persistent y1;
disp("esp sense")
switch seg
 case 1
    y1 = ttAnalogIn(1);
    exectime = sharedmem.wcet_snac;
 case 2
    msg1.data =y1;
    msg1.id = System.tx;
    msg1.timestamp = ttCurrentTime();
    ttSendMsg(2, msg1, 64, System.tx); % Send message (64 bits) to node 2 (controller)
    
    exectime = sharedmem.wcet_snac;
 case 3
    exectime = -1;                         % finished
end
