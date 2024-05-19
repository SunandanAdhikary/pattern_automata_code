function [exectime, Systems] = sensor_code(seg, Systems)

persistent y1 y2 y3 y4;

switch seg
 case 1
    y1 = ttAnalogIn(1);
    y2 = ttAnalogIn(2);
    y3 = ttAnalogIn(3);
    y4 = ttAnalogIn(4);
    exectime = sharedmem.wcet_snac;
 case 2
    msg1.data =y1;
    msg1.id = Systems{1}.tx;
    msg1.timestamp = ttCurrentTime();
    ttSendMsg(2, msg1, 64, Systems{1}.tx); % Send message (64 bits) to node 2 (controller)

    msg2.data =y2;
    msg2.id = Systems{2}.tx;
    msg2.timestamp = ttCurrentTime();
    ttSendMsg(2, msg2, 64, Systems{2}.tx); % Send message (64 bits) to node 2 (controller)

    msg3.data =y3;
    msg3.id = Systems{3}.tx;
    msg3.timestamp = ttCurrentTime();
    ttSendMsg(2, msg3, 64, Systems{3}.tx); % Send message (64 bits) to node 2 (controller)

    msg4.data =y4;
    msg4.id = Systems{4}.tx;
    msg4.timestamp = ttCurrentTime();
    ttSendMsg(2, msg4, 64, Systems{4}.tx); % Send message (64 bits) to node 2 (controller)
    
    exectime = sharedmem.wcet_snac;
 case 3
  exectime = -1;                         % finished
end
