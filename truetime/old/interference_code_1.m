function [exectime, data] = interference_code_1(seg, data)

BWshare = data;
ran = ttAnalogIn(1);
if ran < BWshare
  ttSendMsg(4, 4, 80);   % send 80 bits to myself
end

while ~isempty(ttGetMsg) % read old received messages (if any)
end

exectime = -1;
