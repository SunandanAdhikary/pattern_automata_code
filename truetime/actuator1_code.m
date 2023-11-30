function [exectime, data] = actuator1_code(seg, data)

persistent u

switch seg
 case 1
  u = ttGetMsg;
  exectime = 0.0005;
 otherwise
  if ~isempty(u)
    ttAnalogOut(1, u)
  else
    disp('Error: actuator received empty message!')
  end
  exectime = -1; % finished
end
